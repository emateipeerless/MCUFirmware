#include "modbus_tcp_minimal.h"

#include <string.h>
#include <stdio.h>

#include "main.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"

#define MB_TCP_INVALID_SOCK         (-1)
#define MB_TCP_LOCAL_PORT_BASE      0xC000U
#define MB_TCP_LOCAL_PORT_SPAN      0x3FFFU
#define MB_TCP_CONNECT_ATTEMPTS     4
#define MB_TCP_BKUP_MAGIC           0x4D425430U
#define MB_TCP_BKUP_ADDR            ((volatile MbTcpPersist_t *)0x38800000U)

typedef struct
{
    uint32_t magic;
    uint32_t boot_gen;
    uint16_t last_port;
    uint16_t reserved;
} MbTcpPersist_t;

static uint16_t s_try_port;
static uint8_t  s_try_port_valid;
static uint8_t  s_connect_fail_streak;
static uint8_t  s_bkup_ready;

static int recv_exact(int sock, uint8_t *buf, int len)
{
    int total = 0;
    int ret;

    while (total < len)
    {
        ret = recv(sock, buf + total, len - total, 0);
        if (ret <= 0)
        {
            return -1;
        }
        total += ret;
    }

    return 0;
}

static void mb_apply_timeouts(int sock)
{
    struct timeval tv;

    tv.tv_sec  = 2;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

static void mb_apply_keepalive(int sock)
{
    int ka = 1;

    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));

#if LWIP_TCP_KEEPALIVE
    {
        int idle  = 30;
        int intvl = 5;
        int cnt   = 3;

        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
    }
#endif
}

static uint32_t mb_uid_hash(void)
{
    return HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
}

static void mb_bkup_init(void)
{
    volatile MbTcpPersist_t *p = MB_TCP_BKUP_ADDR;

    if (s_bkup_ready)
    {
        return;
    }

    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_BKPRAM_CLK_ENABLE();

    if (p->magic != MB_TCP_BKUP_MAGIC)
    {
        p->magic     = MB_TCP_BKUP_MAGIC;
        p->boot_gen  = 0U;
        p->last_port = 0U;
        p->reserved  = 0U;
    }

    s_bkup_ready = 1U;
}

static uint16_t mb_wrap_port(uint32_t port)
{
    if (port < MB_TCP_LOCAL_PORT_BASE)
    {
        port = MB_TCP_LOCAL_PORT_BASE;
    }
    if (port > (uint32_t)(MB_TCP_LOCAL_PORT_BASE + MB_TCP_LOCAL_PORT_SPAN))
    {
        port = MB_TCP_LOCAL_PORT_BASE +
               ((port - MB_TCP_LOCAL_PORT_BASE) % (MB_TCP_LOCAL_PORT_SPAN + 1U));
    }
    return (uint16_t)port;
}

static uint16_t mb_seed_port(void)
{
    volatile MbTcpPersist_t *p = MB_TCP_BKUP_ADDR;
    uint32_t reset_flags;
    uint32_t seed;
    uint16_t port;

    mb_bkup_init();

    reset_flags = RCC->RSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();

    seed = p->boot_gen ^ mb_uid_hash() ^ reset_flags ^ HAL_GetTick();
    port = mb_wrap_port(MB_TCP_LOCAL_PORT_BASE + (seed % (MB_TCP_LOCAL_PORT_SPAN + 1U)));

    if (p->last_port != 0U)
    {
        uint16_t next = mb_wrap_port((uint32_t)p->last_port + 1U);

        if (next != port)
        {
            port = next;
        }
    }

    return port;
}

static void mb_persist_connected_port(uint16_t port)
{
    MB_TCP_BKUP_ADDR->last_port = port;
}

static void mb_reset_try_port(void)
{
    s_try_port_valid = 0U;
}

static uint16_t mb_next_try_port(void)
{
    if (!s_try_port_valid)
    {
        s_try_port       = mb_seed_port();
        s_try_port_valid = 1U;
        return s_try_port;
    }

    s_try_port = mb_wrap_port((uint32_t)s_try_port + 1U);
    return s_try_port;
}

static void mb_close_sock(ModbusTcpClient_t *client)
{
    if (client != NULL && client->sock >= 0)
    {
        shutdown(client->sock, SHUT_RDWR);
        closesocket(client->sock);
        client->sock = MB_TCP_INVALID_SOCK;
    }

    mb_reset_try_port();
}

void ModbusTcp_Disconnect(ModbusTcpClient_t *client)
{
    mb_close_sock(client);
}

static int mb_bind_local(int sock, uint16_t port)
{
    struct sockaddr_in local_addr;

    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family      = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port        = htons(port);

    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0)
    {
        return -1;
    }

    return 0;
}

static void mb_connect_backoff(void)
{
    uint32_t delay_ms;

    if (s_connect_fail_streak == 0U)
    {
        return;
    }

    delay_ms = 500U << (s_connect_fail_streak - 1U);
    if (delay_ms > 8000U)
    {
        delay_ms = 8000U;
    }

    HAL_Delay(delay_ms);
}

static int mb_connect_sock(ModbusTcpClient_t *client, int *out_sock)
{
    struct sockaddr_in server_addr;
    int attempt;

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family      = AF_INET;
    server_addr.sin_port        = htons(client->port);
    server_addr.sin_addr.s_addr = inet_addr(client->ip);

    mb_reset_try_port();

    for (attempt = 0; attempt < MB_TCP_CONNECT_ATTEMPTS; attempt++)
    {
        int sock;
        uint16_t local_port;
        int ret;

        local_port = mb_next_try_port();

        sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock < 0)
        {
            printf("Modbus: socket failed\r\n");
            return -2;
        }

        mb_apply_timeouts(sock);

        if (mb_bind_local(sock, local_port) != 0)
        {
            closesocket(sock);
            continue;
        }

        ret = connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
        if (ret >= 0)
        {
            mb_apply_keepalive(sock);
            mb_persist_connected_port(local_port);
            printf("Modbus: connected local port %u\r\n", (unsigned)local_port);
            *out_sock = sock;
            return 0;
        }

        closesocket(sock);
    }

    printf("Modbus: connect failed\r\n");
    return -3;
}

static int mb_ensure_connected(ModbusTcpClient_t *client)
{
    int ret;

    if (client->sock >= 0)
    {
        return 0;
    }

    mb_connect_backoff();

    ret = mb_connect_sock(client, &client->sock);
    if (ret == 0)
    {
        s_connect_fail_streak = 0U;
    }
    else if (s_connect_fail_streak < 8U)
    {
        s_connect_fail_streak++;
    }

    return ret;
}

static int mb_exchange(ModbusTcpClient_t *client,
                       int sock,
                       uint16_t reg_addr,
                       uint16_t *out_value)
{
    uint8_t req[12];
    uint8_t mbap[7];
    uint8_t pdu[256];

    uint16_t tx_id;
    uint16_t length_field;
    int remaining_len;
    int ret;

    tx_id = client->tx_id++;

    req[0] = (uint8_t)(tx_id >> 8);
    req[1] = (uint8_t)(tx_id & 0xFF);
    req[2] = 0x00;
    req[3] = 0x00;
    req[4] = 0x00;
    req[5] = 0x06;
    req[6] = client->unit_id;

    req[7]  = 0x03;
    req[8]  = (uint8_t)(reg_addr >> 8);
    req[9]  = (uint8_t)(reg_addr & 0xFF);
    req[10] = 0x00;
    req[11] = 0x01;

    ret = send(sock, req, sizeof(req), 0);
    if (ret != (int)sizeof(req))
    {
        printf("Modbus: send failed\r\n");
        return -4;
    }

    if (recv_exact(sock, mbap, 7) != 0)
    {
        printf("Modbus: failed to read MBAP\r\n");
        return -5;
    }

    if ((mbap[0] != req[0]) || (mbap[1] != req[1]))
    {
        printf("Modbus: TX ID mismatch\r\n");
        return -6;
    }

    if ((mbap[2] != 0x00) || (mbap[3] != 0x00))
    {
        printf("Modbus: bad protocol ID\r\n");
        return -7;
    }

    length_field = ((uint16_t)mbap[4] << 8) | mbap[5];

    if (length_field < 2)
    {
        printf("Modbus: invalid length\r\n");
        return -8;
    }

    remaining_len = (int)length_field - 1;
    if (remaining_len > (int)sizeof(pdu))
    {
        printf("Modbus: response too large\r\n");
        return -9;
    }

    if (recv_exact(sock, pdu, remaining_len) != 0)
    {
        printf("Modbus: failed to read PDU\r\n");
        return -10;
    }

    if (pdu[0] & 0x80)
    {
        printf("Modbus exception: FC=0x%02X EX=0x%02X\r\n", pdu[0], pdu[1]);
        return -11;
    }

    if ((pdu[0] != 0x03) || (pdu[1] != 0x02))
    {
        printf("Modbus: unexpected response FC=0x%02X BC=0x%02X\r\n", pdu[0], pdu[1]);
        return -12;
    }

    *out_value = ((uint16_t)pdu[2] << 8) | pdu[3];

    return 0;
}

void ModbusTcp_Init(ModbusTcpClient_t *client,
                    const char *ip,
                    uint16_t port,
                    uint8_t unit_id)
{
    mb_bkup_init();
    MB_TCP_BKUP_ADDR->boot_gen++;

    memset(client, 0, sizeof(*client));
    strncpy(client->ip, ip, sizeof(client->ip) - 1);
    client->port    = port;
    client->unit_id = unit_id;
    client->tx_id   = 1;
    client->sock    = MB_TCP_INVALID_SOCK;

    s_connect_fail_streak = 0U;
    mb_reset_try_port();
}

int ModbusTcp_ReadHoldingRegister(ModbusTcpClient_t *client,
                                  uint16_t reg_addr,
                                  uint16_t *out_value)
{
    int ret;

    if ((client == NULL) || (out_value == NULL))
    {
        return -1;
    }

    ret = mb_ensure_connected(client);
    if (ret != 0)
    {
        return ret;
    }

    ret = mb_exchange(client, client->sock, reg_addr, out_value);
    if (ret != 0)
    {
        mb_close_sock(client);
    }
    else
    {
        s_connect_fail_streak = 0U;
    }

    return ret;
}
