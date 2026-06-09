#include "telemetry_store.h"

#include "cmsis_os.h"
#include <string.h>

typedef struct
{
    char     fast[TELEMETRY_JSON_MAX];
    char     slow[TELEMETRY_JSON_MAX];
    uint32_t fast_seq;
    uint32_t slow_seq;
    uint32_t fast_acked_seq;
    uint32_t slow_acked_seq;
    osMutexId mutex;
} TelemetryStore_t;

static TelemetryStore_t s_store;
osMutexDef(TelemetryMutex);

void Telemetry_Init(void)
{
    memset(&s_store, 0, sizeof(s_store));
    s_store.mutex = osMutexCreate(osMutex(TelemetryMutex));
}

bool Telemetry_PublishFast(const char *json)
{
    if (json == NULL || s_store.mutex == NULL)
    {
        return false;
    }

    osMutexWait(s_store.mutex, osWaitForever);
    strncpy(s_store.fast, json, sizeof(s_store.fast) - 1U);
    s_store.fast[sizeof(s_store.fast) - 1U] = '\0';
    s_store.fast_seq++;
    osMutexRelease(s_store.mutex);
    return true;
}

bool Telemetry_PublishSlow(const char *json)
{
    if (json == NULL || s_store.mutex == NULL)
    {
        return false;
    }

    osMutexWait(s_store.mutex, osWaitForever);
    strncpy(s_store.slow, json, sizeof(s_store.slow) - 1U);
    s_store.slow[sizeof(s_store.slow) - 1U] = '\0';
    s_store.slow_seq++;
    osMutexRelease(s_store.mutex);
    return true;
}

bool Telemetry_GetFast(char *dst, size_t dst_len, uint32_t *seq_out)
{
    if (dst == NULL || dst_len == 0U || s_store.mutex == NULL)
    {
        return false;
    }

    osMutexWait(s_store.mutex, osWaitForever);
    if (s_store.fast_seq == 0U)
    {
        osMutexRelease(s_store.mutex);
        return false;
    }

    strncpy(dst, s_store.fast, dst_len - 1U);
    dst[dst_len - 1U] = '\0';
    if (seq_out != NULL)
    {
        *seq_out = s_store.fast_seq;
    }
    osMutexRelease(s_store.mutex);
    return true;
}

bool Telemetry_GetSlow(char *dst, size_t dst_len, uint32_t *seq_out)
{
    if (dst == NULL || dst_len == 0U || s_store.mutex == NULL)
    {
        return false;
    }

    osMutexWait(s_store.mutex, osWaitForever);
    if (s_store.slow_seq == 0U)
    {
        osMutexRelease(s_store.mutex);
        return false;
    }

    strncpy(dst, s_store.slow, dst_len - 1U);
    dst[dst_len - 1U] = '\0';
    if (seq_out != NULL)
    {
        *seq_out = s_store.slow_seq;
    }
    osMutexRelease(s_store.mutex);
    return true;
}

bool Telemetry_TakeFast(char *dst, size_t dst_len, uint32_t *seq_out)
{
    if (dst == NULL || dst_len == 0U || s_store.mutex == NULL)
    {
        return false;
    }

    osMutexWait(s_store.mutex, osWaitForever);
    if (s_store.fast_seq == 0U || s_store.fast_seq <= s_store.fast_acked_seq)
    {
        osMutexRelease(s_store.mutex);
        return false;
    }

    strncpy(dst, s_store.fast, dst_len - 1U);
    dst[dst_len - 1U] = '\0';
    if (seq_out != NULL)
    {
        *seq_out = s_store.fast_seq;
    }
    osMutexRelease(s_store.mutex);
    return true;
}

bool Telemetry_TakeSlow(char *dst, size_t dst_len, uint32_t *seq_out)
{
    if (dst == NULL || dst_len == 0U || s_store.mutex == NULL)
    {
        return false;
    }

    osMutexWait(s_store.mutex, osWaitForever);
    if (s_store.slow_seq == 0U || s_store.slow_seq <= s_store.slow_acked_seq)
    {
        osMutexRelease(s_store.mutex);
        return false;
    }

    strncpy(dst, s_store.slow, dst_len - 1U);
    dst[dst_len - 1U] = '\0';
    if (seq_out != NULL)
    {
        *seq_out = s_store.slow_seq;
    }
    osMutexRelease(s_store.mutex);
    return true;
}

void Telemetry_AckFast(uint32_t seq)
{
    if (s_store.mutex == NULL)
    {
        return;
    }

    osMutexWait(s_store.mutex, osWaitForever);
    if (seq > s_store.fast_acked_seq)
    {
        s_store.fast_acked_seq = seq;
    }
    osMutexRelease(s_store.mutex);
}

void Telemetry_AckSlow(uint32_t seq)
{
    if (s_store.mutex == NULL)
    {
        return;
    }

    osMutexWait(s_store.mutex, osWaitForever);
    if (seq > s_store.slow_acked_seq)
    {
        s_store.slow_acked_seq = seq;
    }
    osMutexRelease(s_store.mutex);
}
