/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "modbus_tcp_minimal.h"
#include "modbus_rtu_master.h"
#include "telemetry_store.h"
#include "lwip/netif.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "EC25.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MB_PANEL_IP              "192.168.3.198"
#define MB_PANEL_PORT            502U
#define MB_PANEL_UNIT            1U
#define MB_RTU_SLAVE             6U
#define MB_RTU_BLOCK_START       1U
#define MB_RTU_BLOCK_COUNT       18U
#define MB_FAST_MS               7000U
#define MB_SLOW_MS               600000U
#define MB_START_MS              20000U
#define CELL_PRINT_MS            1000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId ModbusTaskHandle;
osThreadId CellularTaskHandle;
/* USER CODE BEGIN PV */

typedef struct
{
    uint16_t addr;
    bool     is_long;
} ModbusRtuPollReg_t;

static ModbusTcpClient_t g_modbus_client;
static ModbusRTU_Master_t g_rtu;

extern struct netif gnetif;

static const uint16_t mk3d_fast_tcp[] =
{
    1000, 1001, 1003, 1800, 2006, 2011, 2012, 2013, 2014, 2000, 2001, 2003, 2004
};

static const uint16_t mk3d_slow_tcp[] =
{
    1802, 1803, 1804, 1805, 1806, 1807, 1808,
    1810, 1811, 1812, 1813, 1814, 1815,
    1816, 1817, 1818, 1819, 1820, 1821,
    1822, 1823, 1824, 1825, 1826, 1827,
    1828, 1829, 1830, 1831, 1832, 1833,
    1834, 1835, 1836, 1837, 1838, 1839,
    1840, 1841, 1842, 1843, 1844, 1845,
    1856, 1857, 1858, 1859, 1860, 1861, 2015, 3028, 3029,
};

static const ModbusRtuPollReg_t fcjc_rtu_regs[] =
{
    {  1, false },
    {  2, false },
    {  6, true  },
    {  8, true  },
    { 12, false },
    { 17, false },
    { 18, false },
};

static char s_mb_json_buf[TELEMETRY_JSON_MAX];
static char s_cell_json_buf[TELEMETRY_JSON_MAX];
static uint16_t s_rtu_block[MB_RTU_BLOCK_COUNT];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_UART7_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartModbus(void const * argument);
void StartCellular(void const * argument);

/* USER CODE BEGIN PFP */

static bool Mb_NetReady(void);
static bool Mb_AppendTcpRegs(char *buf, size_t buflen, int *pos,
                             const uint16_t *addrs, uint32_t count);
static bool Mb_AppendRtuRegs(char *buf, size_t buflen, int *pos);
static void Mb_BuildAndPublishFast(void);
static void Mb_BuildAndPublishSlow(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
osMutexId printfMutex;
osMutexDef(PrintfMutex);

static bool Mb_NetReady(void)
{
    return netif_is_up(&gnetif) && netif_is_link_up(&gnetif);
}

static bool Mb_AppendTcpRegs(char *buf, size_t buflen, int *pos,
                             const uint16_t *addrs, uint32_t count)
{
    uint16_t value;

    for (uint32_t i = 0; i < count; i++)
    {
        int n;
        size_t avail;

        if (*pos < 0 || (size_t)*pos >= buflen)
        {
            return false;
        }
        avail = buflen - (size_t)*pos;

        if (ModbusTcp_ReadHoldingRegister(&g_modbus_client, addrs[i], &value) == 0)
        {
            n = snprintf(buf + *pos, avail,
                         "%s\"%u\":%u",
                         (i == 0U) ? "" : ",",
                         (unsigned)addrs[i],
                         (unsigned)value);
        }
        else
        {
            n = snprintf(buf + *pos, avail,
                         "%s\"%u\":null",
                         (i == 0U) ? "" : ",",
                         (unsigned)addrs[i]);
        }

        if (n < 0 || (size_t)n >= avail)
        {
            return false;
        }
        *pos += n;
    }

    return true;
}

static bool Mb_AppendRtuRegs(char *buf, size_t buflen, int *pos)
{
    uint32_t reg_count = (uint32_t)(sizeof(fcjc_rtu_regs) / sizeof(fcjc_rtu_regs[0]));

    if (!ModbusRTU_ReadHoldingRegisters(&g_rtu,
                                        MB_RTU_SLAVE,
                                        MB_RTU_BLOCK_START,
                                        MB_RTU_BLOCK_COUNT,
                                        s_rtu_block))
    {
        int n = snprintf(buf + *pos, buflen - (size_t)*pos, "\"error\":\"read_fail\"");
        if (n < 0 || (size_t)n >= buflen - (size_t)*pos)
        {
            return false;
        }
        *pos += n;
        return true;
    }

    for (uint32_t i = 0; i < reg_count; i++)
    {
        const ModbusRtuPollReg_t *reg = &fcjc_rtu_regs[i];
        uint32_t idx = (uint32_t)(reg->addr - MB_RTU_BLOCK_START);
        int n;
        size_t avail;

        if (idx >= MB_RTU_BLOCK_COUNT)
        {
            continue;
        }

        if (*pos < 0 || (size_t)*pos >= buflen)
        {
            return false;
        }
        avail = buflen - (size_t)*pos;

        if (reg->is_long && (idx + 1U) < MB_RTU_BLOCK_COUNT)
        {
            uint32_t value = ((uint32_t)s_rtu_block[idx] << 16) | s_rtu_block[idx + 1U];
            n = snprintf(buf + *pos, avail,
                         "%s\"%u\":%lu",
                         (i == 0U) ? "" : ",",
                         (unsigned)reg->addr,
                         (unsigned long)value);
        }
        else
        {
            n = snprintf(buf + *pos, avail,
                         "%s\"%u\":%u",
                         (i == 0U) ? "" : ",",
                         (unsigned)reg->addr,
                         (unsigned)s_rtu_block[idx]);
        }

        if (n < 0 || (size_t)n >= avail)
        {
            return false;
        }
        *pos += n;
    }

    return true;
}

static void Mb_BuildAndPublishFast(void)
{
    char *buf = s_mb_json_buf;
    int pos;
    int n;

    if (!Mb_NetReady())
    {
        ModbusTcp_Disconnect(&g_modbus_client);
        return;
    }

    pos = snprintf(buf, sizeof(s_mb_json_buf),
                   "{\"deviceid\":%d,\"config\":\"%s\",\"tcp_fast\":{",
                   TELEMETRY_DEVICE_ID,
                   TELEMETRY_CONFIG_NAME);
    if (pos < 0 || (size_t)pos >= sizeof(s_mb_json_buf))
    {
        return;
    }

    if (!Mb_AppendTcpRegs(buf, sizeof(s_mb_json_buf), &pos, mk3d_fast_tcp,
                          (uint32_t)(sizeof(mk3d_fast_tcp) / sizeof(mk3d_fast_tcp[0]))))
    {
        return;
    }

    n = snprintf(buf + pos, sizeof(s_mb_json_buf) - (size_t)pos, "},\"rtu\":{");
    if (n < 0 || (size_t)n >= sizeof(s_mb_json_buf) - (size_t)pos)
    {
        return;
    }
    pos += n;

    if (!Mb_AppendRtuRegs(buf, sizeof(s_mb_json_buf), &pos))
    {
        return;
    }

    n = snprintf(buf + pos, sizeof(s_mb_json_buf) - (size_t)pos, "}}");
    if (n < 0 || (size_t)n >= sizeof(s_mb_json_buf) - (size_t)pos)
    {
        return;
    }

    Telemetry_PublishFast(buf);
}

static void Mb_BuildAndPublishSlow(void)
{
    char *buf = s_mb_json_buf;
    int pos;
    int n;

    if (!Mb_NetReady())
    {
        ModbusTcp_Disconnect(&g_modbus_client);
        return;
    }

    pos = snprintf(buf, sizeof(s_mb_json_buf),
                   "{\"deviceid\":%d,\"config\":\"%s\",\"tcp_slow\":{",
                   TELEMETRY_DEVICE_ID,
                   TELEMETRY_CONFIG_NAME);
    if (pos < 0 || (size_t)pos >= sizeof(s_mb_json_buf))
    {
        return;
    }

    if (!Mb_AppendTcpRegs(buf, sizeof(s_mb_json_buf), &pos, mk3d_slow_tcp,
                          (uint32_t)(sizeof(mk3d_slow_tcp) / sizeof(mk3d_slow_tcp[0]))))
    {
        return;
    }

    n = snprintf(buf + pos, sizeof(s_mb_json_buf) - (size_t)pos, "}}");
    if (n < 0 || (size_t)n >= sizeof(s_mb_json_buf) - (size_t)pos)
    {
        return;
    }

    Telemetry_PublishSlow(buf);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  printfMutex = osMutexCreate(osMutex(PrintfMutex));
  Telemetry_Init();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityBelowNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ModbusTask */
  osThreadDef(ModbusTask, StartModbus, osPriorityAboveNormal, 0, 512);
  ModbusTaskHandle = osThreadCreate(osThread(ModbusTask), NULL);

  /* definition and creation of CellularTask */
  osThreadDef(CellularTask, StartCellular, osPriorityNormal, 0, 512);
  CellularTaskHandle = osThreadCreate(osThread(CellularTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 19200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart7.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  if (HAL_RS485Ex_Init(&huart7, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED4_Pin */
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
    if (printfMutex != NULL)
    {
        osMutexWait(printfMutex, osWaitForever);
    }

    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, 100);

    if (printfMutex != NULL)
    {
        osMutexRelease(printfMutex);
    }

    return len;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartModbus */
/**
* @brief Function implementing the ModbusTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartModbus */
void StartModbus(void const * argument)
{
  /* USER CODE BEGIN StartModbus */
  uint32_t next_fast_at = 0U;
  uint32_t next_slow_at = 0U;

  (void)argument;

  vTaskDelay(pdMS_TO_TICKS(MB_START_MS));

  {
    uint32_t t0 = HAL_GetTick();
    while (!Mb_NetReady())
    {
      if ((HAL_GetTick() - t0) > 30000U)
      {
        printf("{\"mb\":\"waiting for ethernet link timeout\"}\r\n");
        break;
      }
      osDelay(100);
    }
  }

  ModbusTcp_Init(&g_modbus_client, MB_PANEL_IP, MB_PANEL_PORT, (uint8_t)MB_PANEL_UNIT);
  ModbusRTU_Master_Init(&g_rtu, &huart7);

  printf("{\"mb\":\"started\",\"fast_ms\":%lu,\"slow_ms\":%lu,\"ip\":\"%s\"}\r\n",
         (unsigned long)MB_FAST_MS,
         (unsigned long)MB_SLOW_MS,
         MB_PANEL_IP);

  for (;;)
  {
    uint32_t now = HAL_GetTick();

    if (next_fast_at == 0U || (int32_t)(now - next_fast_at) >= 0)
    {
      Mb_BuildAndPublishFast();
      next_fast_at = HAL_GetTick() + MB_FAST_MS;
    }

    if (next_slow_at == 0U)
    {
      next_slow_at = now + MB_SLOW_MS;
    }
    else if ((int32_t)(now - next_slow_at) >= 0)
    {
      Mb_BuildAndPublishSlow();
      next_slow_at = HAL_GetTick() + MB_SLOW_MS;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
  /* USER CODE END StartModbus */
}

/* USER CODE BEGIN Header_StartCellular */
/**
* @brief Function implementing the CellularTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCellular */
void StartCellular(void const * argument)
{
  /* USER CODE BEGIN StartCellular */
  uint32_t seq;

  (void)argument;

  CELLSetup(&huart4);

  for (;;)
  {
	  GreenOn();
    if (Telemetry_TakeFast(s_cell_json_buf, sizeof(s_cell_json_buf), &seq))
    {
      printf("[CELL] fast seq=%lu: %s\r\n", (unsigned long)seq, s_cell_json_buf);
      if (EC25_PUBLISH(s_cell_json_buf, 1, &huart4))
      {
        Telemetry_AckFast(seq);
      }
      else
      {
        printf("[CELL] fast publish failed seq=%lu\r\n", (unsigned long)seq);
      }
    }

    if (Telemetry_TakeSlow(s_cell_json_buf, sizeof(s_cell_json_buf), &seq))
    {
      printf("[CELL] slow seq=%lu: %s\r\n", (unsigned long)seq, s_cell_json_buf);
      if (EC25_PUBLISH(s_cell_json_buf, 1, &huart4))
      {
        Telemetry_AckSlow(seq);
      }
      else
      {
        printf("[CELL] slow publish failed seq=%lu\r\n", (unsigned long)seq);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(CELL_PRINT_MS));
    GreenOFF();
  }
  /* USER CODE END StartCellular */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30020000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512B;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
