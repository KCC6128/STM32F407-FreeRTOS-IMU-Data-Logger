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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  uint32_t sequence;
  uint32_t tick_ms;

  uint32_t print_dropped_total;
  uint32_t logger_dropped_total;
  uint32_t spi_error_total;

  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;

} IMUData_t;

typedef enum
{
  LOGGER_MESSAGE_SAMPLE = 0,
  LOGGER_MESSAGE_DUMP,
  LOGGER_MESSAGE_CLEAR

} LoggerMessageType_t;

typedef struct
{
  LoggerMessageType_t type;

  union
  {
    IMUData_t sample;
    uint32_t dump_count;

  } payload;

} LoggerMessage_t;

typedef struct
{
  /* Magic number to distinguish valid data from erased flash memory. */
  uint32_t magic;

  uint32_t sequence;
  uint32_t tick_ms;

  uint32_t print_dropped_total;
  uint32_t logger_dropped_total;
  uint32_t spi_error_total;

  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;

  /* 
   * Padding to align struct to exactly 32 bytes.
   * Perfect fit for W25Q64_PAGE_SIZE: 256 Bytes / 32 Bytes = exactly 8 records per Page.
   */
  uint16_t reserved;

} FlashRecord_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ====================================================================
 * W25Q64 Flash Hardware Specifications (From Datasheet)
 * - PAGE   : 256 Bytes (Maximum size for a single Write command)
 * - SECTOR : 4096 Bytes (Minimum size for a single Erase command)
 * ==================================================================== */
#define W25Q64_PAGE_SIZE              256U
#define W25Q64_SECTOR_SIZE            4096U

/* MPU6500 Registers & SPI */
#define WHO_AM_I_MPU6500              0x75
#define PWR_MGMT_1                    0x6B
#define ACCEL_XOUT_H                  0x3B
#define ACCEL_DATA_LEN                6U
#define ACCEL_SPI_LEN                 7U

#define MPU6500_CS_PORT               GPIOE
#define MPU6500_CS_PIN                GPIO_PIN_3

/* W25Q64 Commands & Status */
#define W25Q64_CMD_JEDEC_ID           0x9FU
#define W25Q64_CMD_WRITE_ENABLE       0x06U
#define W25Q64_CMD_READ_STATUS_1      0x05U
#define W25Q64_CMD_SECTOR_ERASE_4K    0x20U
#define W25Q64_CMD_PAGE_PROGRAM       0x02U
#define W25Q64_CMD_READ_DATA          0x03U

#define W25Q64_STATUS_BUSY            0x01U

/* W25Q64 Test Parameters */
#define W25Q64_TEST_ADDRESS           0x007FF000UL
#define W25Q64_TEST_DATA_LEN          8U

#define W25Q64_ERASE_TIMEOUT_MS       2000U
#define W25Q64_PROGRAM_TIMEOUT_MS     100U

/* ====================================================================
 * Logger System Configuration
 * ==================================================================== */
#define FLASH_RECORD_MAGIC            0x31554D49UL  /* "IMU1" */
#define FLASH_LOG_BASE_ADDRESS        0x000000UL    /* Start at Sector 0 */

/* 8-Sector test config: 8 * 4096 = 32KB total */
#define FLASH_LOG_SECTOR_COUNT        8U
#define FLASH_LOG_TOTAL_SIZE          (FLASH_LOG_SECTOR_COUNT * W25Q64_SECTOR_SIZE)

/* Max records: 32768 / 32 = 1024 records */
#define FLASH_LOG_MAX_RECORDS         (FLASH_LOG_TOTAL_SIZE / sizeof(FlashRecord_t))

/* Thread Flags for CommandTask */
#define COMMAND_BUTTON_FLAG           (1U << 0)  /* Event: PA0 physical button pressed */
#define COMMAND_UART_RX_FLAG          (1U << 1)  /* Event: UART command completely received */

/* Hardware & Buffer Settings */
#define BUTTON_DEBOUNCE_MS            200U       /* Button debounce time in milliseconds */
#define UART_COMMAND_BUFFER_SIZE      32U        /* Max character length for a UART command */

/* Task health flags */
#define HEALTH_IMU_FLAG          (1U << 0)
#define HEALTH_PRINT_FLAG        (1U << 1)
#define HEALTH_LOGGER_FLAG       (1U << 2)
#define HEALTH_COMMAND_FLAG      (1U << 3)
#define HEALTH_HEARTBEAT_FLAG    (1U << 4)

#define HEALTH_ALL_FLAGS         (HEALTH_IMU_FLAG | HEALTH_PRINT_FLAG | HEALTH_LOGGER_FLAG | HEALTH_COMMAND_FLAG | HEALTH_HEARTBEAT_FLAG)

#define TASK_IDLE_TIMEOUT_MS      500U
#define HEALTH_WAIT_TIMEOUT_MS    2500U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart5;

/* Definitions for heartbeatTask */
osThreadId_t heartbeatTaskHandle;
const osThreadAttr_t heartbeatTask_attributes = {
  .name = "heartbeatTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for printTask */
osThreadId_t printTaskHandle;
const osThreadAttr_t printTask_attributes = {
  .name = "printTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for loggerTask */
osThreadId_t loggerTaskHandle;
const osThreadAttr_t loggerTask_attributes = {
  .name = "loggerTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for commandTask */
osThreadId_t commandTaskHandle;
const osThreadAttr_t commandTask_attributes = {
  .name = "commandTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for supervisorTask */
osThreadId_t supervisorTaskHandle;
const osThreadAttr_t supervisorTask_attributes = {
  .name = "supervisorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for imuQueue */
osMessageQueueId_t imuQueueHandle;
const osMessageQueueAttr_t imuQueue_attributes = {
  .name = "imuQueue"
};
/* Definitions for loggerQueue */
osMessageQueueId_t loggerQueueHandle;
const osMessageQueueAttr_t loggerQueue_attributes = {
  .name = "loggerQueue"
};
/* Definitions for spiMutex */
osMutexId_t spiMutexHandle;
const osMutexAttr_t spiMutex_attributes = {
  .name = "spiMutex"
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};
/* Definitions for systemHealthEvent */
osEventFlagsId_t systemHealthEventHandle;
const osEventFlagsAttr_t systemHealthEvent_attributes = {
  .name = "systemHealthEvent"
};
/* USER CODE BEGIN PV */

static volatile uint8_t loggingEnabled = 0U;
static volatile uint8_t loggerReady = 0U;
static volatile uint8_t loggerFull = 0U;
static volatile uint8_t loggerBusy = 0U;
static volatile uint32_t loggerRecordCount = 0U;

static uint8_t uartRxByte = 0U;

static char uartCommandBuffer[UART_COMMAND_BUFFER_SIZE];

static volatile uint32_t uartCommandLength = 0U;
static volatile uint8_t uartCommandReady = 0U;

static uint8_t resetByIWDG = 0U;

static volatile uint8_t injectImuHealthFault = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART5_Init(void);
static void MX_IWDG_Init(void);
void StartHeartbeatTask(void *argument);
void StartIMUTask(void *argument);
void StartPrintTask(void *argument);
void StartLoggerTask(void *argument);
void StartCommandTask(void *argument);
void StartSupervisorTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static HAL_StatusTypeDef SPI2_Lock(void)
{
  if (osMutexAcquire(spiMutexHandle, osWaitForever) == osOK)
  {
    return HAL_OK;
  }

  return HAL_ERROR;
}

static void SPI2_Unlock(void)
{
  osMutexRelease(spiMutexHandle);
}

static void UART_Print(const char *str)
{
  if (osMutexAcquire(uartMutexHandle, osWaitForever) == osOK)
  {
    HAL_UART_Transmit(&huart5, (uint8_t *)str, (uint16_t)strlen(str), 100);

    osMutexRelease(uartMutexHandle);
  }
}

static void MPU6500_CS_LOW(void)
{
  HAL_GPIO_WritePin(MPU6500_CS_PORT, MPU6500_CS_PIN, GPIO_PIN_RESET);
}

static void MPU6500_CS_HIGH(void)
{
  HAL_GPIO_WritePin(MPU6500_CS_PORT, MPU6500_CS_PIN, GPIO_PIN_SET);
}

static HAL_StatusTypeDef MPU6500_ReadReg(uint8_t reg, uint8_t *data)
{
  HAL_StatusTypeDef ret;
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = reg | 0x80;   // read command
  tx[1] = 0x00;         // dummy byte

  if (SPI2_Lock() != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  MPU6500_CS_LOW();
  ret = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);
  MPU6500_CS_HIGH();
  
  SPI2_Unlock();

  if (ret == HAL_OK)
  {
    *data = rx[1];
  }

  return ret;
}

static HAL_StatusTypeDef MPU6500_WriteReg(uint8_t reg, uint8_t data)
{
  HAL_StatusTypeDef ret;
  uint8_t tx[2];

  tx[0] = reg & 0x7F;   // write command
  tx[1] = data;

  if (SPI2_Lock() != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  MPU6500_CS_LOW();
  ret = HAL_SPI_Transmit(&hspi2, tx, 2, 100);
  MPU6500_CS_HIGH();
  
  SPI2_Unlock();

  return ret;
}

static HAL_StatusTypeDef MPU6500_ReadAccel(int16_t *accelX, int16_t *accelY, int16_t *accelZ)
{
  HAL_StatusTypeDef status;

  uint8_t tx[ACCEL_SPI_LEN] = {0};
  uint8_t rx[ACCEL_SPI_LEN] = {0};

  tx[0] = ACCEL_XOUT_H | 0x80U;      // read 

  if (SPI2_Lock() != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  MPU6500_CS_LOW();
  status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, ACCEL_SPI_LEN, 100);
  MPU6500_CS_HIGH();
  
  SPI2_Unlock();

  if (status == HAL_OK)
  {
    *accelX = (int16_t)(((uint16_t)rx[1] << 8) | rx[2]);

    *accelY = (int16_t)(((uint16_t)rx[3] << 8) | rx[4]);

    *accelZ = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);
  }

  return status;
}

static void W25Q64_CS_LOW(void)
{
  HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_RESET);
}

static void W25Q64_CS_HIGH(void)
{
  HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef W25Q64_ReadJEDECID(uint8_t *manufacturer, uint8_t *memoryType, uint8_t *capacity)
{
  HAL_StatusTypeDef status;

  uint8_t tx[4] = {W25Q64_CMD_JEDEC_ID, 0x00U, 0x00U, 0x00U};

  uint8_t rx[4] = {0};

  if (SPI2_Lock() != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  W25Q64_CS_LOW();
  status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, sizeof(tx), 100);
  W25Q64_CS_HIGH();

  SPI2_Unlock();

  if (status == HAL_OK)
  {
    /*
     * rx[0] is received while sending the command.
     * Valid JEDEC ID bytes start from rx[1].
     */
    *manufacturer = rx[1];
    *memoryType   = rx[2];
    *capacity     = rx[3];
  }

  return status;
}

// Write Enable
static HAL_StatusTypeDef W25Q64_WriteEnable_NoLock(void)
{
  HAL_StatusTypeDef status;
  uint8_t command = W25Q64_CMD_WRITE_ENABLE;

  W25Q64_CS_LOW();

  status = HAL_SPI_Transmit(&hspi2, &command, 1, 100);

  W25Q64_CS_HIGH();

  return status;
}

// Read Status Register-1
static HAL_StatusTypeDef W25Q64_ReadStatus1(uint8_t *statusRegister)
{
  HAL_StatusTypeDef status;

  uint8_t tx[2] = {W25Q64_CMD_READ_STATUS_1, 0x00U};

  uint8_t rx[2] = {0};

  if (SPI2_Lock() != HAL_OK)
  {
    return HAL_ERROR;
  }

  W25Q64_CS_LOW();

  status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);

  W25Q64_CS_HIGH();

  SPI2_Unlock();

  if (status == HAL_OK)
  {
    *statusRegister = rx[1];
  }

  return status;
}

// Wait While Busy
static HAL_StatusTypeDef W25Q64_WaitWhileBusy(uint32_t timeoutMs)
{
  HAL_StatusTypeDef status;
  uint8_t statusRegister = 0;

  uint32_t startTick = osKernelGetTickCount();

  for (;;)
  {
    status = W25Q64_ReadStatus1(&statusRegister);

    if (status != HAL_OK)
    {
      return status;
    }

    if ((statusRegister & W25Q64_STATUS_BUSY) == 0U)
    {
      return HAL_OK;
    }

    if ((osKernelGetTickCount() - startTick) >= timeoutMs)
    {
      return HAL_TIMEOUT;
    }

    /*
     * Let other tasks run while Flash is busy.
     */
    osDelay(1U);
  }
}

// Sector Erase
static HAL_StatusTypeDef W25Q64_EraseSector4K(uint32_t address)
{
  HAL_StatusTypeDef status;

  uint8_t command[4] = {W25Q64_CMD_SECTOR_ERASE_4K, (uint8_t)(address >> 16), (uint8_t)(address >> 8), (uint8_t)address};

  if (SPI2_Lock() != HAL_OK)
  {
    return HAL_ERROR;
  }

  /*
   * Keep the bus locked between
   * Write Enable and Sector Erase.
   */
  status = W25Q64_WriteEnable_NoLock();

  if (status == HAL_OK)
  {
    W25Q64_CS_LOW();

    status = HAL_SPI_Transmit(&hspi2, command, sizeof(command), 100);

    W25Q64_CS_HIGH();
  }

  SPI2_Unlock();

  if (status == HAL_OK)
  {
    status = W25Q64_WaitWhileBusy(W25Q64_ERASE_TIMEOUT_MS);
  }

  return status;
}

//Page Program
static HAL_StatusTypeDef W25Q64_PageProgram(uint32_t address, const uint8_t *data, uint16_t length)
{
  HAL_StatusTypeDef status;

  uint8_t command[4] = {W25Q64_CMD_PAGE_PROGRAM, (uint8_t)(address >> 16), (uint8_t)(address >> 8), (uint8_t)address};

  if ((length == 0U) || (length > 256U))
  {
    return HAL_ERROR;
  }

  /*
   * Data must stay inside one page.
   */
  if (((address & 0xFFU) + length) > 256U)
  {
    return HAL_ERROR;
  }

  if (SPI2_Lock() != HAL_OK)
  {
    return HAL_ERROR;
  }

  status = W25Q64_WriteEnable_NoLock();

  if (status == HAL_OK)
  {
    W25Q64_CS_LOW();

    status = HAL_SPI_Transmit(&hspi2, command, sizeof(command), 100);

    if (status == HAL_OK)
    {
      status = HAL_SPI_Transmit(&hspi2, (uint8_t *)data, length, 100);
    }

    W25Q64_CS_HIGH();
  }

  SPI2_Unlock();

  if (status == HAL_OK)
  {
    status = W25Q64_WaitWhileBusy(W25Q64_PROGRAM_TIMEOUT_MS);
  }

  return status;
}

// Read Data
static HAL_StatusTypeDef W25Q64_ReadData(uint32_t address, uint8_t *data, uint16_t length)
{
  HAL_StatusTypeDef status;

  uint8_t command[4] = {W25Q64_CMD_READ_DATA, (uint8_t)(address >> 16), (uint8_t)(address >> 8), (uint8_t)address};

  if (SPI2_Lock() != HAL_OK)
  {
    return HAL_ERROR;
  }

  W25Q64_CS_LOW();

  status = HAL_SPI_Transmit(&hspi2, command, sizeof(command), 100);

  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(&hspi2, data, length, 100);
  }

  W25Q64_CS_HIGH();

  SPI2_Unlock();

  return status;
}

// Erase the complete logger region
static HAL_StatusTypeDef Logger_EraseLogRegion(void)
{
  HAL_StatusTypeDef status;

  uint32_t sectorIndex;
  uint32_t sectorAddress;

  for (sectorIndex = 0U; sectorIndex < FLASH_LOG_SECTOR_COUNT; sectorIndex++)
  {
    sectorAddress = FLASH_LOG_BASE_ADDRESS + (sectorIndex * W25Q64_SECTOR_SIZE);

    status = W25Q64_EraseSector4K(sectorAddress);
  
    /*
     * Report progress during erase.
     */
    osEventFlagsSet(systemHealthEventHandle, HEALTH_LOGGER_FLAG);

    if (status != HAL_OK)
    {
      return status;
    }
  }

  return HAL_OK;
}

// DumpLatestRecords
static void Logger_DumpLatestRecords(uint32_t requestedCount)
{
  FlashRecord_t record;

  HAL_StatusTypeDef status;

  uint32_t totalRecords;
  uint32_t dumpCount;
  uint32_t startIndex;
  uint32_t recordIndex;
  uint32_t address;

  int32_t accelX_mg;
  int32_t accelY_mg;
  int32_t accelZ_mg;

  char message[192];

  totalRecords = loggerRecordCount;

  if (totalRecords == 0U)
  {
    UART_Print("No records to dump.\r\n");
    return;
  }

  dumpCount = requestedCount;

  if (dumpCount > totalRecords)
  {
    dumpCount = totalRecords;
  }

  startIndex = totalRecords - dumpCount;

  snprintf(message, sizeof(message), "Dump last %lu records:\r\n", (unsigned long)dumpCount);

  UART_Print(message);

  for (recordIndex = startIndex; recordIndex < totalRecords; recordIndex++)
  {
    address = FLASH_LOG_BASE_ADDRESS + (recordIndex * sizeof(FlashRecord_t));

    status = W25Q64_ReadData(address, (uint8_t *)&record, sizeof(record));

    if (status != HAL_OK)
    {
      UART_Print("Flash dump read failed.\r\n");

      return;
    }

    if (record.magic != FLASH_RECORD_MAGIC)
    {
      snprintf(message, sizeof(message), "LOG[%lu] invalid magic\r\n", (unsigned long)recordIndex);

      UART_Print(message);
      return;
    }

    accelX_mg = ((int32_t)record.accel_x * 1000L) / 16384L;

    accelY_mg = ((int32_t)record.accel_y * 1000L) / 16384L;

    accelZ_mg = ((int32_t)record.accel_z * 1000L) / 16384L;

    snprintf(message, sizeof(message), "LOG[%lu] SEQ=%lu, T=%lu, AX=%ld, AY=%ld, AZ=%ld\r\n", (unsigned long)recordIndex, (unsigned long)record.sequence, (unsigned long)record.tick_ms, (long)accelX_mg, (long)accelY_mg, (long)accelZ_mg);

    UART_Print(message);
    
    osEventFlagsSet(systemHealthEventHandle, HEALTH_LOGGER_FLAG);
  }

  UART_Print("Dump complete.\r\n");
}

// Command Start
static void Command_StartLogging(void)
{
  char message[96];

  if (loggerBusy != 0U)
  {
    UART_Print("Logger is busy.\r\n");
  }
  else if (loggerReady == 0U)
  {
    UART_Print("Logger is not ready.\r\n");
  }
  else if (loggerFull != 0U)
  {
    snprintf(message, sizeof(message), "Logger is full. Records=%lu\r\n", (unsigned long)loggerRecordCount);

    UART_Print(message);
  }
  else if (loggingEnabled != 0U)
  {
    UART_Print("Logging is already running.\r\n");
  }
  else
  {
    loggingEnabled = 1U;

    UART_Print("Logging started.\r\n");
  }
}

// Command Stop
static void Command_StopLogging(void)
{
  char message[96];

  if (loggingEnabled == 0U)
  {
    UART_Print("Logging is already stopped.\r\n");
  }
  else
  {
    loggingEnabled = 0U;

    snprintf(message, sizeof(message), "Logging stopped. Records=%lu\r\n", (unsigned long)loggerRecordCount);

    UART_Print(message);
  }
}

// PA0(Button) Toggle
static void Command_ToggleLogging(void)
{
  if (loggingEnabled == 0U)
  {
    Command_StartLogging();
  }
  else
  {
    Command_StopLogging();
  }
}

// Command Status
static void Command_PrintStatus(void)
{
  char message[192];

  uint32_t imuQueueCount;
  uint32_t loggerQueueCount;

  imuQueueCount = osMessageQueueGetCount(imuQueueHandle);

  loggerQueueCount = osMessageQueueGetCount(loggerQueueHandle);

  snprintf(message, sizeof(message), "[STATUS] ready=%u, busy=%u, logging=%u, full=%u, records=%lu/%lu, imuQ=%lu, loggerQ=%lu\r\n", (unsigned int)loggerReady, (unsigned int)loggerBusy, (unsigned int)loggingEnabled, (unsigned int)loggerFull, (unsigned long)loggerRecordCount, (unsigned long)FLASH_LOG_MAX_RECORDS, (unsigned long)imuQueueCount, (unsigned long)loggerQueueCount);

  UART_Print(message);
}

// Command Dump
static void Command_RequestDump(uint32_t count)
{
  LoggerMessage_t loggerMessage = {0};

  if (loggerBusy != 0U)
  {
    UART_Print("Logger is busy.\r\n");
    return;
  }
  
  if (loggerReady == 0U)
  {
    UART_Print("Logger is not ready.\r\n");
    return;
  }

  if (loggingEnabled != 0U)
  {
    UART_Print("Stop logging before dump.\r\n");

    return;
  }

  if ((count == 0U) || (count > 50U))
  {
    UART_Print("Dump count must be 1 to 50.\r\n");

    return;
  }

  loggerMessage.type = LOGGER_MESSAGE_DUMP;

  loggerMessage.payload.dump_count = count;

  if (osMessageQueuePut(loggerQueueHandle, &loggerMessage, 0U, osWaitForever) == osOK)
  {
    UART_Print("Dump requested.\r\n");
  }
  else
  {
    UART_Print("Dump request failed.\r\n");
  }
}

// Command Clear
static void Command_RequestClear(void)
{
  LoggerMessage_t loggerMessage = {0};

  if (loggerBusy != 0U)
  {
    UART_Print("Logger is busy.\r\n");
    return;
  }

  if (loggerReady == 0U)
  {
    UART_Print("Logger is not ready.\r\n");
    return;
  }

  if (loggingEnabled != 0U)
  {
    UART_Print("Stop logging before clear.\r\n");

    return;
  }

  /*
   * Block new start and dump commands.
   */
  loggerBusy = 1U;
  loggerReady = 0U;

  loggerMessage.type = LOGGER_MESSAGE_CLEAR;

  if (osMessageQueuePut(loggerQueueHandle, &loggerMessage, 0U, osWaitForever) == osOK)
  {
    UART_Print("Clear requested.\r\n");
  }
  else
  {
    loggerBusy = 0U;
    loggerReady = 1U;

    UART_Print("Clear request failed.\r\n");
  }
}

// Command ImuFault
static void Command_InjectImuFault(void)
{
  if (injectImuHealthFault != 0U)
  {
    UART_Print("IMU health fault is already active.\r\n");

    return;
  }

  injectImuHealthFault = 1U;

  UART_Print("Fault injected: IMU health report disabled.\r\n");
}

// Command
static void Command_ProcessLine(const char *command)
{
  if (strcmp(command, "start") == 0)
  {
    Command_StartLogging();
  }
  else if (strcmp(command, "stop") == 0)
  {
    Command_StopLogging();
  }
  else if (strcmp(command, "status") == 0)
  {
    Command_PrintStatus();
  }
  else if (strcmp(command, "help") == 0)
  {
    UART_Print("Commands: start, stop, status, dump N, clear, fault imu, help\r\n");
  }
  else if (strcmp(command, "clear") == 0)
  {
    Command_RequestClear();
  }
  else if (strcmp(command, "fault imu") == 0)
  {
    Command_InjectImuFault();
  }
  else if (strncmp(command, "dump ", 5U) == 0)
  {
    char *endPointer;
    unsigned long count;

    count = strtoul(command + 5, &endPointer, 10);

    if ((*endPointer == '\0') && (count >= 1UL) && (count <= 50UL))
    {
      Command_RequestDump((uint32_t)count);
    }
    else
    {
      UART_Print("Usage: dump N, N=1..50\r\n");
    }
  }
  else
  {
    UART_Print("Unknown command. Type help.\r\n");
  }
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    resetByIWDG = 1U;
  }

  __HAL_RCC_CLEAR_RESET_FLAGS();
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_UART5_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of spiMutex */
  spiMutexHandle = osMutexNew(&spiMutex_attributes);

  /* creation of uartMutex */
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of imuQueue */
  imuQueueHandle = osMessageQueueNew (16, sizeof(IMUData_t), &imuQueue_attributes);

  /* creation of loggerQueue */
  loggerQueueHandle = osMessageQueueNew (32, sizeof(LoggerMessage_t), &loggerQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of heartbeatTask */
  heartbeatTaskHandle = osThreadNew(StartHeartbeatTask, NULL, &heartbeatTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartIMUTask, NULL, &imuTask_attributes);

  /* creation of printTask */
  printTaskHandle = osThreadNew(StartPrintTask, NULL, &printTask_attributes);

  /* creation of loggerTask */
  loggerTaskHandle = osThreadNew(StartLoggerTask, NULL, &loggerTask_attributes);

  /* creation of commandTask */
  commandTaskHandle = osThreadNew(StartCommandTask, NULL, &commandTask_attributes);

  /* creation of supervisorTask */
  supervisorTaskHandle = osThreadNew(StartSupervisorTask, NULL, &supervisorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of systemHealthEvent */
  systemHealthEventHandle = osEventFlagsNew(&systemHealthEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 1249;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, MPU6500_CS_Pin|W25Q64_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin|ORANGE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MPU6500_CS_Pin W25Q64_CS_Pin */
  GPIO_InitStruct.Pin = MPU6500_CS_Pin|W25Q64_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_LED_Pin ORANGE_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin|ORANGE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t lastButtonTick = 0U;
  uint32_t currentTick;

  if (GPIO_Pin == USER_BUTTON_Pin)
  {
    currentTick = HAL_GetTick();

    if ((currentTick - lastButtonTick) >= BUTTON_DEBOUNCE_MS)
    {
      lastButtonTick = currentTick;

      if (commandTaskHandle != NULL)
      {
        osThreadFlagsSet(commandTaskHandle, COMMAND_BUTTON_FLAG);
      }
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    /*
     * Do not overwrite a pending command.
     */
    if (uartCommandReady == 0U)
    {
      if ((uartRxByte == '\r') || (uartRxByte == '\n'))
      {
        if (uartCommandLength > 0U)
        {
          uartCommandBuffer[uartCommandLength] = '\0';

          uartCommandReady = 1U;

          if (commandTaskHandle != NULL)
          {
            osThreadFlagsSet(commandTaskHandle, COMMAND_UART_RX_FLAG);
          }
        }
      }
      else
      {
        if (uartCommandLength <(UART_COMMAND_BUFFER_SIZE - 1U))
        {
          uartCommandBuffer[uartCommandLength] = (char)uartRxByte;

          uartCommandLength++;
        }
        else
        {
          /*
           * Reset an oversized command.
           */
          uartCommandLength = 0U;
        }
      }
    }

    /*
     * Receive the next byte.
     */
    HAL_UART_Receive_IT(&huart5, &uartRxByte, 1U);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartHeartbeatTask */
/**
  * @brief  Function implementing the heartbeatTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartHeartbeatTask */
void StartHeartbeatTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;)
  {
    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
    
    /* Report task health. */
    osEventFlagsSet(systemHealthEventHandle, HEALTH_HEARTBEAT_FLAG);

    osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument)
{
  /* USER CODE BEGIN StartIMUTask */
  /* Infinite loop */
  uint8_t readData = 0;
  HAL_StatusTypeDef status;
  char message[96];
  int messageLength;

  MPU6500_CS_HIGH();

  osDelay(100);

  /* 1. Read WHO_AM_I */
  status = MPU6500_ReadReg(WHO_AM_I_MPU6500, &readData);

  if (status == HAL_OK)
  {
    messageLength = snprintf(message, sizeof(message), "\r\nWHO_AM_I = 0x%02X\r\n",readData);

    UART_Print(message);

    if (readData == 0x70)
    {
      UART_Print("MPU6500 detected.\r\n");
    }
    else
    {
      UART_Print("WHO_AM_I is not 0x70.\r\n");
    }
  }
  else
  {
    UART_Print("Read WHO_AM_I failed.\r\n");
  }

  /* 2. Wake up MPU6500 */
  status = MPU6500_WriteReg(PWR_MGMT_1, 0x00);

  if (status == HAL_OK)
  {
    UART_Print("Write PWR_MGMT_1 success.\r\n");
  }
  else
  {
    UART_Print("Write PWR_MGMT_1 failed.\r\n");
  }

  osDelay(100);

  /* 3. Read back PWR_MGMT_1 */
  status = MPU6500_ReadReg(PWR_MGMT_1,&readData);

  if (status == HAL_OK)
  {
    messageLength = snprintf(message, sizeof(message), "PWR_MGMT_1 = 0x%02X\r\n\r\n", readData);

    UART_Print(message);
  }

  int16_t accelX = 0;
  int16_t accelY = 0;
  int16_t accelZ = 0;

  uint32_t nextWakeTick;

  uint32_t sequence = 0;
  uint32_t printDroppedCount = 0;
  uint32_t loggerDroppedCount = 0;
  uint32_t spiErrorCount = 0;

  IMUData_t imuData = {0};
  LoggerMessage_t loggerMessage = {0};

  /* Start the fixed sampling period. */
  nextWakeTick = osKernelGetTickCount();

  for (;;)
  {
    status = MPU6500_ReadAccel(&accelX, &accelY, &accelZ);

    if (status == HAL_OK)
    {
      sequence++;

      imuData.sequence = sequence;
      imuData.tick_ms = osKernelGetTickCount();

      imuData.print_dropped_total = printDroppedCount;

      imuData.logger_dropped_total = loggerDroppedCount;

      imuData.spi_error_total = spiErrorCount;

      imuData.accel_x = accelX;
      imuData.accel_y = accelY;
      imuData.accel_z = accelZ;

      /*
       * Send data to PrintTask.
       */
      if (osMessageQueuePut(imuQueueHandle, &imuData, 0U, 0U) != osOK)
      {
        printDroppedCount++;
      }

      /*
       * Send data to LoggerTask.
       */
      if (loggingEnabled != 0U)
      {
        loggerMessage.type = LOGGER_MESSAGE_SAMPLE;

        loggerMessage.payload.sample = imuData;

        if (osMessageQueuePut(loggerQueueHandle, &loggerMessage, 0U, 0U) != osOK)
        {
          loggerDroppedCount++;
        }
      }
    }
    else
    {
      spiErrorCount++;
    }

    /* Report task health unless fault is injected. */
    if (injectImuHealthFault == 0U)
    {
      osEventFlagsSet(systemHealthEventHandle, HEALTH_IMU_FLAG);
    }
    
    /* Run at 50 Hz. */
    nextWakeTick += 20U;
    osDelayUntil(nextWakeTick);
  }
  /* USER CODE END StartIMUTask */
}

/* USER CODE BEGIN Header_StartPrintTask */
/**
* @brief Function implementing the printTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPrintTask */
void StartPrintTask(void *argument)
{
  /* USER CODE BEGIN StartPrintTask */
  /* Infinite loop */
  osStatus_t queueStatus;
  
  IMUData_t imuData;

  uint32_t receivedCount = 0;

  int32_t accelX_mg;
  int32_t accelY_mg;
  int32_t accelZ_mg;

  char message[160];
  int messageLength;

  for (;;)
  {
    queueStatus = osMessageQueueGet(imuQueueHandle, &imuData, NULL, TASK_IDLE_TIMEOUT_MS);

    /* Report task health. */
    osEventFlagsSet(systemHealthEventHandle, HEALTH_PRINT_FLAG);
    
    if (queueStatus == osOK)
    {
      receivedCount++;

      /*
       * Consume every sample, but print every 25 samples.
       */
      if (receivedCount >= 25U)
      {
        receivedCount = 0;

        accelX_mg = ((int32_t)imuData.accel_x * 1000L) / 16384L;

        accelY_mg = ((int32_t)imuData.accel_y * 1000L) / 16384L;

        accelZ_mg = ((int32_t)imuData.accel_z * 1000L) / 16384L;

        messageLength = snprintf(message, sizeof(message), "SEQ=%lu, T=%lu ms, AX=%ld mg, AY=%ld mg, AZ=%ld mg, PDROP=%lu, LDROP=%lu, SPIERR=%lu\r\n", (unsigned long)imuData.sequence, (unsigned long)imuData.tick_ms, (long)accelX_mg, (long)accelY_mg, (long)accelZ_mg, (unsigned long)imuData.print_dropped_total, (unsigned long)imuData.logger_dropped_total, (unsigned long)imuData.spi_error_total);

        UART_Print(message);

        HAL_GPIO_TogglePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin);
      }
    }
  }
  /* USER CODE END StartPrintTask */
}

/* USER CODE BEGIN Header_StartLoggerTask */
/**
* @brief Function implementing the loggerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLoggerTask */
void StartLoggerTask(void *argument)
{
  /* USER CODE BEGIN StartLoggerTask */
  /* Infinite loop */
  osStatus_t queueStatus;
  
  LoggerMessage_t loggerMessage;
  IMUData_t imuData;

  FlashRecord_t record;
  FlashRecord_t firstRecord = {0};
  FlashRecord_t lastRecord = {0};

  HAL_StatusTypeDef status;

  uint32_t writeAddress = FLASH_LOG_BASE_ADDRESS;

  uint32_t lastWrittenSequence = 0;

  char message[160];

  UART_Print("Logger erase started.\r\n");

  loggerBusy = 1U;

  status = Logger_EraseLogRegion();

  loggerBusy = 0U;

  if (status != HAL_OK)
  {
    loggingEnabled = 0U;
    loggerReady = 0U;

    UART_Print("Logger sector erase failed.\r\n");

    osThreadExit();
  }

  UART_Print("Logger erase success.\r\n");

  /*
   * Prepare the logger and wait for a start command.
   */
  osMessageQueueReset(loggerQueueHandle);

  loggingEnabled = 0U;
  loggerReady = 1U;
  loggerFull = 0U;
  loggerRecordCount = 0U;

  UART_Print("Logger ready. Press PA0 to start.\r\n\r\n");

  for (;;)
  {
    queueStatus = osMessageQueueGet(loggerQueueHandle, &loggerMessage, NULL, TASK_IDLE_TIMEOUT_MS);

    /* Report task health. */
    osEventFlagsSet(systemHealthEventHandle, HEALTH_LOGGER_FLAG);    

    if (queueStatus != osOK)
    {
      continue;
    }

    if (loggerMessage.type == LOGGER_MESSAGE_SAMPLE)
    {
      imuData = loggerMessage.payload.sample;

      if (loggerRecordCount >= FLASH_LOG_MAX_RECORDS)
      {
        continue;
      }

      memset(&record, 0, sizeof(record));

      record.magic = FLASH_RECORD_MAGIC;

      record.sequence = imuData.sequence;

      record.tick_ms = imuData.tick_ms;

      record.print_dropped_total = imuData.print_dropped_total;

      record.logger_dropped_total = imuData.logger_dropped_total;

      record.spi_error_total = imuData.spi_error_total;

      record.accel_x = imuData.accel_x;

      record.accel_y = imuData.accel_y;

      record.accel_z = imuData.accel_z;

      status = W25Q64_PageProgram(writeAddress, (const uint8_t *)&record, sizeof(record));

      if (status != HAL_OK)
      {
        loggingEnabled = 0U;
        loggerReady = 0U;

        UART_Print("Logger write failed.\r\n");

        continue;
      }

      lastWrittenSequence = record.sequence;

      loggerRecordCount++;

      writeAddress += sizeof(FlashRecord_t);

      if (loggerRecordCount == FLASH_LOG_MAX_RECORDS)
      {
        loggingEnabled = 0U;
        loggerFull = 1U;

        UART_Print("Logger write complete.\r\n");

        status = W25Q64_ReadData(FLASH_LOG_BASE_ADDRESS, (uint8_t *)&firstRecord, sizeof(firstRecord));

        if (status != HAL_OK)
        {
          UART_Print("Read first record failed.\r\n");

          osThreadExit();
        }

        status = W25Q64_ReadData(FLASH_LOG_BASE_ADDRESS + ((loggerRecordCount - 1U) * sizeof(FlashRecord_t)), (uint8_t *)&lastRecord, sizeof(lastRecord));

        if (status != HAL_OK)
        {
          UART_Print("Read last record failed.\r\n");

          osThreadExit();
        }

        snprintf(message, sizeof(message), "Logger records=%lu, first=%lu, last=%lu\r\n", (unsigned long)loggerRecordCount, (unsigned long)firstRecord.sequence, (unsigned long)lastRecord.sequence);

        UART_Print(message);

        if ((firstRecord.magic == FLASH_RECORD_MAGIC) && (lastRecord.magic == FLASH_RECORD_MAGIC) && (lastRecord.sequence == lastWrittenSequence))
        {
          UART_Print("Logger verify: PASS\r\n");
        }
        else
        {
          UART_Print("Logger verify: FAIL\r\n");
        }

        UART_Print("Logger full. Dump is still available.\r\n");
      }
    }
    else if (loggerMessage.type == LOGGER_MESSAGE_DUMP)
    {
      Logger_DumpLatestRecords(loggerMessage.payload.dump_count);
    }
    else if (loggerMessage.type == LOGGER_MESSAGE_CLEAR)
    {
      UART_Print("Logger clear started.\r\n");

      loggingEnabled = 0U;
      loggerReady = 0U;

      status = Logger_EraseLogRegion();

      if (status != HAL_OK)
      {
        loggerBusy = 0U;
        loggerReady = 0U;

        UART_Print("Logger clear failed.\r\n");

        continue;
      }

      /*
       * Reset queued logger commands and samples.
       */
      osMessageQueueReset(loggerQueueHandle);

      writeAddress = FLASH_LOG_BASE_ADDRESS;

      loggerRecordCount = 0U;
      lastWrittenSequence = 0U;

      loggerFull = 0U;
      loggingEnabled = 0U;

      memset(&firstRecord, 0, sizeof(firstRecord));

      memset(&lastRecord, 0,sizeof(lastRecord));

      loggerReady = 1U;
      loggerBusy = 0U;

      UART_Print("Logger clear complete.\r\n");
    }
  }

  /* USER CODE END StartLoggerTask */
}

/* USER CODE BEGIN Header_StartCommandTask */
/**
* @brief Function implementing the commandTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommandTask */
void StartCommandTask(void *argument)
{
  /* USER CODE BEGIN StartCommandTask */
  /* Infinite loop */
  uint32_t flags;

  char command[UART_COMMAND_BUFFER_SIZE];

  /*
   * Start UART byte reception.
   */
  if (HAL_UART_Receive_IT(&huart5, &uartRxByte, 1U) != HAL_OK)
  {
    UART_Print("UART RX start failed.\r\n");
  }
  else
  {
    if (resetByIWDG != 0U)
    {
      UART_Print("\r\n--- MCU RESET ---\r\nReset cause: IWDG watchdog\r\n");
    }
    else
    {
      UART_Print("\r\n--- MCU RESET ---\r\nReset cause: Power-on\r\n");
    }
    
    UART_Print("Commands: start, stop, status, dump N, clear, fault imu, help\r\n\r\n");
  }

  for (;;)
  {
    flags = osThreadFlagsWait(COMMAND_BUTTON_FLAG | COMMAND_UART_RX_FLAG, osFlagsWaitAny, TASK_IDLE_TIMEOUT_MS);
    
    /* Report task health. */
    osEventFlagsSet(systemHealthEventHandle, HEALTH_COMMAND_FLAG);

    if ((flags & osFlagsError) != 0U)
    {
      continue;
    }

    /*
     * Handle PA0.
     */
    if ((flags & COMMAND_BUTTON_FLAG) != 0U)
    {
      Command_ToggleLogging();
    }

    /*
     * Handle UART command.
     */
    if ((flags & COMMAND_UART_RX_FLAG) != 0U)
    {
      if (uartCommandReady != 0U)
      {
        strncpy(command, uartCommandBuffer, sizeof(command));

        command[UART_COMMAND_BUFFER_SIZE - 1U] = '\0';

        /*
         * Accept the next command.
         */
        uartCommandLength = 0U;
        uartCommandReady = 0U;

        Command_ProcessLine(command);
      }
    }
  }
  /* USER CODE END StartCommandTask */
}

/* USER CODE BEGIN Header_StartSupervisorTask */
/**
* @brief Function implementing the supervisorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSupervisorTask */
void StartSupervisorTask(void *argument)
{
  /* USER CODE BEGIN StartSupervisorTask */
  /* Infinite loop */
  uint32_t flags;
  uint32_t currentFlags;
  uint32_t healthyCycles = 0U;

  char message[128];

  /*
   * Let all tasks finish initialization.
   */
  osDelay(1000);

  for (;;)
  {
    /*
     * Start a new health window.
     */
    osEventFlagsClear(systemHealthEventHandle, HEALTH_ALL_FLAGS);

    flags = osEventFlagsWait(systemHealthEventHandle, HEALTH_ALL_FLAGS, osFlagsWaitAll, HEALTH_WAIT_TIMEOUT_MS);

    if ((flags & osFlagsError) == 0U)
    {
      // Feed watchdog only when all tasks are healthy.
      HAL_IWDG_Refresh(&hiwdg);
      
      healthyCycles++;

      // Avoid excessive UART output.
      if (healthyCycles >= 5U)
      {
        healthyCycles = 0U;

        UART_Print("Health monitor: OK\r\n");
      }
    }
    else
    {
      healthyCycles = 0U;

      currentFlags = osEventFlagsGet(systemHealthEventHandle);

      snprintf(message, sizeof(message), "Health monitor: FAIL \r\nflags=0x%02lX, expected=0x%02lX\r\n", (unsigned long)currentFlags, (unsigned long)HEALTH_ALL_FLAGS);

      UART_Print(message);
          
      // Do not feed the watchdog.
    }

    osDelay(500);
  }
  /* USER CODE END StartSupervisorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
