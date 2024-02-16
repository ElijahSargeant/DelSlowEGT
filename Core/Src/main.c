/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "lwip/udp.h"
#include "lwip\err.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;

osThreadId defaultTaskHandle;
osThreadId ADCTempPollHandle;
osThreadId TempSensorPollHandle;
osThreadId CANFrameHandle;
osThreadId UpdateLEDSHandle;
osThreadId CalibrateThermoHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void const * argument);
void getADCTemps(void const * argument);
void getTempSensorData(void const * argument);
void sendCANFrame(void const * argument);
void setLEDStatus(void const * argument);
void setADCCalibration(void const * argument);

/* USER CODE BEGIN PFP */

void udp_Callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

float float_bits(const uint32_t u) 
{
        union {
          uint32_t u;
          float f;
        } temp;
        
        temp.u = u;
        return temp.f;
}

float lessThan500Coeff[10], greaterThan500Coeff[10];

float hornerCalc(float x, float* coeffs )
{
  float y = 0.0f;
  
  for (int8_t i = 9; i > -1; --i) {
    y = coeffs[i] + (y * x);
  }
  return y;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//All in Celsius
float cylinder1TempData;
float cylinder2TempData;
float cylinder3TempData;
float cylinder4TempData;

int8_t temperatureFromSensor;


//SPI, I2C, CAN, ETH, each gets 8 bits
/**
Microcontroller Status Bits

SPI bits 0-7
  0: Configuration OK
  1: Cylinder1 OK
  2: Cylinder2 OK
  3: Cylinder3 OK
  4: Cylinder4 OK
  5: Above Operating Temp
  6: Dangerous Temp Exceeded
  7: Conspicuous Outlier in Temps

I2C bits 8-15
  0: Connection OK
  1: Temp OK
  2: Temp Exceed 90C
  3: Temp Below 0C
  4: Spare
  5: Spare
  6: Spare
  7: Spare

CAN bits 16-23
  0: Connection OK
  1: Status Frame Error
  2: LED Flash Request
  3: LED Flash Request Eth
  4: Spare
  5: Spare
  6: Spare
  7: Spare

ETH bits 24-31
  0: Connection to Host OK
  1: Last packet send success
  2: packet received success
  3: Calibration All Request
  4: Calibration Cyl1 Request
  5: Calibration Cyl2 Request
  6: Calibration Cyl3 Request
  7: Calibration Cyl4 Request

**/
uint32_t microcontrollerStatus;

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ADCTempPoll */
  osThreadDef(ADCTempPoll, getADCTemps, osPriorityAboveNormal, 0, 128);
  ADCTempPollHandle = osThreadCreate(osThread(ADCTempPoll), NULL);

  /* definition and creation of TempSensorPoll */
  osThreadDef(TempSensorPoll, getTempSensorData, osPriorityLow, 0, 128);
  TempSensorPollHandle = osThreadCreate(osThread(TempSensorPoll), NULL);

  /* definition and creation of CANFrame */
  osThreadDef(CANFrame, sendCANFrame, osPriorityNormal, 0, 128);
  CANFrameHandle = osThreadCreate(osThread(CANFrame), NULL);

  /* definition and creation of UpdateLEDS */
  osThreadDef(UpdateLEDS, setLEDStatus, osPriorityIdle, 0, 128);
  UpdateLEDSHandle = osThreadCreate(osThread(UpdateLEDS), NULL);

  /* definition and creation of CalibrateThermo */
  osThreadDef(CalibrateThermo, setADCCalibration, osPriorityRealtime, 0, 128);
  CalibrateThermoHandle = osThreadCreate(osThread(CalibrateThermo), NULL);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 4;
  hfdcan1.Init.NominalTimeSeg2 = 5;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 0;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10C0ECFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_32BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x8005;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LEDOrange_Pin|LEDRed_Pin|LEDYellow_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LEDOrange_Pin LEDRed_Pin LEDYellow_Pin */
  GPIO_InitStruct.Pin = LEDOrange_Pin|LEDRed_Pin|LEDYellow_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//Where we deal with UDP messages coming from web interface
void udp_Callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) 
{
 
  //addr is the IP adress message came from
  //port is the port message came form
  
  //pcb is the data structure that holds stuff?
  //p is packet buffer that was received
  
  char* commandReceived;
  
  commandReceived = p->payload;
  
  printf("received packet from ip address %u and port %u, the payload received: %s", addr->addr, port, commandReceived); 
  
  pbuf_free(p);
  
  uint8_t convertedCommand = atoi(commandReceived);
  
  microcontrollerStatus += 0x04000000;//set bit 26
  
  switch (convertedCommand) {
  
    case 0x01: //cyl1 cal
      microcontrollerStatus += 0x08000000;//set bit 27
      break;
    case 0x02: //cyl2 cal
      microcontrollerStatus += 0x10000000;//set bit 28
      break;
    case 0x03: //cyl3 cal
      microcontrollerStatus += 0x20000000;//set bit 29
      break;
    case 0x04: //cyl4 cal
      microcontrollerStatus += 0x40000000;//set bit 30
      break;
    case 0x0F: //master cal
      microcontrollerStatus += 0x80000000;//set bit 31
      break;
    case 0x11: //flash all LEDs
      microcontrollerStatus += 0x00040000;//set bit 19 CAN
      break;
  }
  
  /**
  Microcontroller Status Bits
  ETH bits 24-31
  2: packet received success
  3: Calibration All Request
  4: Calibration Cyl1 Request
  5: Calibration Cyl2 Request
  6: Calibration Cyl3 Request
  7: Calibration Cyl4 Request
  **/
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
  
  osDelay(1000);

  ip_addr_t PC_IPADDR;
  IP_ADDR4(&PC_IPADDR, 192, 168, 1, 100);//IP of host PC

  struct udp_pcb* delSlowEGTUDP = udp_new();
  
  
  struct pbuf* cyl1UDPBuffer = NULL;
  struct pbuf* cyl2UDPBuffer = NULL;
  struct pbuf* cyl3UDPBuffer = NULL;
  struct pbuf* cyl4UDPBuffer = NULL;
  
  udp_recv(delSlowEGTUDP, udp_Callback, NULL);
  
  /**
  Microcontroller Status Bits
  ETH bits 24-31
  0: Connection to Host OK
  1: Last packet send success
  **/
    
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
    
    //Default Task will always clear out non-sticky status (every 10ms)
    //Only sticky rn are bits 0,8,16,24
    microcontrollerStatus &= 0xFEFEFEFE;
  
    if((microcontrollerStatus & 0x10000000) == 0) {
      if(udp_connect(delSlowEGTUDP, &PC_IPADDR, 3000) == 0) {//Port 3000
        microcontrollerStatus += 0x01000000;//set bit 24 *Sticky*
      }
    }
    
//Keep for Debugging
/*
    const char* helloWorldMessage = "Hello from the DelSlowEGT Board";
    struct pbuf* egtUDPBuffer     = NULL;
    egtUDPBuffer = pbuf_alloc(PBUF_TRANSPORT, strlen(helloWorldMessage), PBUF_RAM);

    if(egtUDPBuffer != NULL) {
      memcpy(egtUDPBuffer->payload, helloWorldMessage, strlen(helloWorldMessage));
      udp_send(delSlowEGTUDP, egtUDPBuffer);
      pbuf_free(egtUDPBuffer);
      
      microcontrollerStatus += 0x02000000;//set bit 25
    }
*/
    
    cyl4UDPBuffer = pbuf_alloc(PBUF_TRANSPORT, sizeof(cylinder4TempData), PBUF_RAM);
    cyl3UDPBuffer = pbuf_alloc(PBUF_TRANSPORT, sizeof(cylinder3TempData), PBUF_RAM);
    cyl2UDPBuffer = pbuf_alloc(PBUF_TRANSPORT, sizeof(cylinder2TempData), PBUF_RAM);
    cyl1UDPBuffer = pbuf_alloc(PBUF_TRANSPORT, sizeof(cylinder1TempData), PBUF_RAM);
    
    if(cyl4UDPBuffer != NULL) { 
      memcpy(cyl4UDPBuffer->payload, &cylinder4TempData, sizeof(cylinder4TempData));
      udp_send(delSlowEGTUDP, cyl4UDPBuffer);
      pbuf_free(cyl4UDPBuffer);
    }
    if(cyl3UDPBuffer != NULL) { 
      memcpy(cyl3UDPBuffer->payload, &cylinder3TempData, sizeof(cylinder3TempData));
      udp_send(delSlowEGTUDP, cyl3UDPBuffer);
      pbuf_free(cyl3UDPBuffer);
    }
    if(cyl2UDPBuffer != NULL) { 
      memcpy(cyl2UDPBuffer->payload, &cylinder2TempData, sizeof(cylinder2TempData));
      udp_send(delSlowEGTUDP, cyl2UDPBuffer);
      pbuf_free(cyl2UDPBuffer);
    }
    if(cyl1UDPBuffer != NULL) { 
      memcpy(cyl1UDPBuffer->payload, &cylinder1TempData, sizeof(cylinder1TempData));
      udp_send(delSlowEGTUDP, cyl1UDPBuffer);
      pbuf_free(cyl1UDPBuffer);
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_getADCTemps */
/**
* @brief Function implementing the ADCTempPoll thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_getADCTemps */
void getADCTemps(void const * argument)
{
  /* USER CODE BEGIN getADCTemps */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  //ADC Configuration
  uint8_t ADC_CONFIG_0[4];
  ADC_CONFIG_0[3] = 0b01000110;
  ADC_CONFIG_0[2] = 0b00000000;
  ADC_CONFIG_0[1] = 0b00000000;
  ADC_CONFIG_0[0] = 0b01100011;
  
  uint8_t ADC_CONFIG_1[4];
  ADC_CONFIG_1[3] = 0b01001010;
  ADC_CONFIG_1[2] = 0b00000000;
  ADC_CONFIG_1[1] = 0b00000000;
  ADC_CONFIG_1[0] = 0b00111100;
  
  uint8_t ADC_CONFIG_2[4];
  ADC_CONFIG_1[3] = 0b01001110;
  ADC_CONFIG_1[2] = 0b00000000; 
  ADC_CONFIG_1[1] = 0b00000000;
  ADC_CONFIG_1[0] = 0b10100001;
  
  uint8_t ADC_CONFIG_3[4];
  ADC_CONFIG_3[3] = 0b01010010;
  ADC_CONFIG_3[2] = 0b00000000;
  ADC_CONFIG_3[1] = 0b00000000;
  ADC_CONFIG_3[0] = 0b11110000;
  
  uint8_t ADC_SCAN_REG[4];
  ADC_SCAN_REG[3] = 0b01011110;
  ADC_SCAN_REG[2] = 0b00000000;
  ADC_SCAN_REG[1] = 0b00001111;
  ADC_SCAN_REG[0] = 0b00000000; 
  
  uint8_t ADC_TIME_REG[4];
  ADC_TIME_REG[3] = 0b01100010;
  ADC_TIME_REG[2] = 0b00000000;
  ADC_TIME_REG[1] = 0b00000000;
  ADC_TIME_REG[0] = 0b00000000;
  
  uint8_t ADC_CONVERSION_START[4];
  ADC_CONVERSION_START[3] = 0b01101001;
  ADC_CONVERSION_START[2] = 0b00000000;
  ADC_CONVERSION_START[1] = 0b00000000;
  ADC_CONVERSION_START[0] = 0b00000000;

  //ADC read
  uint8_t ADC_READ_DATA[4];
  ADC_READ_DATA[3] = 0b01000001;
  ADC_READ_DATA[2] = 0b00000000;
  ADC_READ_DATA[1] = 0b00000000;
  ADC_READ_DATA[0] = 0b00000000;
  
  uint8_t adcTempData[4];
  
  uint32_t magicOperatingTempNumber = 500;//Celsius
  uint32_t magicDangerousTempNumber = 800;//Celsius
  float magicOutlierThresholdNumber = 1.1;//If one temp is outside 1 stddev away from avg uh oh

  // const uint8_t ADC_CONFIG_0_ADDR = 0b01000110;//0x1
  // const uint8_t ADC_CONFIG_1_ADDR = 0b01001010;//0x2
  // const uint8_t ADC_CONFIG_2_ADDR = 0b01001110;//0x3
  // const uint8_t ADC_CONFIG_3_ADDR = 0b01010010;//0x4
  // const uint8_t ADC_SCAN_REG_ADDR = 0b01011110;//0x7
  // const uint8_t ADC_TIME_REG_ADDR = 0b01100010;//0x8


  /*
  CONFIG0 Register
    -VREF_SEL Bit = 0 for external reference
    -ADC_MODE[1:0] = 11 for conversion mode
    -CLK_SEL[1:0] = 10 for internal clock no output

  CONFIG3 Register
    -CONV_MODE[1:0] = 11 for continuous conversions
      -Has TIMER[23:0] delay between each conversion cycle
      -Figure 5-17 for timing diagram

  Programming gain - CONFIG2 Register, GAIN[2:0]
    -try a gain of 011 (4x analog gain frist) ~12dB increase
    -must also set BOOST[1:0] to 10 to keep bias current at 1x
    -Use OSR[3:0] to reduce noise impacts of reading, max out cause thermos respond slow anyways
      -OSR[3:0] = 1111, for OSR total of 98304, Data rate reduced to 12.5-50Hz

  Use DATA_FORMAT[1:0] of 11 to get channel number along with data (32bit total data)
    -only in scan mode

  SCAN[15:0] looks like following to get all 4 channels:
    0000 1111 0000 0000

    CH6-CH7 ID : 1011
    CH4-CH5 ID : 1010
    CH2-CH3 ID : 1001
    CH0-CH1 ID : 1000

  DLY[2:0] = 000 for no delay between scan cycles
  */

 /*
  Compatible with SPI 0,0 and 1,1
  Data clocked out on falling edge
  Data clcoked in on rising edge

  Start SPI with CS falling, end with CS rising

  -First send COMMAND Byte
  [7:6] = 01; Device address (always)
  [5:2] = 1010; Start continuous conversion
  [1:0] = 01; read register Address[5:2]
        = 10; incremental write register Address[5:2]

    When command is clocking in, status is clocked out (full-duplex only), to see successful write or not
    STATUS[7:0] = 00[5:3]xxx, where [5:3] is device address commanded
  
  -For 10 (incremental write), following behavior is used:
    -look at Figure 6.3
  -For 01 (static read), following behavior:
    -ADC DATA is at address 0x0 (0000)

  PG90 has all register breakdowns
 */

/*
  Init with writing to CONFIG registers

  Loop with grabbing data off SDI whenever we want, chuck it into buffer with matching channels
*/

  //Initial write to ADC to configure on powerup
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, ADC_CONFIG_0, 4, 100);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  osDelay(1);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, ADC_CONFIG_1, 4, 100);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  osDelay(1);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, ADC_CONFIG_2, 4, 100);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  osDelay(1);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, ADC_CONFIG_3, 4, 100);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  osDelay(1);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, ADC_SCAN_REG, 4, 100);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  osDelay(1);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, ADC_TIME_REG, 4, 100);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  osDelay(1);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, ADC_CONVERSION_START, 4, 100);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  osDelay(1);
  
  uint32_t cylinder1RawData;
  uint32_t cylinder2RawData;
  uint32_t cylinder3RawData;
  uint32_t cylinder4RawData;
  
  
  
  lessThan500Coeff[0] = 0.0f;
  lessThan500Coeff[1] = 25.08355f;
  lessThan500Coeff[2] = 0.07860106f;
  lessThan500Coeff[3] = -0.2503131f;
  lessThan500Coeff[4] = 0.08315270f;
  lessThan500Coeff[5] = -0.01228034f;
  lessThan500Coeff[6] = 0.0009804036f;
  lessThan500Coeff[7] = -0.00004413030f;
  lessThan500Coeff[8] = 0.000001057734f;
  lessThan500Coeff[9] = -0.00000001052755f;
  
  greaterThan500Coeff[0] = -131.8058f;
  greaterThan500Coeff[1] = 48.30222f;
  greaterThan500Coeff[2] = -1.646031f;
  greaterThan500Coeff[3] = 0.05464731f;
  greaterThan500Coeff[4] = -0.0009650715f;
  greaterThan500Coeff[5] = 0.000008802193f;
  greaterThan500Coeff[6] = 0.0f;
  greaterThan500Coeff[7] = 0.0f;
  greaterThan500Coeff[8] = 0.0f;
  greaterThan500Coeff[9] = 0.0f;
  
  uint32_t coeffCutoffTemp = 0x20078B;//~500C with 3.3V ref and 4x Gain
  float workingmilliVolts = 0;
  

  /* Infinite loop */
  for(;;)
  {
    //Every 1s
    osDelay(1000);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, ADC_READ_DATA, 4, 100);
    microcontrollerStatus += HAL_SPI_Receive(&hspi1, adcTempData, 4, 100) == HAL_OK ? 0x00000001 : 0x00000001;//Set bit 0 *Sticky*
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

    //mask off top 4 bits here
    switch (adcTempData[3])
    {
    case 0b1011:
      cylinder4RawData = (((uint32_t)adcTempData[3] << 24) | ((uint32_t)adcTempData[2] << 16) | ((uint32_t)adcTempData[1] << 8) | ((uint32_t)adcTempData[0]));
      
      workingmilliVolts = float_bits(cylinder4RawData) / 101680.0523f; //convert to millivolts
      
      cylinder4TempData = hornerCalc(workingmilliVolts, workingmilliVolts > coeffCutoffTemp ? greaterThan500Coeff : lessThan500Coeff) + temperatureFromSensor;//add on cold junction temp to offset properly
      
      microcontrollerStatus += 0x0000010;//Set bit 4
      break;
    case 0b1010:
      cylinder3RawData = ((uint32_t)adcTempData[3] << 24) | ((uint32_t)adcTempData[2] << 16) | ((uint32_t)adcTempData[1] << 8) | ((uint32_t)adcTempData[0]);
      
      workingmilliVolts = float_bits(cylinder3RawData) / 101680.0523f; //convert to millivolts
      
      cylinder3TempData = hornerCalc(workingmilliVolts, workingmilliVolts > coeffCutoffTemp ? greaterThan500Coeff : lessThan500Coeff) + temperatureFromSensor;//add on cold junction temp to offset properly
      
      microcontrollerStatus += 0x0000008;//Set bit 3
      break;
    case 0b1001:
      cylinder2RawData = ((uint32_t)adcTempData[3] << 24) | ((uint32_t)adcTempData[2] << 16) | ((uint32_t)adcTempData[1] << 8) | ((uint32_t)adcTempData[0]);
      
      workingmilliVolts = float_bits(cylinder2RawData) / 101680.0523f; //convert to millivolts
      
      cylinder2TempData = hornerCalc(workingmilliVolts, workingmilliVolts > coeffCutoffTemp ? greaterThan500Coeff : lessThan500Coeff) + temperatureFromSensor;//add on cold junction temp to offset properly
      
      microcontrollerStatus += 0x0000004;//Set bit 2
      break;
    case 0b1000:
      cylinder1RawData = ((uint32_t)adcTempData[3] << 24) | ((uint32_t)adcTempData[2] << 16) | ((uint32_t)adcTempData[1] << 8) | ((uint32_t)adcTempData[0]);
      
      workingmilliVolts = float_bits(cylinder1RawData) / 101680.0523f; //convert to millivolts
      
      cylinder1TempData = hornerCalc(workingmilliVolts, workingmilliVolts > coeffCutoffTemp ? greaterThan500Coeff : lessThan500Coeff) + temperatureFromSensor;//add on cold junction temp to offset properly
      
      microcontrollerStatus += 0x0000002;//Set bit 1
      break;
    default:
      break;
    }
    
    float avgTemp = (cylinder4TempData + cylinder3TempData + cylinder2TempData + cylinder1TempData) / 4;
    
    float stdDev = (((cylinder4TempData - avgTemp) * (cylinder4TempData - avgTemp))
               +  ((cylinder3TempData - avgTemp) * (cylinder3TempData - avgTemp)) 
               +  ((cylinder2TempData - avgTemp) * (cylinder2TempData - avgTemp))
               +  ((cylinder1TempData - avgTemp) * (cylinder1TempData - avgTemp))) / 2;
    
    microcontrollerStatus += avgTemp > magicOperatingTempNumber    ? 0x00000020: 0;//set bit 5
    microcontrollerStatus += avgTemp > magicDangerousTempNumber    ? 0x00000040: 0;//set bit 6
    microcontrollerStatus += stdDev  > magicOutlierThresholdNumber ? 0x00000080: 0;//set bit 7
    
/**
Microcontroller Status Bits

SPI bits 0-7
  0: Configuration OK
  1: Cylinder1 OK
  2: Cylinder2 OK
  3: Cylinder3 OK
  4: Cylinder4 OK
  5: Above Operating Temp
  6: Dangerous Temp Exceeded
  7: Conspicuous Outlier in Temps
**/
  }
    
  /* USER CODE END getADCTemps */
}

/* USER CODE BEGIN Header_getTempSensorData */
/**
* @brief Function implementing the TempSensorPoll thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_getTempSensorData */
void getTempSensorData(void const * argument)
{
  /* USER CODE BEGIN getTempSensorData */

  uint8_t tempSensorWriteAddr = 0b10010000;
  uint8_t tempData[2];
  
  int8_t magicExceed90Number = 90;

  /* Infinite loop */
  for(;;)
  {
    //Every 3s
    osDelay(3000);
    if(HAL_I2C_Master_Receive(&hi2c1, tempSensorWriteAddr << 1, tempData, 1, 100) == HAL_OK) {
     microcontrollerStatus += 0x00000100;//set bit 8 *Sticky* 
     microcontrollerStatus += 0x00000200;//set bit 9
    } else {
     microcontrollerStatus -= 0x00000100;//reset bit 8 *Sticky*
    }
    
    //temp is in celsius
    temperatureFromSensor = tempData[0];
    
    microcontrollerStatus += temperatureFromSensor > magicExceed90Number ? 0x00000400 : 0;//set bit 10
    microcontrollerStatus += temperatureFromSensor < 0                   ? 0x00000800 : 0;//set bit 11
    
/**
Microcontroller Status Bits
I2C bits 8-15
  0: Connection OK
  1: Temp OK
  2: Temp Exceed 90C
  3: Temp Below 0C
  4: Spare
  5: Spare
  6: Spare
  7: Spare
**/
  }
  /* USER CODE END getTempSensorData */
}

/* USER CODE BEGIN Header_sendCANFrame */
/**
* @brief Function implementing the CANFrame thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sendCANFrame */
void sendCANFrame(void const * argument)
{
  /* USER CODE BEGIN sendCANFrame */

  FDCAN_TxHeaderTypeDef TxHeader;
  FDCAN_FilterTypeDef canFilter;
  
  TxHeader.Identifier = 0x300;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.DataLength = FDCAN_DATA_BYTES_8;
  TxHeader.FDFormat = FDCAN_FRAME_CLASSIC;

  //0x300 for temp board frames?
  /*
    FRAMES
    id:   0x300
    name: CYLINDER_TEMPS
    dlc:  8 bytes

    FIELD
    frame:      CYLINDER_TEMPS
    name:       CYLINDER_1_TEMP
    start:      0
    length:     32 bits (4 bytes)
    multiplier: 1
    divisor:    10
    offset:     0
    signed:     SIGNED

    FRAME(0x300, CYLINDER12_TEMPS, 8);
    FIELD(CYLINDER_TEMPS, CYLINDER_1_TEMP,  0, 32, 1, 1, 0, SIGNED);
    FIELD(CYLINDER_TEMPS, CYLINDER_2_TEMP, 32, 32, 1, 1, 0, SIGNED);
    FRAME(0x300, CYLINDER34_TEMPS, 8);
    FIELD(CYLINDER_TEMPS, CYLINDER_3_TEMP,  0, 32, 1, 1, 0, SIGNED);
    FIELD(CYLINDER_TEMPS, CYLINDER_4_TEMP, 32, 32, 1, 1, 0, SIGNED);

    BAUD RATE: 
    2024: 500kHz
    2023: 500kHz
    2022: 250kHz
  */

  HAL_FDCAN_ConfigFilter(&hfdcan1, &canFilter);
  HAL_FDCAN_Start(&hfdcan1);
  


  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
    
    //Take 4 32bit and crush into two 64bit 
    uint8_t Cylinder34TempData[8];
    Cylinder34TempData[7] = ((unsigned)cylinder4TempData >> 24) & 0xFF;
    Cylinder34TempData[6] = ((unsigned)cylinder4TempData >> 16) & 0xFF;
    Cylinder34TempData[5] = ((unsigned)cylinder4TempData >> 8)  & 0xFF;
    Cylinder34TempData[4] = ((unsigned)cylinder4TempData >> 0)  & 0xFF;
    Cylinder34TempData[3] = ((unsigned)cylinder3TempData >> 24) & 0xFF;
    Cylinder34TempData[2] = ((unsigned)cylinder3TempData >> 16) & 0xFF;
    Cylinder34TempData[1] = ((unsigned)cylinder3TempData >> 8)  & 0xFF;
    Cylinder34TempData[0] = ((unsigned)cylinder3TempData >> 0)  & 0xFF;
    
    uint8_t Cylinder12TempData[8];
    Cylinder12TempData[7] = ((unsigned)cylinder2TempData >> 24) & 0xFF;
    Cylinder12TempData[6] = ((unsigned)cylinder2TempData >> 16) & 0xFF;
    Cylinder12TempData[5] = ((unsigned)cylinder2TempData >> 8)  & 0xFF;
    Cylinder12TempData[4] = ((unsigned)cylinder2TempData >> 0)  & 0xFF;
    Cylinder12TempData[3] = ((unsigned)cylinder1TempData >> 24) & 0xFF;
    Cylinder12TempData[2] = ((unsigned)cylinder1TempData >> 16) & 0xFF;
    Cylinder12TempData[1] = ((unsigned)cylinder1TempData >> 8)  & 0xFF;
    Cylinder12TempData[0] = ((unsigned)cylinder1TempData >> 0)  & 0xFF;
    
    microcontrollerStatus += HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, Cylinder34TempData) == HAL_OK ? 0x00010000: 0x00020000;//set bit 16 *Sticky* or set bit 17 on fail
    microcontrollerStatus += HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, Cylinder12TempData) == HAL_OK ? 0x00010000: 0x00020000;//set bit 16 *Sticky* or set bit 17 on fail
    
/**
Microcontroller Status Bits
CAN bits 16-23
  0: Connection OK
  1: Status Frame Error
  2: LED Flash Request
  3: Spare
  4: Spare
  5: Spare
  6: Spare
  7: Spare
**/
  }
  /* USER CODE END sendCANFrame */
}

/* USER CODE BEGIN Header_setLEDStatus */
/**
* @brief Function implementing the UpdateLEDS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_setLEDStatus */
void setLEDStatus(void const * argument)
{
  /* USER CODE BEGIN setLEDStatus */
  //errorLED - Red
  //not at operating temp LED - Orange
  //CAN connected LED - Yellow
  //Set all high on powerup
  HAL_GPIO_WritePin(GPIOE, LEDOrange_Pin|LEDRed_Pin|LEDYellow_Pin, GPIO_PIN_SET);
  
  
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    
    //keep high until at op temp
    HAL_GPIO_WritePin(GPIOE, LEDOrange_Pin, (microcontrollerStatus & 0x00000020) > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    
    //once can connected go high
    HAL_GPIO_WritePin(GPIOE, LEDYellow_Pin, (microcontrollerStatus & 0x00010000) > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    //If any comm erros go high
    HAL_GPIO_WritePin(GPIOE, LEDRed_Pin, (microcontrollerStatus & 0x01010101) < 0x01010101 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    //Light up all LEDs
    HAL_GPIO_WritePin(GPIOE, LEDOrange_Pin | LEDYellow_Pin | LEDRed_Pin, (microcontrollerStatus & 0x00080000) > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    /**
    Microcontroller Status Bits

    SPI bits 0-7
      0: Configuration OK
      5: Above Operating Temp

    I2C bits 8-15
      0: Connection OK

    CAN bits 16-23
      0: Connection OK
      3: LED Flash Request

    ETH bits 24-31
      0: Connection to Host OK
    **/
  }
  /* USER CODE END setLEDStatus */
}

/* USER CODE BEGIN Header_setADCCalibration */
/**
* @brief Function implementing the CalibrateThermo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_setADCCalibration */
void setADCCalibration(void const * argument)
{
  /* USER CODE BEGIN setADCCalibration */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END setADCCalibration */
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
  MPU_InitStruct.BaseAddress = 0x30000200;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512B;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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

#ifdef  USE_FULL_ASSERT
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
