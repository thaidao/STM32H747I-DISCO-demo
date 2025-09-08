/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "app_touchgfx.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>  // For string manipulation

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// System state enumeration
// Represents the different states of the Battery Management System (BMS)
typedef enum {
	INIT = 0,		// System is in initition
    STANDBY,     	// System is idle, no charging or discharging
    CHARGING,    	// System is charging the battery
    DISCHARGING, 	// System is discharging the battery
    ERROR_STATE  	// Error state, typically caused by sensor errors
} SystemState_t;


typedef struct{
	SystemState_t	eState;
	char			sStateName[16];
}st_SystemStateTbl_t;

const st_SystemStateTbl_t sysStatetbl[5] = {
    {INIT,        "INIT"},        // System is in initiation
    {STANDBY,     "STANDBY"},     // System is idle, no charging or discharging
    {CHARGING,    "CHARGING"},    // System is charging the battery
    {DISCHARGING, "DISCHARGING"}, // System is discharging the battery
    {ERROR_STATE, "ERROR_STATE"}  // Error state, typically caused by sensor errors
};

// Global state variable, initial state is STANDBY
static SystemState_t g_currentState = INIT;
static bool	g_pumpIsRunning = false;

// Error flags for voltage and current sensors
static bool g_voltageErrorFlag = false;
static bool g_currentErrorFlag = false;

static float g_voltageVal = 0;	//Store value of battery voltage
static float g_currentVal = 0;	//Store value of pump current


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LD1_GPIO_PORT                          GPIOI
#define LD1_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOI_CLK_ENABLE()
#define LD1_PIN                                GPIO_PIN_12  //Green

#define LD2_GPIO_PORT                          GPIOI
#define LD2_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOI_CLK_ENABLE()
#define LD2_PIN                                GPIO_PIN_13  //yellow

#define LD3_GPIO_PORT                          GPIOI
#define LD3_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOI_CLK_ENABLE()
#define LD3_PIN                                GPIO_PIN_14	//red

#define LD4_GPIO_PORT                          GPIOI
#define LD4_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOI_CLK_ENABLE()
#define LD4_PIN                                GPIO_PIN_15	//blue

#define LED_GREEN_GPIO_PORT						LD1_GPIO_PORT
#define LED_GREEN_CLK_ENABLE					LD1_GPIO_CLK_ENABLE()
#define LED_GREEN_PIN							LD1_PIN

#define LED_YELLOW_GPIO_PORT					LD2_GPIO_PORT
#define LED_YELLOW_CLK_ENABLE					LD2_GPIO_CLK_ENABLE()
#define LED_YELLOW_PIN							LD2_PIN

#define LED_RED_GPIO_PORT						LD3_GPIO_PORT
#define LED_RED_CLK_ENABLE						LD3_GPIO_CLK_ENABLE()
#define LED_RED_PIN								LD3_PIN

#define LED_BLUE_GPIO_PORT						LD4_GPIO_PORT
#define LED_BLUE_CLK_ENABLE						LD4_GPIO_CLK_ENABLE()
#define LED_BLUE_PIN							LD4_PIN


// Sensor range definitions (units: volts for voltage and amperes for current)
#define VOLTAGE_MIN_CHARGING 			2.1  // Minimum voltage for charging [V]
#define VOLTAGE_MAX_CHARGING 			2.5  // Maximum voltage for charging [V]
#define VOLTAGE_MIN_DISCHARGING 		0.9  // Minimum voltage for discharging [V]
#define VOLTAGE_MAX_DISCHARGING 		1.3  // Maximum voltage for discharging [V]
#define CURRENT_MIN 					1.0  // Minimum current for pump operation [A]
#define CURRENT_MAX 					1.3  // Maximum current for pump operation [A]

// For testing
#define TEST_STRATEGY_1		1		//Enable test strategy 1

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

DSI_HandleTypeDef hdsi;

JPEG_HandleTypeDef hjpeg;
MDMA_HandleTypeDef hmdma_jpeg_infifo_th;
MDMA_HandleTypeDef hmdma_jpeg_outfifo_th;

LTDC_HandleTypeDef hltdc;

QSPI_HandleTypeDef hqspi;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* Definitions for TouchGFXTask */
osThreadId_t TouchGFXTaskHandle;
const osThreadAttr_t TouchGFXTask_attributes = {
  .name = "TouchGFXTask",
  .stack_size = 3048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for videoTask */
osThreadId_t videoTaskHandle;
const osThreadAttr_t videoTask_attributes = {
  .name = "videoTask",
  .stack_size = 1000 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for StateMachineTsk */
osThreadId_t StateMachineTskHandle;
const osThreadAttr_t StateMachineTsk_attributes = {
  .name = "StateMachineTsk",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for VoltageSensTsk */
osThreadId_t VoltageSensTskHandle;
const osThreadAttr_t VoltageSensTsk_attributes = {
  .name = "VoltageSensTsk",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PCurrentSensTsk */
osThreadId_t PCurrentSensTskHandle;
const osThreadAttr_t PCurrentSensTsk_attributes = {
  .name = "PCurrentSensTsk",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PumpControlTsk */
osThreadId_t PumpControlTskHandle;
const osThreadAttr_t PumpControlTsk_attributes = {
  .name = "PumpControlTsk",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
OTM8009A_Object_t OTM8009AObj;
OTM8009A_IO_t IOCtx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_MDMA_Init(void);
static void MX_FMC_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_DMA2D_Init(void);
static void MX_DSIHOST_DSI_Init(void);
static void MX_LTDC_Init(void);
static void MX_CRC_Init(void);
static void MX_JPEG_Init(void);
static void MX_USART1_UART_Init(void);
void TouchGFX_Task(void *argument);
extern void videoTaskFunc(void *argument);
void StateMachineTask(void *argument);
void VoltageSensorTask(void *argument);
void PumpCurrentSensorTask(void *argument);
void PumpControlTask(void *argument);

/* USER CODE BEGIN PFP */
// Utility functions
void checkVoltage(float voltage);
void updateVoltage(float voltage);
float getVoltage();

void checkCurrent(float current);
void changeState(SystemState_t newState);
float getVoltage();

void uartLog(const char *task, const char *level, const char *function, const char *message);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_MDMA_Init();
  MX_FMC_Init();
  MX_QUADSPI_Init();
  MX_DMA2D_Init();
  MX_DSIHOST_DSI_Init();
  MX_LTDC_Init();
  MX_CRC_Init();
  MX_JPEG_Init();
  MX_USART1_UART_Init();
  MX_TouchGFX_Init();
  /* Call PreOsInit function */
  MX_TouchGFX_PreOSInit();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of TouchGFXTask */
  TouchGFXTaskHandle = osThreadNew(TouchGFX_Task, NULL, &TouchGFXTask_attributes);

  /* creation of videoTask */
  videoTaskHandle = osThreadNew(videoTaskFunc, NULL, &videoTask_attributes);

  /* creation of StateMachineTsk */
  StateMachineTskHandle = osThreadNew(StateMachineTask, NULL, &StateMachineTsk_attributes);

  /* creation of VoltageSensTsk */
  VoltageSensTskHandle = osThreadNew(VoltageSensorTask, NULL, &VoltageSensTsk_attributes);

  /* creation of PCurrentSensTsk */
  PCurrentSensTskHandle = osThreadNew(PumpCurrentSensorTask, NULL, &PCurrentSensTsk_attributes);

  /* creation of PumpControlTsk */
  PumpControlTskHandle = osThreadNew(PumpControlTask, NULL, &PumpControlTsk_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
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
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_R2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB888;
  hdma2d.Init.OutputOffset = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief DSIHOST Initialization Function
  * @param None
  * @retval None
  */
static void MX_DSIHOST_DSI_Init(void)
{

  /* USER CODE BEGIN DSIHOST_Init 0 */
  HAL_GPIO_WritePin(GPIOG , GPIO_PIN_3 , GPIO_PIN_RESET);
  HAL_Delay(20);/* wait 20 ms */
  HAL_GPIO_WritePin(GPIOG , GPIO_PIN_3, GPIO_PIN_SET);/* Deactivate XRES */
  HAL_Delay(10);/* Wait for 10ms after releasing XRES before sending commands */
  /* USER CODE END DSIHOST_Init 0 */

  DSI_PLLInitTypeDef PLLInit = {0};
  DSI_HOST_TimeoutTypeDef HostTimeouts = {0};
  DSI_PHY_TimerTypeDef PhyTimings = {0};
  DSI_LPCmdTypeDef LPCmd = {0};
  DSI_CmdCfgTypeDef CmdCfg = {0};

  /* USER CODE BEGIN DSIHOST_Init 1 */

  /* USER CODE END DSIHOST_Init 1 */
  hdsi.Instance = DSI;
  hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  hdsi.Init.TXEscapeCkdiv = 4;
  hdsi.Init.NumberOfLanes = DSI_TWO_DATA_LANES;
  PLLInit.PLLNDIV = 119;
  PLLInit.PLLIDF = DSI_PLL_IN_DIV3;
  PLLInit.PLLODF = DSI_PLL_OUT_DIV2;
  if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK)
  {
    Error_Handler();
  }
  HostTimeouts.TimeoutCkdiv = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout = 0;
  HostTimeouts.HighSpeedReadTimeout = 0;
  HostTimeouts.LowPowerReadTimeout = 0;
  HostTimeouts.HighSpeedWriteTimeout = 0;
  HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
  HostTimeouts.LowPowerWriteTimeout = 0;
  HostTimeouts.BTATimeout = 0;
  if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
  {
    Error_Handler();
  }
  PhyTimings.ClockLaneHS2LPTime = 28;
  PhyTimings.ClockLaneLP2HSTime = 33;
  PhyTimings.DataLaneHS2LPTime = 15;
  PhyTimings.DataLaneLP2HSTime = 25;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 0;
  if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigFlowControl(&hdsi, DSI_FLOW_CONTROL_BTA) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetLowPowerRXFilter(&hdsi, 10000) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  LPCmd.LPGenShortWriteNoP = DSI_LP_GSW0P_ENABLE;
  LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_ENABLE;
  LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_ENABLE;
  LPCmd.LPGenShortReadNoP = DSI_LP_GSR0P_ENABLE;
  LPCmd.LPGenShortReadOneP = DSI_LP_GSR1P_ENABLE;
  LPCmd.LPGenShortReadTwoP = DSI_LP_GSR2P_ENABLE;
  LPCmd.LPGenLongWrite = DSI_LP_GLW_ENABLE;
  LPCmd.LPDcsShortWriteNoP = DSI_LP_DSW0P_ENABLE;
  LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_ENABLE;
  LPCmd.LPDcsShortReadNoP = DSI_LP_DSR0P_ENABLE;
  LPCmd.LPDcsLongWrite = DSI_LP_DLW_ENABLE;
  LPCmd.LPMaxReadPacket = DSI_LP_MRDP_ENABLE;
  LPCmd.AcknowledgeRequest = DSI_ACKNOWLEDGE_ENABLE;
  if (HAL_DSI_ConfigCommand(&hdsi, &LPCmd) != HAL_OK)
  {
    Error_Handler();
  }
  CmdCfg.VirtualChannelID = 0;
  CmdCfg.ColorCoding = DSI_RGB888;
  CmdCfg.CommandSize = 400;
  CmdCfg.TearingEffectSource = DSI_TE_EXTERNAL;
  CmdCfg.TearingEffectPolarity = DSI_TE_RISING_EDGE;
  CmdCfg.HSPolarity = DSI_HSYNC_ACTIVE_HIGH;
  CmdCfg.VSPolarity = DSI_VSYNC_ACTIVE_HIGH;
  CmdCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  CmdCfg.VSyncPol = DSI_VSYNC_RISING;
  CmdCfg.AutomaticRefresh = DSI_AR_DISABLE;
  CmdCfg.TEAcknowledgeRequest = DSI_TE_ACKNOWLEDGE_ENABLE;
  if (HAL_DSI_ConfigAdaptedCommandMode(&hdsi, &CmdCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DSIHOST_Init 2 */

  /* USER CODE END DSIHOST_Init 2 */

}

/**
  * @brief JPEG Initialization Function
  * @param None
  * @retval None
  */
static void MX_JPEG_Init(void)
{

  /* USER CODE BEGIN JPEG_Init 0 */

  /* USER CODE END JPEG_Init 0 */

  /* USER CODE BEGIN JPEG_Init 1 */

  /* USER CODE END JPEG_Init 1 */
  hjpeg.Instance = JPEG;
  if (HAL_JPEG_Init(&hjpeg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN JPEG_Init 2 */

  /* USER CODE END JPEG_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AH;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AH;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 0;
  hltdc.Init.VerticalSync = 0;
  hltdc.Init.AccumulatedHBP = 2;
  hltdc.Init.AccumulatedVBP = 2;
  hltdc.Init.AccumulatedActiveW = 402;
  hltdc.Init.AccumulatedActiveH = 482;
  hltdc.Init.TotalWidth = 403;
  hltdc.Init.TotalHeigh = 483;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 400;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 480;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB888;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xD0000000;
  pLayerCfg.ImageWidth = 400;
  pLayerCfg.ImageHeight = 480;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

      /* Configure DSI PHY HS2LP and LP2HS timings */

  __HAL_LTDC_DISABLE(&hltdc);
  DSI_LPCmdTypeDef LPCmd;

  HAL_DSI_Start(&hdsi);

  /* Configure the audio driver */
  IOCtx.Address     = 0;
  IOCtx.GetTick     = BSP_GetTick;
  IOCtx.WriteReg    = DSI_IO_Write;
  IOCtx.ReadReg     = DSI_IO_Read;
  OTM8009A_RegisterBusIO(&OTM8009AObj, &IOCtx);

  OTM8009A_Init(&OTM8009AObj ,OTM8009A_FORMAT_RGB888, OTM8009A_ORIENTATION_LANDSCAPE);
  HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_DISPOFF, 0x00);

  LPCmd.LPGenShortWriteNoP = DSI_LP_GSW0P_DISABLE;
  LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_DISABLE;
  LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_DISABLE;
  LPCmd.LPGenShortReadNoP = DSI_LP_GSR0P_DISABLE;
  LPCmd.LPGenShortReadOneP = DSI_LP_GSR1P_DISABLE;
  LPCmd.LPGenShortReadTwoP = DSI_LP_GSR2P_DISABLE;
  LPCmd.LPGenLongWrite = DSI_LP_GLW_DISABLE;
  LPCmd.LPDcsShortWriteNoP = DSI_LP_DSW0P_DISABLE;
  LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_DISABLE;
  LPCmd.LPDcsShortReadNoP = DSI_LP_DSR0P_DISABLE;
  LPCmd.LPDcsLongWrite = DSI_LP_DLW_DISABLE;
  HAL_DSI_ConfigCommand(&hdsi, &LPCmd);

  HAL_LTDC_SetPitch(&hltdc, 800, 0);
  __HAL_LTDC_ENABLE(&hltdc);
  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 3;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_ENABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */
  BSP_QSPI_Init_t init ;
  init.InterfaceMode=MT25TL01G_QPI_MODE;
  init.TransferRate= MT25TL01G_DTR_TRANSFER ;
  init.DualFlashMode= MT25TL01G_DUALFLASH_ENABLE;
  if (BSP_QSPI_Init(0,&init) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  if (BSP_QSPI_EnableMemoryMappedMode(0) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable MDMA controller clock
  */
static void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */

  /* MDMA interrupt initialization */
  /* MDMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MDMA_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);

}

/* FMC initialization function */
void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */
  FMC_Bank1_R->BTCR[0] &= ~FMC_BCRx_MBKEN;
  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_32;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */
  BSP_SDRAM_DeInit(0);
  if(BSP_SDRAM_Init(0) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOJ, LCD_BL_Pin|FRAME_RATE_Pin|RENDER_TIME_Pin|VSYNC_FREQ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCU_ACTIVE_GPIO_Port, MCU_ACTIVE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_BL_Pin FRAME_RATE_Pin RENDER_TIME_Pin VSYNC_FREQ_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin|FRAME_RATE_Pin|RENDER_TIME_Pin|VSYNC_FREQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RESET_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LCD_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_ACTIVE_Pin */
  GPIO_InitStruct.Pin = MCU_ACTIVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MCU_ACTIVE_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(LD1_GPIO_PORT, LD1_PIN, GPIO_PIN_RESET);

   /*Configure GPIO pins */
   GPIO_InitStruct.Pin = LD1_PIN;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(LD1_GPIO_PORT, &GPIO_InitStruct);

   //---------------------------------------
   HAL_GPIO_WritePin(LD2_GPIO_PORT, LD2_PIN, GPIO_PIN_RESET);

   /*Configure GPIO pins */
   GPIO_InitStruct.Pin = LD2_PIN;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(LD2_GPIO_PORT, &GPIO_InitStruct);

   //---------------------------------------
   HAL_GPIO_WritePin(LD3_GPIO_PORT, LD3_PIN, GPIO_PIN_RESET);

   /*Configure GPIO pins */
   GPIO_InitStruct.Pin = LD3_PIN;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(LD3_GPIO_PORT, &GPIO_InitStruct);

   //---------------------------------------
   HAL_GPIO_WritePin(LD3_GPIO_PORT, LD4_PIN, GPIO_PIN_RESET);

   /*Configure GPIO pins */
   GPIO_InitStruct.Pin = LD4_PIN;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(LD4_GPIO_PORT, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Update the global voltage value.
 *
 * This function updates the global voltage value to the specified value.
 *
 * @param voltage The new voltage value to be set.
 */
void updateVoltage(float voltage)
{
    g_voltageVal = voltage;
}

/**
 * @brief Get the current global voltage value.
 *
 * This function retrieves the current global voltage value.
 *
 * @return The current voltage value.
 */
float getVoltage()
{
    return g_voltageVal;
}

/**
 * @brief Simulate voltage reading from a sensor.
 *
 * This function simulates reading the voltage value from a sensor. It uses a dummy
 * voltage value for testing and can be replaced with actual sensor readings.
 *
 * @return The simulated voltage value.
 */
float readVoltageSensor()
{
    static float fDummyVoltage = 0;

    //@todo for testing: Get current value of slider on UI 0~3V
    #ifdef TEST_STRATEGY_1
        fDummyVoltage = (fDummyVoltage < 2.7) ? fDummyVoltage + 0.15 : 0.6; // Increase 0.15V each cycle
    #else
        fDummyVoltage = 2.2; // In charging
    #endif
    return fDummyVoltage;
}

/**
 * @brief Simulate current reading from a sensor.
 *
 * This function simulates reading the current value from a sensor. It uses a dummy
 * current value for testing and can be replaced with actual sensor readings.
 *
 * @return The simulated current value.
 */
float readCurrentSensor()
{
    static float fDummyCurrent = 0;

    //@todo for testing: Get current value of slider on UI (0~2A)
    #ifdef TEST_STRATEGY_1
        fDummyCurrent = 1.2;
    #else
        fDummyCurrent = (fDummyCurrent < 1.4) ? fDummyCurrent + 0.05 : 0.8; // Increase 0.05A each cycle
    #endif
    return fDummyCurrent;
}

/**
 * @brief Update the global current value.
 *
 * This function updates the global current value to the specified value.
 *
 * @param current The new current value to be set.
 */
void updateCurrent(float current)
{
    g_currentVal = current;
}

/**
 * @brief Get the current global current value.
 *
 * This function retrieves the current global current value.
 *
 * @return The current current value.
 */
float getCurrent()
{
    return g_currentVal;
}

/**
 * @brief Start the pump and activate the corresponding control actions.
 *
 * This function simulates starting the pump by turning on the blue LED and
 * changing the button text to "Running".
 */
void pumpStart()
{
	g_pumpIsRunning = true;
    // Turn on LED blue
    HAL_GPIO_WritePin(LED_BLUE_GPIO_PORT, LED_BLUE_PIN, GPIO_PIN_SET);
    // Change text of button from "Idle" to "Running"
}

/**
 * @brief Stop the pump and deactivate the corresponding control actions.
 *
 * This function simulates stopping the pump by turning off the blue LED and
 * changing the button text to "Idle".
 */
void pumpStop()
{
	g_pumpIsRunning = false;
    // Turn off LED blue
    HAL_GPIO_WritePin(LED_BLUE_GPIO_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);
    // Change text of button from "Running" to "Idle"
}

/**
 * @brief Check if the pump is running or not.
 *
 * This function checks if the pump is running or not.
 *
 * @return true if the pump is running, false otherwise.
 */
bool isPumpRunning()
{
	return g_pumpIsRunning;
}

/**
 * @brief Stop the pump and deactivate the corresponding control actions.
 *
 * This function simulates stopping the pump by turning off the blue LED and
 * changing the button text to "Idle".
 */

/**
 * @brief Check if the system is in the charging voltage range.
 *
 * This function checks if the current voltage value is within the charging range.
 *
 * @return true if the voltage is within the charging range, false otherwise.
 */
bool isChargingRange()
{
    float fVoltage = getVoltage();

    if (fVoltage > VOLTAGE_MIN_CHARGING && fVoltage < VOLTAGE_MAX_CHARGING) {
        return true;
    }

    return false;
}

/**
 * @brief Check if the system is in the discharging voltage range.
 *
 * This function checks if the current voltage value is within the discharging range.
 *
 * @return true if the voltage is within the discharging range, false otherwise.
 */
bool isDishargingRange()
{
    float fVoltage = getVoltage();

    if (fVoltage > VOLTAGE_MIN_DISCHARGING && fVoltage < VOLTAGE_MAX_DISCHARGING) {
        return true;
    }

    return false;
}

/**
 * @brief Get the current system state.
 *
 * This function retrieves the current system state.
 *
 * @return The current system state.
 */
SystemState_t getSystemSate()
{
    return g_currentState;
}

/**
 * @brief Set the system state to the specified state.
 *
 * This function sets the current system state to the specified state.
 *
 * @param state The new system state to be set.
 */
void setSystemState(SystemState_t state)
{
    g_currentState = state;
}

/**
 * @brief Change the system state.
 *
 * This utility function changes the system state if it is different from the current state.
 * It also logs the transition for debugging purposes.
 *
 * @param state The new system state to be set.
 */
void changeSystemState(SystemState_t state)
{
    char strTemp[128] = "";

    if (state == g_currentState) return;

    // Debug log
    sprintf(strTemp, "\n\r[DEBUG][INFO][changeSystemState] Before = %s, After = %s", sysStatetbl[g_currentState].sStateName, sysStatetbl[state].sStateName);
    USART1_Print(strTemp);

    // Set new state
    setSystemState(state);

    // LED state indicators
    if(state == DISCHARGING)
    	HAL_GPIO_WritePin(LED_YELLOW_GPIO_PORT, LED_YELLOW_PIN, GPIO_PIN_SET);
    else if (state == CHARGING)
    	HAL_GPIO_WritePin(LED_RED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_SET);
    else{
    	HAL_GPIO_WritePin(LED_YELLOW_GPIO_PORT, LED_YELLOW_PIN, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(LED_RED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(LED_BLUE_GPIO_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);
    }
}

/**
 * @brief Initialize the system with dummy values for testing.
 *
 * This function initializes the system by setting dummy values for voltage and current.
 */
void initializeSystem()
{
    // Dummy data for test
    updateVoltage(2.1);
    updateCurrent(1.1);

    // Initialize system status
    g_pumpIsRunning = false;
    g_currentState = INIT;


	HAL_GPIO_WritePin(LED_GREEN_GPIO_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_YELLOW_GPIO_PORT, LED_YELLOW_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_RED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_BLUE_GPIO_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Check if the system has been initialized.
 *
 * This function checks if the system has been initialized successfully.
 *
 * @return true if the system is initialized, false otherwise.
 */
bool isInitializeSystemDone()
{
    return true;
}

/**
 * @brief Check if there is any system error.
 *
 * This function checks if there are any voltage or current errors in the system.
 *
 * @return true if there is an error, false otherwise.
 */
bool isSystemError()
{
    if (g_voltageErrorFlag == false && g_currentErrorFlag == false)
        return false;

    return true;
}

/**
 * @brief Check if there is a request to change the system state from a higher entity.
 *
 * This function checks if there is a request to change the system state from a higher entity.
 *
 * @return true if there is a request, false otherwise.
 */
bool isHigherEntityChangeStateReq()
{
    return false;
}

/**
 * @brief Check if the voltage is within the acceptable range.
 *
 * This function checks if the voltage is within the range based on the current system state
 * (charging or discharging). If it is not, an error flag is set.
 *
 * @param voltage The voltage value to be checked.
 */
void checkVoltage(float voltage)
{
    SystemState_t sysState = getSystemSate();

    if (sysState == CHARGING && (g_voltageVal < VOLTAGE_MIN_CHARGING || g_voltageVal > VOLTAGE_MAX_CHARGING)) {
        g_voltageErrorFlag = true;
        uartLog("VoltageSensorTask", "ERROR", __FUNCTION__, "Voltage out of range for charging.");
        return;
    }

    if (sysState == DISCHARGING && (g_voltageVal < VOLTAGE_MIN_DISCHARGING || g_voltageVal > VOLTAGE_MAX_DISCHARGING)) {
        g_voltageErrorFlag = true;
        uartLog("VoltageSensorTask", "ERROR", __FUNCTION__, "Voltage out of range for discharging.");
        return;
    }

    g_voltageErrorFlag = false;
}

/**
 * @brief Check if the current is within the acceptable range.
 *
 * This function checks if the current is within the specified range. If it is not,
 * an error flag is set.
 *
 * @param current The current value to be checked.
 */
void checkCurrent(float current)
{
    if (g_currentVal < CURRENT_MIN || g_currentVal > CURRENT_MAX) {
        g_currentErrorFlag = true;
        uartLog("PumpCurrentSensorTask", "ERROR", __FUNCTION__, "Current out of range.");
    }else{
        g_currentErrorFlag = false;
    }
}

/**
 * @brief Handle any system errors that occur.
 *
 * This function handles system errors by checking voltage and current error flags
 * and printing appropriate error messages.
 */
void handleSystemError()
{
    if (g_voltageErrorFlag && g_currentErrorFlag) {
        USART1_Print("\n\r[DEBUG][ERROR] System is error");
    } else if (g_voltageErrorFlag) {
        USART1_Print("\n\r[DEBUG][ERROR] Voltage is out of range");
    } else if (g_currentErrorFlag) {
        USART1_Print("\n\r[DEBUG][ERROR] Pump current is out of range");
    }
}

/**
 * @brief Log a message to the UART.
 *
 * This function logs a message to the UART with a timestamp, task, level, function,
 * and message content.
 *
 * @param task The task name.
 * @param level The log level.
 * @param function The function name where the log is generated.
 * @param message The log message.
 */
void uartLog(const char *task, const char *level, const char *function, const char *message)
{
    return;
    char printStr[128];
    uint32_t time_ms = osKernelGetTickCount(); // Get current time in ms
    snprintf(printStr, sizeof(printStr), "[%lu] [%s] [%s] [%s] %s\r\n", time_ms, task, level, function, message);
    USART1_Print(printStr);
}

/**
 * @brief Log a debug message to the UART.
 *
 * This function logs a debug message to the UART with a timestamp, task, level,
 * function, and message content.
 *
 * @param task The task name.
 * @param level The log level.
 * @param function The function name where the log is generated.
 * @param message The debug message.
 */
void uartDebugLog(const char *task, const char *level, const char *function, const char *message)
{
    return;
    char printStr[128];
    uint32_t time_ms = osKernelGetTickCount(); // Get current time in ms
    snprintf(printStr, sizeof(printStr), "[%lu] [%s] [%s] [%s] %s\r\n", time_ms, task, level, function, message);
    USART1_Print(printStr);
}

/**
 * @brief Transmit a string to the UART.
 *
 * This function transmits a string to the UART.
 *
 * @param str The string to be transmitted.
 */
void USART1_Print(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/**
 * @brief Transmit a string followed by a newline to the UART.
 *
 * This function transmits a string followed by a newline to the UART.
 *
 * @param str The string to be transmitted.
 */
void USART1_Println(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
    uint8_t newline[] = "\r\n";
    HAL_UART_Transmit(&huart1, newline, sizeof(newline) - 1, HAL_MAX_DELAY);
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_TouchGFX_Task */
/**
  * @brief  Function implementing the TouchGFXTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TouchGFX_Task */
__weak void TouchGFX_Task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

#if 0
/* USER CODE BEGIN Header_StateMachineTask */
/**
* @brief Function implementing the StateMachineTsk thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StateMachineTask */
void StateMachineTask1(void *argument)
{
  /* USER CODE BEGIN StateMachineTask */
  /* Infinite loop */
  for(;;)
  {
    //osDelay(1);
	//Heart beat - I'm still alive
	HAL_GPIO_TogglePin(LED_GREEN_GPIO_PORT, LED_GREEN_PIN);
	HAL_GPIO_TogglePin(LED_YELLOW_GPIO_PORT, LED_YELLOW_PIN);
	HAL_GPIO_TogglePin(LED_RED_GPIO_PORT, LED_RED_PIN);
	HAL_GPIO_TogglePin(LED_BLUE_GPIO_PORT, LED_BLUE_PIN);
	osDelay(500);
  }
  /* USER CODE END StateMachineTask */
}
#else

//State machine task
void StateMachineTask(void *argument) {

	char strTemp[80];
	float fCurrent = 0;// getCurrent();
	float fVoltage = 0;//getVoltage();
	SystemState_t sysState; //= getSystemSate();

	//Initialize system
	sprintf(strTemp,"\n\r\n\r===========================================================================");
	USART1_Print(strTemp);
	initializeSystem();
	sprintf(strTemp,"\n\r\n\r[DEBUG][INFO] System state = INIT");
	USART1_Print(strTemp);


    osDelay(3000);  // Simulate periodic state machine check

	// Super loop
    for (;;) {

#if 1	//for debug purpose only
    	fCurrent = getCurrent();
    	fVoltage = getVoltage();
    	sysState = getSystemSate();

    	//sprintf(strTemp,"Current:%0.1f, Voltage:%0.1f",fCurrent,fVoltage);
    	//uartDebugLog("StateMachineTask", "ERROR", __FUNCTION__, strTemp);

    	sprintf(strTemp,"\n\r[DEBUG][INFO] Current:%0.2fA, Voltage:%0.2fV",fCurrent,fVoltage);
    	USART1_Print(strTemp);
#endif
    	// If received request change state from higher entity
		if(isHigherEntityChangeStateReq()){
			//changeSystemState(ERROR_STATE);
		}

		switch (sysState) {
			case INIT:
				if ( isInitializeSystemDone() == true){
					changeSystemState(STANDBY);
				}
				break;

			case STANDBY:
				//Check system is error or not
				if ( !isSystemError()){

					if(isChargingRange() == true){
						changeSystemState(CHARGING);

					}else if (isDishargingRange() == true){
						changeSystemState(DISCHARGING);
					}
				}else {
					changeSystemState(ERROR_STATE);
				}

				break;

			case CHARGING:
			case DISCHARGING:
				if(isSystemError() == true){
					changeSystemState(ERROR_STATE);
				}

				break;

			case ERROR_STATE:
				// Error handling
				handleSystemError();

				//Change to standby
				changeSystemState(STANDBY);
				break;

			default:
				break;
		}

		//Heart beat - I'm still alive
		HAL_GPIO_TogglePin(LED_GREEN_GPIO_PORT, LED_GREEN_PIN);
        osDelay(1000);  // Simulate periodic state machine check
    }
}

#endif

/* USER CODE BEGIN Header_VoltageSensorTask */
/**
* @brief Function implementing the VoltageSensTsk thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VoltageSensorTask */
void VoltageSensorTask(void *argument)
{
  /* USER CODE BEGIN VoltageSensorTask */
  /* Infinite loop */
  for(;;)
  {
	// Voltage reading from sensor
	float fVoltage = readVoltageSensor();

	// Update global voltage value
	updateVoltage(fVoltage);

	// Check voltage and raise error if any
	checkVoltage(fVoltage);

	// Log voltage sensor task status
	uartLog("VoltageSensorTask", "INFO", __FUNCTION__, "Voltage check completed.");

	// Simulate periodic delay of 1 second
	osDelay(1000);
  }
  /* USER CODE END VoltageSensorTask */
}

/* USER CODE BEGIN Header_PumpCurrentSensorTask */
/**
* @brief Function implementing the PCurrentSensTsk thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PumpCurrentSensorTask */
void PumpCurrentSensorTask(void *argument)
{
  /* USER CODE BEGIN PumpCurrentSensorTask */
  /* Infinite loop */
  for(;;)
  {
	// Current reading from sensor
	float fCurrent = readCurrentSensor();

	// Update global current value
	updateCurrent(fCurrent);

	// Check voltage and raise error if any
	checkCurrent(fCurrent);

	// Log current sensor task status
	uartLog("PumpCurrentSensorTask", "INFO", __FUNCTION__, "Current check completed.");

	// Simulate periodic delay of 1 second
	osDelay(1000);
  }
  /* USER CODE END PumpCurrentSensorTask */
}

/* USER CODE BEGIN Header_PumpControlTask */
/**
* @brief Function implementing the PumpControlTsk thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PumpControlTask */
void PumpControlTask(void *argument)
{
	/* USER CODE BEGIN PumpControlTask */
	char strTemp[64] = "";


	/* Infinite loop */
	for(;;)
	{
	  if (g_currentState == CHARGING || g_currentState == DISCHARGING) {
		// Log pump control task when pump is operating
		uartLog("PumpControlTask", "INFO", __FUNCTION__, "Pump is operating.");

		//Start pump if it is not running
		if(isPumpRunning()==false){
			pumpStart();

			//For debug
			sprintf(strTemp, "\n\r[DEBUG][INFO] Pump has started");
			USART1_Print(strTemp);
		}
	  } else {
		// Log pump stop status
		uartLog("PumpControlTask", "INFO", __FUNCTION__, "Pump is stopped.");

		// Stop pump
		if(isPumpRunning()==true){
			pumpStop();

			//For debug
			sprintf(strTemp, "\n\r[DEBUG][INFO] Pump has stopped");
			USART1_Print(strTemp);
		}
	  }

	  osDelay(500);  // Simulate periodic check
	}
  /* USER CODE END PumpControlTask */
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
  MPU_InitStruct.BaseAddress = 0x90000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128MB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0xD0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
  MPU_InitStruct.BaseAddress = 0x10000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER5;
  MPU_InitStruct.BaseAddress = 0x10040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
