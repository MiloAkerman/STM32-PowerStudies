/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "fatfs.h"
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL,
} BUFFER_StateTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_FREQUENCY       48000U
#define AUDIO_CHANNEL_NUMBER  2U
#define AUDIO_BUFFER_SIZE     4096U
#define AUDIO_PCM_CHUNK_SIZE  32U
#define REFERENCE_VOLTAGE 0.00002f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockA4;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai4_a;

SD_HandleTypeDef hsd1;

UART_HandleTypeDef huart1;

MDMA_HandleTypeDef hmdma_mdma_channel0_sdmmc1_end_data_0;
/* USER CODE BEGIN PV */
static AUDIO_Drv_t  *Audio_Drv = NULL;
WM8994_Init_t codec_init;

/* Buffer status variable */
__IO BUFFER_StateTypeDef bufferStatus = BUFFER_OFFSET_NONE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_BDMA_Init(void);
static void MX_MDMA_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SAI4_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SAI1_Init(void);
/* USER CODE BEGIN PFP */
static void AUDIO_IN_PDMToPCM_Init(uint32_t AudioFreq, uint32_t ChannelNumber);
static void AUDIO_IN_PDMToPCM(uint16_t *PDMBuf, uint16_t *PCMBuf, uint32_t ChannelNumber);
static int32_t WM8994_Probe(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

float calculate_decibel(int16_t *buffer, size_t size) {
    float sum = 0.0f;

    for (size_t i = 0; i < size; i++) {
        float voltage = (float)buffer[i] / 32768.0f; // Normalize 16-bit value
        sum += voltage * voltage;
    }

    float rms = sqrtf(sum / size);
    float spl = 20.0f * log10f(rms / REFERENCE_VOLTAGE);

    return spl;
}

int16_t input_buffer[AUDIO_BUFFER_SIZE] __attribute__((section(".DATA_RAM_D3")));
int16_t output_buffer[AUDIO_BUFFER_SIZE];

/* PDM Filters params */
PDM_Filter_Handler_t  PDM_FilterHandler[2];
PDM_Filter_Config_t   PDM_FilterConfig[2];
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
/* USER CODE END Boot_Mode_Sequence_0 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  //SCB_EnableDCache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_BDMA_Init();
  MX_MDMA_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SAI4_Init();
  //MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_SAI1_Init();
  MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_7);

  // TODO: Cache invalidation to avoid disabling cache
  //SCB_InvalidateDCache_by_Addr((uint32_t*)(((uint32_t)input_buffer) & ~(uint32_t)0x1F), sizeof(input_buffer)+32);

  BSP_AUDIO_Init_t BSP_OutputConfig = {0};
  BSP_OutputConfig.BitsPerSample = 16;
  BSP_OutputConfig.ChannelsNbr = 2;
  BSP_OutputConfig.Device = WM8994_OUT_HEADPHONE;
  BSP_OutputConfig.SampleRate = 16000;
  BSP_OutputConfig.Volume = 60;

  BSP_AUDIO_IN_Init(1, &BSP_OutputConfig); //for PDM2PCM
  BSP_AUDIO_OUT_Init(1, &BSP_OutputConfig);

  printf("Starting Input SAI4/BDMA...\r\n");
  if (HAL_SAI_Receive_DMA(&hsai_BlockA4, (uint8_t *)input_buffer, sizeof(input_buffer)) != HAL_OK) {
	  printf("SAI4/BDMA initialization failed! Error: %ld\r\n", hsai_BlockA4.ErrorCode);
  } else {
	  printf("SAI4/BDMA started successfully.\r\n");
  }

  printf("Starting Output SAI1/DMA...\r\n");
  if (BSP_AUDIO_OUT_Play(1, (uint8_t *)output_buffer, sizeof(output_buffer)) != BSP_ERROR_NONE) {
	  printf("SAI1/DMA initialization failed!\r\n");
  } else {
	  printf("SAI1/DMA started successfully.\r\n");
  }

  // TODO: Remove power testing code when done
  //  Testing the power consumption in sleep mode
//  HAL_Delay(2000);
//  HAL_SuspendTick();
//  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFI);
//  HAL_ResumeTick();

  //  Test the power consumption in standby mode
//  HAL_Delay(2000);
//  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
//  HAL_PWR_EnterSTANDBYMode();

  // Test the power consumption in standby mode
//  HAL_Delay(2000);
//  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
//  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
//  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
//  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
//	HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_7);
//  HAL_PWREx_EnterSTANDBYMode(PWR_D2_DOMAIN);
//  HAL_PWREx_EnterSTANDBYMode(PWR_D1_DOMAIN);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if(HAL_GPIO_ReadPin(GPIOJ, GPIO_PIN_0)){
//		  printf("GPIO INPUT WORKING\n");
//	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 6;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI4A|RCC_PERIPHCLK_SAI1;
  PeriphClkInitStruct.PLL2.PLL2M = 21;
  PeriphClkInitStruct.PLL2.PLL2N = 289;
  PeriphClkInitStruct.PLL2.PLL2P = 7;
  PeriphClkInitStruct.PLL2.PLL2Q = 7;
  PeriphClkInitStruct.PLL2.PLL2R = 7;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
  PeriphClkInitStruct.Sai4AClockSelection = RCC_SAI4ACLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */
	__HAL_RCC_SAI1_CLK_ENABLE();
  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_MCKDIV;
  hsai_BlockA1.Init.Mckdiv = 6;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.Init.PdmInit.Activation = DISABLE;
  hsai_BlockA1.Init.PdmInit.MicPairsNbr = 1;
  hsai_BlockA1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockA1.FrameInit.FrameLength = 128;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 4;
  hsai_BlockA1.SlotInit.SlotActive = 0x00000005;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */
  /* Init PDM Filters */
//  __HAL_SAI_ENABLE(&hsai_BlockA1);
  //WM8994_Probe();
  BSP_AUDIO_IN_PDMToPCM_Init(1, 16000, 2, 2);
  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SAI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI4_Init(void)
{

  /* USER CODE BEGIN SAI4_Init 0 */
  /* USER CODE END SAI4_Init 0 */

  /* USER CODE BEGIN SAI4_Init 1 */

  /* USER CODE END SAI4_Init 1 */
  hsai_BlockA4.Instance = SAI4_Block_A;
  hsai_BlockA4.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA4.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockA4.Init.DataSize = SAI_DATASIZE_16;
  hsai_BlockA4.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA4.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA4.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA4.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA4.Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;
  hsai_BlockA4.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_ENABLE;
  hsai_BlockA4.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA4.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_MCKDIV;
  hsai_BlockA4.Init.Mckdiv = 6;
  hsai_BlockA4.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA4.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA4.Init.PdmInit.Activation = ENABLE;
  hsai_BlockA4.Init.PdmInit.MicPairsNbr = 1;
  hsai_BlockA4.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockA4.FrameInit.FrameLength = 16;
  hsai_BlockA4.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA4.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA4.FrameInit.FSPolarity = SAI_FS_ACTIVE_HIGH;
  hsai_BlockA4.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA4.SlotInit.FirstBitOffset = 0;
  hsai_BlockA4.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA4.SlotInit.SlotNumber = 1;
  hsai_BlockA4.SlotInit.SlotActive = 0x00000001;
  if (HAL_SAI_Init(&hsai_BlockA4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI4_Init 2 */
  /* USER CODE END SAI4_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd1.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_BDMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_BDMA_CLK_ENABLE();

  /* DMA interrupt init */
  /* BDMA_Channel0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * Enable MDMA controller clock
  * Configure MDMA for global transfers
  *   hmdma_mdma_channel0_sdmmc1_end_data_0
  */
static void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */

  /* Configure MDMA channel MDMA_Channel0 */
  /* Configure MDMA request hmdma_mdma_channel0_sdmmc1_end_data_0 on MDMA_Channel0 */
  hmdma_mdma_channel0_sdmmc1_end_data_0.Instance = MDMA_Channel0;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.Request = MDMA_REQUEST_SDMMC1_END_DATA;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.BufferTransferLength = 1;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel0_sdmmc1_end_data_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel0_sdmmc1_end_data_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* MDMA interrupt initialization */
  /* MDMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Enable GPIOJ clock
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Rx Transfer completed callbacks.
  * @param  hsai : pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SAI_RxCpltCallback could be implemented in the user file
   */
//	printf("Buffer Full!! =======================================\r\n");
//	printf("SAI State: %d \r\n", HAL_SAI_GetState(&hsai_BlockA4));
//	printf("SAI Error: %lu \r\n", HAL_SAI_GetError(&hsai_BlockA4));
//	printf("SAI Buffer Address: 0x%08lx\r\n", SAI4_Block_A->DR);  // this should change over time if data is coming in
//	printf("DMA State: %d \r\n", HAL_DMA_GetState(&hdma_sai4_a));
//	printf("DMA Error: %lu \r\n", HAL_DMA_GetError(&hdma_sai4_a));
//	printf("Pointer location: %p \r\n", input_buffer);
//	printf("Audio Buffer Data:\r\n");
//	for (int i = 0; i < 10; i++) {
//		printf("[%d]: %d\r\n", i, input_buffer[i]);
//	}
	BSP_AUDIO_IN_PDMToPCM(1, (uint16_t*)input_buffer, (uint16_t*)output_buffer);

	float decibel_level = calculate_decibel(input_buffer, AUDIO_BUFFER_SIZE);
	//printf("SPL: %.2f dB\r\n", decibel_level);
}

/**
  * @brief  Rx Transfer Half completed callbacks
  * @param  hsai : pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SAI_RxHalfCpltCallback could be implenetd in the user file
   */
	//printf("Buffer Half Full!! ==================================\r\n");
}

/**
  * @brief  Register Bus IOs if component ID is OK
  * @retval error status
  */
static int32_t WM8994_Probe(void)
{
  int32_t ret = BSP_ERROR_NONE;
  WM8994_IO_t              IOCtx;
  static WM8994_Object_t   WM8994Obj;
  uint32_t id;

  /* Configure the audio driver */
  IOCtx.Address     = AUDIO_I2C_ADDRESS;
  IOCtx.Init        = BSP_I2C4_Init;
  IOCtx.DeInit      = BSP_I2C4_DeInit;
  IOCtx.ReadReg     = BSP_I2C4_ReadReg16;
  IOCtx.WriteReg    = BSP_I2C4_WriteReg16;
  IOCtx.GetTick     = BSP_GetTick;

  if(WM8994_RegisterBusIO (&WM8994Obj, &IOCtx) != WM8994_OK)
  {
    ret = BSP_ERROR_BUS_FAILURE;
  }
  else
  {
    /* Reset the codec */
    if(WM8994_Reset(&WM8994Obj) != WM8994_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else if(WM8994_ReadID(&WM8994Obj, &id) != WM8994_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else if(id != WM8994_ID)
    {
      ret = BSP_ERROR_UNKNOWN_COMPONENT;
    }
    else
    {
      Audio_Drv = (AUDIO_Drv_t *) &WM8994_Driver;
      Audio_CompObj = &WM8994Obj;
    }
  }
  return ret;
}

/**
* @brief  Init PDM Filters.
* @param  AudioFreq: Audio sampling frequency
* @param  ChannelNumber: Number of audio channels in the PDM buffer.
* @retval None
*/
static void AUDIO_IN_PDMToPCM_Init(uint32_t AudioFreq, uint32_t ChannelNumber)
{
  uint32_t index = 0;

  /* Enable CRC peripheral to unlock the PDM library */
  __HAL_RCC_CRC_CLK_ENABLE();

  for(index = 0; index < ChannelNumber; index++)
  {
    /* Init PDM filters */
    PDM_FilterHandler[index].bit_order  = PDM_FILTER_BIT_ORDER_MSB;
    PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_LE;
    PDM_FilterHandler[index].high_pass_tap = 2122358088;
    PDM_FilterHandler[index].out_ptr_channels = ChannelNumber;
    PDM_FilterHandler[index].in_ptr_channels  = ChannelNumber;
    PDM_Filter_Init((PDM_Filter_Handler_t *)(&PDM_FilterHandler[index]));

    /* Configure PDM filters */
    PDM_FilterConfig[index].output_samples_number = AudioFreq/1000;
    PDM_FilterConfig[index].mic_gain = 24;
    PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_64;
    PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
  }
}

/**
  * @brief  Convert audio format from PDM to PCM.
  * @param  PDMBuf: Pointer to PDM buffer data
  * @param  PCMBuf: Pointer to PCM buffer data
  * @param  ChannelNumber: PDM Channels number.
  * @retval None
*/
static void AUDIO_IN_PDMToPCM(uint16_t *PDMBuf, uint16_t *PCMBuf, uint32_t ChannelNumber)
{
  uint32_t index = 0;

  /* Invalidate Data Cache to get the updated content of the SRAM*/
  //SCB_InvalidateDCache_by_Addr((uint32_t *)PDMBuf, AUDIO_BUFFER_SIZE);

  for(index = 0; index < ChannelNumber; index++)
  {
    PDM_Filter(&((uint8_t*)(PDMBuf))[index], (uint16_t*)&(PCMBuf[index]), &PDM_FilterHandler[index]);
  }

  /* Clean Data Cache to update the content of the SRAM */
  //SCB_CleanDCache_by_Addr((uint32_t*)PCMBuf, AUDIO_PCM_CHUNK_SIZE*2);
}
/* USER CODE END 4 */

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
	  printf("Error!\r\n");
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
