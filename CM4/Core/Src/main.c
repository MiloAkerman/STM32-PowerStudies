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

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// SAI audio constants
#define AUDIO_FREQUENCY       16000U
#define AUDIO_CHANNEL_NUMBER  1U
#define AUDIO_BUFFER_SIZE     1024U
#define INFERENCE_BUFFER_SIZE 65536U //+ 25600U // ~4 sec of audio
											 // sec. per exp. of 2: 2^(x-10)/31.2 for n>=10). Add in multiples of 128

#define DECIMATION_FACTOR     64U
#define PDM_WORDS_PER_CHUNK   (AUDIO_PCM_CHUNK_SIZE * DECIMATION_FACTOR / BITS_PER_SAMPLE)
#define AUDIO_PCM_CHUNK_SIZE  16U
#define BITS_PER_SAMPLE		  16U

// Uncomment to enable headphone audio playback
//#define AUDIO_PLAYBACK

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SAI_HandleTypeDef hsai_BlockA4;
DMA_HandleTypeDef hdma_sai4_a;
SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai1_a;
UART_HandleTypeDef huart1;

uint16_t input_buffer[AUDIO_BUFFER_SIZE] __attribute__((section(".DATA_RAM_D3")));
uint16_t output_buffer[AUDIO_BUFFER_SIZE];
volatile uint32_t output_buffPtr;
uint16_t inference_buffer[INFERENCE_BUFFER_SIZE];
volatile uint32_t inference_buffPtr;

volatile BUFFER_StateTypeDef bufferStatus = BUFFER_OFFSET_NONE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_BDMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SAI4_Init(void);
#ifdef AUDIO_PLAYBACK
static void MX_DMA_Init(void);
static void MX_SAI1_Init(void);
#endif
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  Re-implementation of printf() to operate with USART (DO NOT REMOVE OR WILL NOT PRINT)
 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
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

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_USART1_UART_Init();
  MX_BDMA_Init();
  MX_SAI4_Init();
#ifdef AUDIO_PLAYBACK
  MX_DMA_Init();
  MX_SAI1_Init();
#endif

  /* USER CODE BEGIN 2 */

  BSP_AUDIO_Init_t BSP_OutputConfig = {0};
  BSP_OutputConfig.BitsPerSample = BITS_PER_SAMPLE;
  BSP_OutputConfig.ChannelsNbr = AUDIO_CHANNEL_NUMBER;
  BSP_OutputConfig.Device = WM8994_OUT_HEADPHONE;
  BSP_OutputConfig.SampleRate = AUDIO_FREQUENCY;
  BSP_OutputConfig.Volume = 60;

  memset(input_buffer, 0, AUDIO_BUFFER_SIZE * sizeof(uint16_t));

  BSP_AUDIO_IN_Init(1, &BSP_OutputConfig); // unused for input, used for PDM2PCM

  printf("Starting Input SAI4/BDMA...\r\n");

  HAL_StatusTypeDef input_init_response;
  input_init_response = BSP_AUDIO_IN_Record(1, (uint8_t *)input_buffer, AUDIO_BUFFER_SIZE);
  if (input_init_response != HAL_OK) {
	  printf("SAI4/BDMA initialization failed! Error: %d\r\n", input_init_response);
  } else {
  	  printf("SAI4/BDMA started successfully.\r\n");
  }

#ifdef AUDIO_PLAYBACK
  BSP_AUDIO_OUT_Init(1, &BSP_OutputConfig);
  printf("Starting Output SAI1/DMA...\r\n");

  int32_t output_init_response;
  output_init_response = BSP_AUDIO_OUT_Play(1, (uint8_t *)output_buffer, sizeof(output_buffer));
  if (output_init_response != BSP_ERROR_NONE) {
	  printf("SAI1/DMA initialization failed! Error: %ld\r\n", output_init_response);
  } else {
  	  printf("SAI1/DMA started successfully.\r\n");
  }
#endif

  // Initialize PDM2PCM filter
  BSP_AUDIO_IN_PDMToPCM_Init(1, AUDIO_FREQUENCY, AUDIO_CHANNEL_NUMBER, AUDIO_CHANNEL_NUMBER);

  /* Initialize Rx buffer status */
  bufferStatus &= BUFFER_OFFSET_NONE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Wait for half-buffer
	  if (0)
	  {
		  // Invalidate cache to avoid data mismatch issues
		  //dcache_invalidate(&input_buffer[0], (AUDIO_BUFFER_SIZE/2)*sizeof(uint16_t));
		  //printf("Buffer half full! \r\n");
		  // Process the first half of PDM buffer to PCM
		  for (int i = 0; i < (AUDIO_BUFFER_SIZE/2) / PDM_WORDS_PER_CHUNK; i++)
		  {
			  uint16_t *pdm_chunk = &input_buffer[i * PDM_WORDS_PER_CHUNK];
			  uint16_t  *pcm_out    = &output_buffer[output_buffPtr];
			  BSP_AUDIO_IN_PDMToPCM(1, pdm_chunk, pcm_out); // Apply PDM2PCM filter
			  //process_pcm_block(pcm_out); // Denoise PCM output with FIR filter
			  //dcache_clean(pcm_out, AUDIO_PCM_CHUNK_SIZE * sizeof(int16_t)); // Clean cache to avoid data mismatch issues
			  for (int j = 0; j < AUDIO_PCM_CHUNK_SIZE; j++) {
				  inference_buffer[inference_buffPtr + j] = output_buffer[output_buffPtr + j];
			  }

			  inference_buffPtr += AUDIO_PCM_CHUNK_SIZE;
			  output_buffPtr += AUDIO_PCM_CHUNK_SIZE;
		  }
		  bufferStatus &= ~BUFFER_OFFSET_HALF;
	  }

	  // Wait for full-buffer
	  if (0)
	  {
		  // Invalidate cache to avoid data mismatch issues
		  //dcache_invalidate(&input_buffer[AUDIO_BUFFER_SIZE/2], (AUDIO_BUFFER_SIZE/2)*sizeof(uint16_t));
		  //printf("Buffer full! \r\n");

		  // Process the second half the same way
		  for (int i = 0; i < (AUDIO_BUFFER_SIZE/2) / PDM_WORDS_PER_CHUNK; i++)
		  {
			  uint16_t *pdm_chunk = &input_buffer[AUDIO_BUFFER_SIZE/2 + i * PDM_WORDS_PER_CHUNK];
			  uint16_t  *pcm_out    = &output_buffer[output_buffPtr];
			  BSP_AUDIO_IN_PDMToPCM(1, pdm_chunk, pcm_out);
			  //process_pcm_block(pcm_out);
			  //dcache_clean(pcm_out, AUDIO_PCM_CHUNK_SIZE * sizeof(int16_t));
			  for (int j = 0; j < AUDIO_PCM_CHUNK_SIZE; j++) {
				  inference_buffer[inference_buffPtr + j] = output_buffer[output_buffPtr + j];
			  }

			  inference_buffPtr += AUDIO_PCM_CHUNK_SIZE;
			  output_buffPtr += AUDIO_PCM_CHUNK_SIZE;
		  }
		  bufferStatus &= ~BUFFER_OFFSET_FULL;
	  }

	  // Wrap pcm pointer if needed
	  if (output_buffPtr >= AUDIO_BUFFER_SIZE)
		  output_buffPtr = 0;

	  // Run inference when inference buffer is full
	  if (inference_buffPtr >= INFERENCE_BUFFER_SIZE) {
		  printf("Reached end of inference buffer on CM4\r\n");
		  inference_buffPtr = 0;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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

#ifdef AUDIO_PLAYBACK
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
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;
  hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_16K;
//  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_MCKDIV;
//  hsai_BlockA1.Init.Mckdiv = 6;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_MONOMODE;
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

  /* USER CODE END SAI1_Init 2 */

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
#endif

/**
  * @brief SAI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI4_Init(void)
{

  /* USER CODE BEGIN SAI4_Init 0 */
	__HAL_RCC_SAI4_CLK_ENABLE();
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
  hsai_BlockA4.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_HF;
//  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_16K;
  hsai_BlockA4.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_MCKDIV;
  hsai_BlockA4.Init.Mckdiv = 6;
  hsai_BlockA4.Init.MonoStereoMode = SAI_MONOMODE;
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

/* USER CODE BEGIN 4 */

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  hsai : pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
//	printf("Full Test\r\n");
	bufferStatus |= BUFFER_OFFSET_FULL;
}

/**
  * @brief  Rx Transfer Half completed callbacks
  * @param  hsai : pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
//	printf("Half Test\r\n");
	bufferStatus |= BUFFER_OFFSET_HALF;
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
