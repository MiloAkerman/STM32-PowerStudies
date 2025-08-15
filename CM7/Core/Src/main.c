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
#include "buow_sample_pcm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE BEGIN PD */
#define INFERENCE_BUFFER_SIZE 65536U //+ 25600U // ~4 sec of audio
											 // sec. per exp. of 2: 2^(x-10)/31.2 for n>=10). Add in multiples of 128
// PDM2PCM filter constants
#define DECIMATION_FACTOR     64U
#define PDM_WORDS_PER_CHUNK   (AUDIO_PCM_CHUNK_SIZE * DECIMATION_FACTOR / BITS_PER_SAMPLE)

#define AUDIO_PCM_CHUNK_SIZE  16U
#define BITS_PER_SAMPLE		  16U
// DSP noise reduction constants
#define FIR_TAPS        64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

// Peripheral handlers
SD_HandleTypeDef hsd1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN 0 */
FATFS FatFs;

// SAI SETUP for audio retrieval and output
// input_buffer must be located in RAM_D3 since it is accessed by BDMA
uint16_t inference_buffer[INFERENCE_BUFFER_SIZE];
volatile uint32_t inference_buffPtr;

// FIR SETUP for denoising
/* 32-tap:
 * -42,  -62, -120, -203,  -196,  173,   634,  1030,
    827,  -183, -1491, -2570, -2067, 256,  4038,  7590,
    7590, 4038,  256, -2067, -2570, -1491, -183,   827,
    1030,  634,  173,  -196,  -203, -120,   -62,   -42
 */
const q15_t firCoeffs[FIR_TAPS] = {
		-12,  -18,  -38,  -60,  -65,  -34,   44,  189,
		384,  594,  766,  823,  690,  369,  -18, -420,
	   -711,  -793,  -591,  -140,  436, 1027, 1477, 1611,
	   1365,  711,  -52,  -740, -1214, -1328, -976,  -236,
		658, 1381, 1835, 1810, 1290,  415, -613, -1514,
	  -2074, -2052, -1442, -390,  738, 1728, 2417, 2571,
	   2127, 1222,  145, -821, -1511, -1704, -1296,  -496,
		477, 1250, 1640, 1578, 1034,  201, -766, -1492
};
static q15_t firState[FIR_TAPS + AUDIO_PCM_CHUNK_SIZE - 1];
static arm_fir_instance_q15 S;

uint32_t sample_n = 0;
bool writing = false;
/* USER CODE END 0 */
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SDMMC1_SD_Init(void);
/* USER CODE BEGIN PFP */
void fir_init(void);
void process_pcm_block(uint16_t *pcm_chunk);
static void dcache_invalidate(void *addr, uint32_t size);
static void dcache_clean(void *addr, uint32_t size);
static void sd_init(void);
static void sd_writepcm(uint16_t* pcm_buf, uint32_t pcm_size);
void create_wav_header(wav_header *header,
	int sample_rate, int num_channels, int bit_depth, int data_size);
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

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
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
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
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
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */

  // Initialize FIR filter
  fir_init();
  // Initialize fs write and mount SD card
  sd_init();
  
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

    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_FALLING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd1.Init.ClockDiv = 8;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PJ7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Config the FIR filter for PCM denoising
  * @param  None
  * @retval None
  */
void fir_init(void) {
    arm_fir_init_q15(&S, FIR_TAPS, (q15_t *)firCoeffs, firState, AUDIO_PCM_CHUNK_SIZE);
}

/**
  * @brief  Apply FIR filter to PCM chunk for denoising
  * @param  pcm_chunk : Pointer to PCM chunk to be processed
  * @retval None
  */
void process_pcm_block(uint16_t *pcm_chunk) {
    // Allocate a temporary buffer to keep original unfiltered PCM.
    q15_t filtered[AUDIO_PCM_CHUNK_SIZE];

    // Run the FIR: input is q15_t, but int16_t map directly to q15_t
    arm_fir_q15(&S,
                (q15_t *)pcm_chunk,  // source buffer
                filtered,            // destination buffer
				AUDIO_PCM_CHUNK_SIZE);     // number of samples

    // Copy back filtered output to pcm_chunk
    for (int i = 0; i < AUDIO_PCM_CHUNK_SIZE; i++) {
        pcm_chunk[i] = filtered[i];
    }
}

/**
  * @brief Create a WAV header
  * @param header Pointer to the wav_header structure
  * @param sample_rate Sample rate in Hz
  * @param num_channels Number of audio channels (1 for mono, 2 for stereo)
  * @param bit_depth Bit depth (e.g., 16 for PCM)
  * @param data_size Size of the audio data in bytes
  */
void create_wav_header(wav_header *header, int sample_rate, int num_channels, int bit_depth, int data_size) {
	// RIFF Header
	memcpy(header->riff_header, "RIFF", 4);
	header->wav_size = data_size + 36; // Total file size - 8 bytes for RIFF header
	memcpy(header->wave_header, "WAVE", 4);

	// Format Header
	memcpy(header->fmt_header, "fmt ", 4); // Note the trailing space
	header->fmt_chunk_size = 16; // For PCM
	header->audio_format = 1; // PCM
	header->num_channels = num_channels;
	header->sample_rate = sample_rate;
	header->byte_rate = sample_rate * num_channels * (bit_depth / 8);
	header->sample_alignment = num_channels * (bit_depth / 8);
	header->bit_depth = bit_depth;

	// Data Header
	memcpy(header->data_header, "data", 4);
	header->data_bytes = data_size; // Number of bytes in data
}

/**
 * @brief Mount SD card
 * @retval None
 */
static void sd_init(void) {
	FRESULT FR_Status;

	// Mount the SD card
	FR_Status = f_mount(&FatFs, SDPath, 1);
	if (FR_Status != FR_OK){
		printf("Error! While Mounting SD Card, Error Code: (%i)\r\n", FR_Status);
	}
	printf("SD Card Mounted Successfully! \r\n\n");
}

/**
  * @brief  Invalidate portion of DCache
  * @param  addr : Address of memory location to be invalidated
  * @param  size : Size of memory chunk to be invalidated
  * @retval None
  */
static void dcache_invalidate(void *addr, uint32_t size) {
    uint32_t a = (uint32_t)addr & ~31U;
    uint32_t s = ((size + 31U) & ~31U);
    SCB_InvalidateDCache_by_Addr((uint32_t*)a, s);
}

/**
  * @brief  Clean portion of DCache
  * @param  addr : Address of memory location to be cleaned
  * @param  size : Size of memory chunk to be cleaned
  * @retval None
  */
static void dcache_clean(void *addr, uint32_t size) {
    uint32_t a = (uint32_t)addr & ~31U;
    uint32_t s = ((size + 31U) & ~31U);
    SCB_CleanDCache_by_Addr((uint32_t*)a, s);
}

/**
  * @brief Write PCM buffer with header to .wav file
  * @retval None
  */
static void sd_writepcm(uint16_t* pcm_buf, uint32_t pcm_size){
	wav_header header;
	FIL Fil;
	FRESULT FR_Status;
	UINT WWC;
	char file_name_buffer[20];

	// Open a file for writing and write to it
	snprintf(file_name_buffer, 20, "sample%ld.wav", sample_n++); // format file name. 20 is max characters
	FR_Status = f_open(&Fil, file_name_buffer, FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
	if(FR_Status != FR_OK)
	{
	  printf("Error! While Creating/Opening A New .wav File, Error Code: (%i)\r\n", FR_Status);
	  return;
	}
	printf(".wav File created! Writing data...");

	// wav_header header
	create_wav_header(&header, 16000, 1, 16, pcm_size * sizeof(uint16_t));

	// TODO: Clocks are too slow. Both playback and SD are only holding 1sec at full speed.
	// Write Data To The Text File
	//f_puts("Writing to SD Card Over SDMMC\n", &Fil);
	printf("Passed size: %ld\r\n", pcm_size * sizeof(uint16_t));
	printf("Calculated size: %d (%d)\r\n", sizeof(pcm_buf), sizeof(pcm_buf)*sizeof(uint16_t));
	f_write(&Fil, &header, sizeof(wav_header), &WWC);
	f_write(&Fil, (uint8_t*) pcm_buf, pcm_size * sizeof(uint16_t), &WWC);
	printf("Data Bytes Written: %d\r\n", WWC);


	// Close The File
	f_close(&Fil);
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
