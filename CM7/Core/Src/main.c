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
typedef enum {
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL,
} BUFFER_StateTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// SAI audio constants
#define AUDIO_FREQUENCY       16000U
#define AUDIO_CHANNEL_NUMBER  1U
#define AUDIO_BUFFER_SIZE     1024U
#define INFERENCE_BUFFER_SIZE 32768U //+ 25600U // ~3.5 sec of audio (2^16 + 128 * 200; 2^17 overloads .bss
											  // sec. per exp. of 2: 2^(x-10)/31.2 for n>=10). Add in multiples of 128
#define AUDIO_PCM_CHUNK_SIZE  16U
#define BITS_PER_SAMPLE		  16U

// PDM2PCM filter constants
#define DECIMATION_FACTOR     64U
#define PDM_WORDS_PER_CHUNK   (AUDIO_PCM_CHUNK_SIZE * DECIMATION_FACTOR / BITS_PER_SAMPLE)

// DSP noise reduction constants
#define FIR_TAPS        64

// Uncomment to enable headphone audio playback
// #define AUDIO_PLAYBACK
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

RTC_HandleTypeDef hrtc;

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockA4;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai4_a;

SD_HandleTypeDef hsd1;

UART_HandleTypeDef huart1;

MDMA_HandleTypeDef hmdma_mdma_channel0_sdmmc1_end_data_0;
/* USER CODE BEGIN PV */

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
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void fir_init(void);
void process_pcm_block(uint16_t *pcm_chunk);
static void dcache_invalidate(void *addr, uint32_t size);
static void dcache_clean(void *addr, uint32_t size);
static void SD_Card_Power_Test(void);
void create_wav_header(wav_header *header,
		int sample_rate, int num_channels, int bit_depth, int data_size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Re-implementation of printf() to operate with USART
 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

// SAI SETUP for audio retrieval and output
// input_buffer must be located in RAM_D3 since it is accessed by BDMA
uint16_t input_buffer[AUDIO_BUFFER_SIZE] __attribute__((section(".DATA_RAM_D3")));
uint16_t inference_buffer[INFERENCE_BUFFER_SIZE];
volatile uint32_t inference_buffPtr;
uint16_t output_buffer[AUDIO_BUFFER_SIZE];
volatile uint32_t output_buffPtr;

volatile BUFFER_StateTypeDef bufferStatus = BUFFER_OFFSET_NONE;

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
  SCB_EnableDCache();

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
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_SAI1_Init();
  MX_RTC_Init();
  MX_X_CUBE_AI_Init();
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
  input_init_response = HAL_SAI_Receive_DMA(&hsai_BlockA4, (uint8_t *)input_buffer, AUDIO_BUFFER_SIZE);
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

//  // Initialize FIR filter
//  fir_init();
//  // Initialize PDM2PCM filter
//  BSP_AUDIO_IN_PDMToPCM_Init(1, AUDIO_FREQUENCY, AUDIO_CHANNEL_NUMBER, AUDIO_CHANNEL_NUMBER);
//
//  SD_Card_Power_Test();
  
  HAL_NVIC_SetPriority(TAMP_STAMP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TAMP_STAMP_IRQn);

  //printf("Clock: %02lu:%02lu:%02lu\r\n", endTime.Hours, endTime.Minutes, endTime.Seconds);

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

  //Test the power consumption in standby mode
  HAL_Delay(2000);
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4);
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4);
  HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_7);
  HAL_PWREx_EnterSTANDBYMode(PWR_D1_DOMAIN);

  /* Initialize Rx buffer status */
  bufferStatus &= BUFFER_OFFSET_NONE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  // Wait for half-buffer
//	  if ((bufferStatus & BUFFER_OFFSET_HALF) == BUFFER_OFFSET_HALF)
//	  {
//		  // Invalidate cache to avoid data mismatch issues
//		  dcache_invalidate(&input_buffer[0], (AUDIO_BUFFER_SIZE/2)*sizeof(uint16_t));
//		  printf("Buffer half full! \r\n");
//		  // Process the first half of PDM buffer to PCM
//		  for (int i = 0; i < (AUDIO_BUFFER_SIZE/2) / PDM_WORDS_PER_CHUNK; i++)
//		  {
//			  uint16_t *pdm_chunk = &input_buffer[i * PDM_WORDS_PER_CHUNK];
//			  uint16_t  *pcm_out    = &output_buffer[output_buffPtr];
//			  BSP_AUDIO_IN_PDMToPCM(1, pdm_chunk, pcm_out); // Apply PDM2PCM filter
//			  process_pcm_block(pcm_out); // Denoise PCM output with FIR filter
//			  dcache_clean(pcm_out, AUDIO_PCM_CHUNK_SIZE * sizeof(int16_t)); // Clean cache to avoid data mismatch issues
//			  for (int j = 0; j < AUDIO_PCM_CHUNK_SIZE; j++) {
//				  inference_buffer[inference_buffPtr + j] = output_buffer[output_buffPtr + j];
//			  }
//			  inference_buffPtr += AUDIO_PCM_CHUNK_SIZE;
//			  output_buffPtr += AUDIO_PCM_CHUNK_SIZE;
//		  }
//		  bufferStatus &= ~BUFFER_OFFSET_HALF;
//
//		  //MX_X_CUBE_AI_Process(&output_buffer[])
//	  }
//
//	  // Wait for full-buffer
//	  if ((bufferStatus & BUFFER_OFFSET_FULL) == BUFFER_OFFSET_FULL)
//	  {
//		  // Invalidate cache to avoid data mismatch issues
//		  dcache_invalidate(&input_buffer[AUDIO_BUFFER_SIZE/2], (AUDIO_BUFFER_SIZE/2)*sizeof(uint16_t));
//		  printf("Buffer full! \r\n");
//
//		  // Process the second half the same way
//		  for (int i = 0; i < (AUDIO_BUFFER_SIZE/2) / PDM_WORDS_PER_CHUNK; i++)
//		  {
//			  uint16_t *pdm_chunk = &input_buffer[AUDIO_BUFFER_SIZE/2 + i * PDM_WORDS_PER_CHUNK];
//			  uint16_t  *pcm_out    = &output_buffer[output_buffPtr];
//			  BSP_AUDIO_IN_PDMToPCM(1, pdm_chunk, pcm_out);
//			  process_pcm_block(pcm_out);
//			  dcache_clean(pcm_out, AUDIO_PCM_CHUNK_SIZE * sizeof(int16_t));
//			  for (int j = 0; j < AUDIO_PCM_CHUNK_SIZE; j++) {
//				  inference_buffer[inference_buffPtr + j] = output_buffer[output_buffPtr + j];
//			  }
//			  inference_buffPtr += AUDIO_PCM_CHUNK_SIZE;
//			  output_buffPtr += AUDIO_PCM_CHUNK_SIZE;
//		  }
//		  bufferStatus &= ~BUFFER_OFFSET_FULL;
//	  }
//
//	  // Wrap pcm pointer if needed
//	  if (output_buffPtr >= AUDIO_BUFFER_SIZE)
//		  output_buffPtr = 0;
//	  if (inference_buffPtr >= INFERENCE_BUFFER_SIZE) {
//		  MX_X_CUBE_AI_Process(&inference_buffer[INFERENCE_BUFFER_SIZE/2], INFERENCE_BUFFER_SIZE);
//		  inference_buffPtr = 0;
//		  while(1) {}
//	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

// IRQ handler
void RTC_TAMP_STAMP_IRQHandler(void) {
    HAL_RTCEx_TamperTimeStampIRQHandler(&hrtc);
}

void HAL_RTCEx_TimeStampEventCallback(RTC_HandleTypeDef *hrtc) {
    RTC_TimeTypeDef ts; RTC_DateTypeDef ds;
    HAL_RTCEx_GetTimeStamp(hrtc, &ts, &ds, RTC_FORMAT_BIN); // time of the wake event
    printf("Wakeup event at %02lu:%02lu:%02lu on %02lu/%02lu/%04lu\r\n",
		   ts.Hours, ts.Minutes, ts.Seconds, ds.Month, ds.Date, 2000 + ds.Year);
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
  * @brief SD Card Power Test
  * @retval None
  */
static void SD_Card_Power_Test(void){
	FATFS FatFs;
	FIL Fil;
	FRESULT FR_Status;
	UINT WWC;

	// Mount the SD card
	FR_Status = f_mount(&FatFs, SDPath, 1);
	if (FR_Status != FR_OK){
		printf("Error! While Mounting SD Card, Error Code: (%i)\r\n", FR_Status);
	}
	printf("SD Card Mounted Successfully! \r\n\n");

	HAL_Delay(1000);
	// Toggle pin when starting to write
	HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_7);
	// Open a file for writing and write to it
	FR_Status = f_open(&Fil, "BuowSample.wav", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
	if(FR_Status != FR_OK)
	{
	  printf("Error! While Creating/Opening A New Text File, Error Code: (%i)\r\n", FR_Status);
	  return;
	}

	//create wav header for audio data
	printf("buow_pcm_size: %d\r\n", buow_pcm_size);
	//wav_header header;
	//create_wav_header(&header, 16000, 1, 16, buow_pcm_size);

	// Write Data To The Text File
	//f_puts("Writing to SD Card Over SDMMC\n", &Fil);
	//f_write(&Fil, &header, sizeof(wav_header), &WWC);
	//printf("Header Bytes Written: %d\r\n", WWC);
	f_write(&Fil, buow_pcm_buffer, buow_pcm_size, &WWC);
	printf("Data Bytes Written: %d\r\n", WWC);


	// Close The File
	f_close(&Fil);

	// Toggle pin when done writing
	HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_7);
}

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_AUGUST;
  sDate.Date = 0x13;
  sDate.Year = 0x25;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the TimeStamp
  */
  if (HAL_RTCEx_SetTimeStamp_IT(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_DEFAULT) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hsai_BlockA4.Init.NoDivider = SAI_MCK_OVERSAMPLING_ENABLE;
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
