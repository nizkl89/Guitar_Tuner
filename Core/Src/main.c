/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for guitar tuner using STM32F103VET6 with CMSIS-DSP
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
#include "lcd.h"
#include "arm_math.h" // CMSIS-DSP library for FFT
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float real;
  float imag;
} complex_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_SIZE 2048
#define SAMPLING_RATE 4096 // Increased to capture E4 and harmonics
#define ADC_MAX 4095
#define MIN_MAG_THRESHOLD 0.1f // Minimum FFT magnitude to reject noise
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
uint16_t adc_buffer[2][FFT_SIZE] _attribute_((aligned(4))) = {{0xFFFF}, {0xFFFF}}; // Double buffer, initialized to 0xFFFF for debug
float fft_input[FFT_SIZE]; // Input buffer for CMSIS-DSP FFT
float fft_output[FFT_SIZE]; // Output buffer for CMSIS-DSP FFT (complex interleaved)
arm_rfft_fast_instance_f32 fft_handler; // CMSIS-DSP FFT instance
volatile uint8_t adc_complete = 0;
volatile uint8_t buffer_idx = 0;
volatile uint8_t processing_buffer = 0; // Buffer index for main loop processing

const float note_freqs[] = {82.41, 110.00, 146.83, 196.00, 246.94, 329.63}; // Standard guitar tuning frequencies
const char *note_names[] = {"E2", "A2", "D3", "G3", "B3", "E4"};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FSMC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void apply_hann_window(float *data, uint32_t N);
void high_pass_filter(float *data, uint32_t N);
float find_dominant_freq(float *fft_out, uint32_t N);
void map_to_note(float freq, char *note, char *status);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Apply Hann window to input data to reduce spectral leakage
  * @param  data: Input data array
  * @param  N: Number of samples
  * @retval None
  */
void apply_hann_window(float *data, uint32_t N) {
  for (uint32_t i = 0; i < N; i++) {
    data[i] *= 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (N - 1)));
  }
}

/**
  * @brief  Apply a simple high-pass filter to remove low-frequency noise
  * @param  data: Input data array
  * @param  N: Number of samples
  * @retval None
  */
void high_pass_filter(float *data, uint32_t N) {
  // Simple first-order high-pass filter with cutoff ~20 Hz
  float alpha = 0.98f; // Adjusted for lower cutoff
  float prev = 0.0f;
  for (uint32_t i = 0; i < N; i++) {
    float curr = data[i];
    data[i] = alpha * (prev + data[i] - (i > 0 ? data[i-1] : 0.0f));
    prev = curr;
  }
}

/**
  * @brief  Find dominant frequency from FFT output
  * @param  fft_out: FFT output array (complex interleaved)
  * @param  N: FFT size
  * @retval Dominant frequency in Hz
  */
float find_dominant_freq(float *fft_out, uint32_t N) {
  float max_mag = 0.0f;
  uint32_t max_idx = 0;
  uint32_t min_idx = (uint32_t)(50.0f * N / SAMPLING_RATE); // Min 50 Hz
  uint32_t max_idx_limit = (uint32_t)(400.0f * N / SAMPLING_RATE); // Max 400 Hz
  for (uint32_t i = min_idx; i < max_idx_limit && i < N / 2; i++) {
    float mag;
    arm_sqrt_f32(fft_out[2*i] * fft_out[2*i] + fft_out[2*i+1] * fft_out[2*i+1], &mag);
    if (mag > max_mag) {
      max_mag = mag;
      max_idx = i;
    }
  }
  // Debug signal magnitude
  char buff[50];
  sprintf(buff, "Max Mag: %d.%02d    ", (int)max_mag, (int)((max_mag - (int)max_mag) * 100));
  LCD_DrawString(20, 150, buff);

  // Check for sufficient signal strength
  if (max_mag < MIN_MAG_THRESHOLD) {
    LCD_DrawString(20, 110, "No Signal          ");
    return 0.0f; // Return 0 Hz for no valid signal
  }

  float freq = (float)max_idx * SAMPLING_RATE / N;
  if (max_idx > 0 && max_idx < N / 2 - 1) {
    float y1, y2, y3;
    arm_sqrt_f32(fft_out[2*(max_idx-1)] * fft_out[2*(max_idx-1)] + fft_out[2*(max_idx-1)+1] * fft_out[2*(max_idx-1)+1], &y1);
    y2 = max_mag;
    arm_sqrt_f32(fft_out[2*(max_idx+1)] * fft_out[2*(max_idx+1)] + fft_out[2*(max_idx+1)+1] * fft_out[2*(max_idx+1)+1], &y3);
    float p = (y3 - y1) / (2.0f * (2.0f * y2 - y1 - y3));
    freq = (float)(max_idx + p) * SAMPLING_RATE / N;
  }
  // Debug raw frequency
  sprintf(buff, "Raw Freq: %d.%02d Hz    ", (int)freq, (int)((freq - (int)freq) * 100));
  LCD_DrawString(20, 110, buff);
  // Check for second harmonic for guitar signals, only for freq < 220 Hz
  if (freq < 220.0f) {
    float fund = freq / 2;
    for (int i = 0; i < 6; i++) {
      if (fabs(fund - note_freqs[i]) < 5.0f) {
        freq = fund;
        break;
      }
    }
  }
  return freq;
}

/**
  * @brief  Map detected frequency to nearest guitar note and tuning status
  * @param  freq: Detected frequency
  * @param  note: Output buffer for note name
  * @param  status: Output buffer for tuning status
  * @retval None
  */
void map_to_note(float freq, char *note, char *status) {
  if (freq < 50.0f) {
    strcpy(note, "--");
    strcpy(status, "No Signal");
    return;
  }
  float min_diff = 1000.0f;
  int closest_note = 0;
  for (int i = 0; i < 6; i++) {
    float diff = fabs(freq - note_freqs[i]);
    if (diff < min_diff) {
      min_diff = diff;
      closest_note = i;
    }
  }
  strcpy(note, note_names[closest_note]);
  float target = note_freqs[closest_note];
  if (fabs(freq - target) / target < 0.02f)
    strcpy(status, "In Tune");
  else if (freq < target)
    strcpy(status, "Flat");
  else
    strcpy(status, "Sharp");
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
  // Initialize CMSIS-DSP FFT
  arm_rfft_fast_init_f32(&fft_handler, FFT_SIZE);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FSMC_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // Reset LCD (PD12, active-low)
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_Delay(200);

  // Backlight/control (PE1, active-high)
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_Delay(50);

  LCD_INIT();
  HAL_Delay(200);
  LCD_DrawString(20, 10, "Tuner Starting...");

  HAL_TIM_Base_Start(&htim3);
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer[0], FFT_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char buff[120];
  float freq;
  char note[10], status[20];
  while (1)
  {
    if (adc_complete) {
      adc_complete = 0; // Reset flag immediately to avoid reprocessing
      uint8_t buf_to_process = processing_buffer; // Use synchronized buffer index

      // Debug: Display first few ADC values to check buffer contents
      sprintf(buff, "Buf %d: %u %u %u    ", buf_to_process,
              adc_buffer[buf_to_process][0], adc_buffer[buf_to_process][1],
              adc_buffer[buf_to_process][2]);
      LCD_DrawString(20, 130, buff);

      // Display raw ADC value
      sprintf(buff, "ADC: %u    ", adc_buffer[buf_to_process][0]);
      LCD_DrawString(20, 30, buff);

      // Check for ADC overrun (OVR bit is bit 5 in ADC_SR)
      if (hadc1.Instance->SR & (1U << 5)) {
        LCD_DrawString(20, 170, "ADC Overrun");
      }

      // Remove DC offset and prepare FFT input
      float mean = 0.0f;
      for (uint32_t i = 0; i < FFT_SIZE; i++) {
        mean += adc_buffer[buf_to_process][i];
      }
      mean /= FFT_SIZE;
      for (uint32_t i = 0; i < FFT_SIZE; i++) {
        fft_input[i] = (float)(adc_buffer[buf_to_process][i] - mean) / (ADC_MAX / 2.0f); // Scale to [-1, 1]
      }

      // Apply high-pass filter to remove low-frequency noise from guitar signal
      high_pass_filter(fft_input, FFT_SIZE);

      // Apply Hann window
      apply_hann_window(fft_input, FFT_SIZE);

      // Compute FFT using CMSIS-DSP
      arm_rfft_fast_f32(&fft_handler, fft_input, fft_output, 0);
      freq = find_dominant_freq(fft_output, FFT_SIZE);

      // Get note and status
      map_to_note(freq, note, status);

      // Find note frequency
      float note_freq = 0.0f;
      for (int i = 0; i < 6; i++) {
        if (strcmp(note, note_names[i]) == 0) {
          note_freq = note_freqs[i];
          break;
        }
      }

      // Display note with standard frequency
      int note_int_part = (int)note_freq;
      int note_frac_part = (int)((note_freq - note_int_part) * 100);
      sprintf(buff, "Note: %s (%d.%02d Hz)    ", note, note_int_part, note_frac_part);
      LCD_DrawString(20, 50, buff);

      // Display status
      sprintf(buff, "Status: %s    ", status);
      LCD_DrawString(20, 70, buff);

      // Display current frequency
      int freq_int_part = (int)freq;
      int freq_frac_part = (int)((freq - freq_int_part) * 100);
      sprintf(buff, "Freq: %d.%02d Hz    ", freq_int_part, freq_frac_part);
      LCD_DrawString(20, 90, buff);

      // LED indicators
      if (strcmp(status, "In Tune") == 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Green on
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 | GPIO_PIN_5, GPIO_PIN_SET);
      } else if (strcmp(status, "Flat") == 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // Red on
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
      } else if (strcmp(status, "Sharp") == 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // Blue on
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_5, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5, GPIO_PIN_SET); // All off for no signal
      }
    }
    HAL_Delay(10); // Reduced delay to improve responsiveness
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5; // Increased for stability
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 976;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 35; // (72 MHz / (35 + 1)) = 2 MHz
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 487;  // 2 MHz / (487 + 1) ≈ 4096 Hz
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Enable DMA controller clock
  * @retval None
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0); // Highest priority
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  /* Configure DMA for ADC1 */
  hdma_adc1.Instance = DMA1_Channel1;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_NORMAL; // Normal mode for simplicity
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pins : PB0 PB1 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

/**
  * @brief FSMC initialization function
  * @retval None
  */
static void MX_FSMC_Init(void)
{
  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_AFIO_FSMCNADV_DISCONNECTED();
}

/* USER CODE BEGIN 4 */
/**
  * @brief  ADC conversion complete callback
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    // Verify DMA state before proceeding
    if (HAL_DMA_GetState(&hdma_adc1) == HAL_DMA_STATE_READY) {
      // Critical section to synchronize buffer toggle
      __disable_irq();
      adc_complete = 1;
      buffer_idx = !buffer_idx;
      processing_buffer = !buffer_idx;
      __enable_irq();

      // Stop current DMA transfer
      if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK) {
        LCD_DrawString(20, 150, "DMA Stop Error");
      }

      // Start DMA transfer for the new buffer
      if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer[buffer_idx], FFT_SIZE) != HAL_OK) {
        LCD_DrawString(20, 150, "DMA Start Error");
      }
    } else {
      // Log DMA state for debugging
      char buff[50];
      sprintf(buff, "DMA State: %d", HAL_DMA_GetState(&hdma_adc1));
      LCD_DrawString(20, 150, buff);
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
