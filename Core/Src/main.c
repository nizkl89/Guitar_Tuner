/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for guitar tuner using STM32F103VET6 with CMSIS-DSP and XPT2046 touch screen
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
#include "lcdtp.h"
#include "xpt2046.h"
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

typedef struct {
  uint16_t x_min;
  uint16_t x_max;
  uint16_t y_min;
  uint16_t y_max;
  const char *note;
  float freq;
} Button_t;

typedef enum {
  STATE_NOTE_SELECT,
  STATE_DATA_DISPLAY
} ScreenState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_SIZE 2048
#define SAMPLING_RATE 4096 // Increased to capture E4 and harmonics
#define ADC_MAX 4095
#define MIN_MAG_THRESHOLD 0.1f // Minimum FFT magnitude to reject noise
#define BUTTON_WIDTH 60
#define BUTTON_HEIGHT 40
#define BUTTON_SPACING 10
#define BACK_BUTTON_WIDTH 60
#define BACK_BUTTON_HEIGHT 30
#define WIDTH_EN_CHAR 16
#define HEIGHT_EN_CHAR 24
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
uint16_t adc_buffer[2][FFT_SIZE] __attribute__((aligned(4))) = {{0xFFFF}, {0xFFFF}}; // Double buffer
float fft_input[FFT_SIZE]; // Input buffer for CMSIS-DSP FFT
float fft_output[FFT_SIZE]; // Output buffer for CMSIS-DSP FFT (complex interleaved)
arm_rfft_fast_instance_f32 fft_handler; // CMSIS-DSP FFT instance
volatile uint8_t adc_complete = 0;
volatile uint8_t buffer_idx = 0;
volatile uint8_t processing_buffer = 0; // Buffer index for main loop processing

const float note_freqs[] = {82.41, 110.00, 146.83, 196.00, 246.94, 329.63}; // Standard guitar tuning frequencies
const char *note_names[] = {"E2", "A2", "D3", "G3", "B3", "E4"};
Button_t buttons[] = {
  {20, 20 + BUTTON_WIDTH, 40, 40 + BUTTON_HEIGHT, "E2", 82.41},
  {90, 90 + BUTTON_WIDTH, 40, 40 + BUTTON_HEIGHT, "A2", 110.00},
  {160, 160 + BUTTON_WIDTH, 40, 40 + BUTTON_HEIGHT, "D3", 146.83},
  {20, 20 + BUTTON_WIDTH, 100, 100 + BUTTON_HEIGHT, "G3", 196.00},
  {90, 90 + BUTTON_WIDTH, 100, 100 + BUTTON_HEIGHT, "B3", 246.94},
  {160, 160 + BUTTON_WIDTH, 100, 100 + BUTTON_HEIGHT, "E4", 329.63}
};
const uint8_t num_buttons = 6;
Button_t back_button = {10, 10 + BACK_BUTTON_WIDTH, 280, 280 + BACK_BUTTON_HEIGHT, "Back", 0.0f};
int selected_note_idx = -1; // No note selected initially
ScreenState screen_state = STATE_NOTE_SELECT; // Start with note selection
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
void map_to_note(float freq, float target_freq, char *note, char *status);
void draw_buttons(void);
void draw_data_screen(void);
void check_touch(void);
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
  // Check for sufficient signal strength
  if (max_mag < MIN_MAG_THRESHOLD) {
    if (screen_state == STATE_DATA_DISPLAY) {
      LCD_DrawString(20, 140, "No Signal          ");
    }
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
  if (screen_state == STATE_DATA_DISPLAY) {
    // Debug raw frequency
    char buff[50];
    sprintf(buff, "Raw Freq: %d.%02d Hz", (int)freq, (int)((freq - (int)freq) * 100));
    LCD_DrawString(20, 140, buff);
  }
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
  * @brief  Map detected frequency to tuning status relative to target note
  * @param  freq: Detected frequency
  * @param  target_freq: Target note frequency
  * @param  note: Output buffer for note name
  * @param  status: Output buffer for tuning status
  * @retval None
  */
void map_to_note(float freq, float target_freq, char *note, char *status) {
  strcpy(note, note_names[selected_note_idx]); // Always set to selected note
  if (freq < 50.0f) {
    strcpy(status, "No Signal");
    return;
  }
  if (fabs(freq - target_freq) / target_freq < 0.02f)
    strcpy(status, "In Tune");
  else if (freq < target_freq)
    strcpy(status, "Flat");
  else
    strcpy(status, "Sharp");
}

/**
  * @brief  Draw buttons for guitar notes
  * @param  None
  * @retval None
  */
void draw_buttons(void) {
  __disable_irq(); // Prevent interrupts during screen update
  LCD_Clear(0, 0, 240, 320, BACKGROUND); // Double clear for reliability
  LCD_Clear(0, 0, 240, 320, BACKGROUND);
  for (uint8_t i = 0; i < num_buttons; i++) {
    // Draw button rectangle
    LCD_DrawLine(buttons[i].x_min, buttons[i].y_min, buttons[i].x_max, buttons[i].y_min, BLACK);
    LCD_DrawLine(buttons[i].x_max, buttons[i].y_min, buttons[i].x_max, buttons[i].y_max, BLACK);
    LCD_DrawLine(buttons[i].x_max, buttons[i].y_max, buttons[i].x_min, buttons[i].y_max, BLACK);
    LCD_DrawLine(buttons[i].x_min, buttons[i].y_max, buttons[i].x_min, buttons[i].y_min, BLACK);
    // Draw note name
    uint16_t text_x = buttons[i].x_min + (BUTTON_WIDTH - strlen(buttons[i].note) * WIDTH_EN_CHAR) / 2;
    uint16_t text_y = buttons[i].y_min + (BUTTON_HEIGHT - HEIGHT_EN_CHAR) / 2;
    LCD_DrawString_Color(text_x, text_y, buttons[i].note, WHITE, BLACK);
  }
  __enable_irq();
}

/**
  * @brief  Draw data screen with tuning information and Back button
  * @param  None
  * @retval None
  */
void draw_data_screen(void) {
  __disable_irq(); // Prevent interrupts during screen update
  LCD_Clear(0, 0, 240, 320, BACKGROUND);
  // Draw Back button
  LCD_DrawLine(back_button.x_min, back_button.y_min, back_button.x_max, back_button.y_min, BLACK);
  LCD_DrawLine(back_button.x_max, back_button.y_min, back_button.x_max, back_button.y_max, BLACK);
  LCD_DrawLine(back_button.x_max, back_button.y_max, back_button.x_min, back_button.y_max, BLACK);
  LCD_DrawLine(back_button.x_min, back_button.y_max, back_button.x_min, back_button.y_min, BLACK);
  uint16_t text_x = back_button.x_min + (BACK_BUTTON_WIDTH - strlen(back_button.note) * WIDTH_EN_CHAR) / 2;
  uint16_t text_y = back_button.y_min + (BACK_BUTTON_HEIGHT - HEIGHT_EN_CHAR) / 2;
  LCD_DrawString_Color(text_x, text_y, back_button.note, WHITE, BLACK);
  __enable_irq();
}

/**
  * @brief  Check for touch input and update state
  * @param  None
  * @retval None
  */
void check_touch(void) {
  strType_XPT2046_Coordinate touch_coord;
  if (ucXPT2046_TouchFlag && XPT2046_Get_TouchedPoint(&touch_coord, &strXPT2046_TouchPara)) {
    if (screen_state == STATE_NOTE_SELECT) {
      for (uint8_t i = 0; i < num_buttons; i++) {
        if (touch_coord.x >= buttons[i].x_min && touch_coord.x <= buttons[i].x_max &&
            touch_coord.y >= buttons[i].y_min && touch_coord.y <= buttons[i].y_max) {
          selected_note_idx = i;
          screen_state = STATE_DATA_DISPLAY;
          adc_complete = 0; // Reset to ensure fresh data
          draw_data_screen();
          break;
        }
      }
    } else if (screen_state == STATE_DATA_DISPLAY) {
      if (touch_coord.x >= back_button.x_min && touch_coord.x <= back_button.x_max &&
          touch_coord.y >= back_button.y_min && touch_coord.y <= back_button.y_max) {
        selected_note_idx = -1;
        screen_state = STATE_NOTE_SELECT;
        adc_complete = 0; // Prevent immediate data redraw
        draw_buttons();
      }
    }
    ucXPT2046_TouchFlag = 0;
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

  // Initialize LCD
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_Delay(50);
  LCD_INIT();
  HAL_Delay(200);

  // Initialize touch screen
  macXPT2046_CS_DISABLE();
//  XPT2046_Init();
  while (!XPT2046_Touch_Calibrate()); // Run calibration until successful
  #define macXPT2046_Coordinate_GramScan 1 // Portrait 240x320, top-left origin
  LCD_GramScan(macXPT2046_Coordinate_GramScan); // Set orientation
  LCD_Clear(0, 0, 240, 320, BACKGROUND);
  draw_buttons();

  // Start ADC and timer
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
    // Check for touch input
    check_touch();

    if (screen_state == STATE_DATA_DISPLAY && adc_complete && selected_note_idx >= 0) {
      adc_complete = 0; // Reset flag
      uint8_t buf_to_process = processing_buffer;

      // Remove DC offset and prepare FFT input
      float mean = 0.0f;
      for (uint32_t i = 0; i < FFT_SIZE; i++) {
        mean += adc_buffer[buf_to_process][i];
      }
      mean /= FFT_SIZE;
      for (uint32_t i = 0; i < FFT_SIZE; i++) {
        fft_input[i] = (float)(adc_buffer[buf_to_process][i] - mean) / (ADC_MAX / 2.0f); // Scale to [-1, 1]
      }

      // Apply high-pass filter
      high_pass_filter(fft_input, FFT_SIZE);

      // Apply Hann window
      apply_hann_window(fft_input, FFT_SIZE);

      // Compute FFT
      arm_rfft_fast_f32(&fft_handler, fft_input, fft_output, 0);
      freq = find_dominant_freq(fft_output, FFT_SIZE);

      // Get note and status
      map_to_note(freq, buttons[selected_note_idx].freq, note, status);

      // Display data
      sprintf(buff, "ADC: %u", adc_buffer[buf_to_process][0]);
      LCD_DrawString(20, 20, buff);

      LCD_DrawString(20, 50, "                    "); // Clear previous target
      sprintf(buff, "Target: %s", note);
      LCD_DrawString(20, 50, buff);

      LCD_DrawString(20, 80, "                    "); // Clear previous status
      sprintf(buff, "Status: %s", status);
      LCD_DrawString(20, 80, buff);

      int freq_int_part = (int)freq;
      int freq_frac_part = (int)((freq - freq_int_part) * 100);
      sprintf(buff, "Freq: %d.%02d Hz", freq_int_part, freq_frac_part);
      LCD_DrawString(20, 110, buff);

      if (hadc1.Instance->SR & (1U << 5)) {
        LCD_DrawString(20, 170, "ADC Overrun");
      } else {
        LCD_DrawString(20, 170, "                "); // Clear overrun message
      }

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
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5, GPIO_PIN_SET); // All off
      }
    }
    HAL_Delay(20); // Increased delay to test timing
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pins : PB0 PB1 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PE0 PE1 PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
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
    if (HAL_DMA_GetState(&hdma_adc1) == HAL_DMA_STATE_READY) {
      // Critical section to synchronize buffer toggle
      __disable_irq();
      adc_complete = 1;
      buffer_idx = !buffer_idx;
      processing_buffer = !buffer_idx;
      __enable_irq();

      // Stop current DMA transfer
      if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK) {
        // Error handling without display
      }

      // Start DMA transfer for the new buffer
      if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer[buffer_idx], FFT_SIZE) != HAL_OK) {
        // Error handling without display
      }
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
