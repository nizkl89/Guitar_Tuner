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

typedef enum {
  TUNE_IDLE,
  TUNE_PULSE_ON,
  TUNE_PULSE_OFF
} TuneState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_SIZE 2048
#define SAMPLING_RATE 4098.36f // Adjusted to match actual TIM3 frequency
#define ADC_MAX 4095
#define MIN_MAG_THRESHOLD 0.1f // Default FFT magnitude threshold
#define BUTTON_WIDTH 60
#define BUTTON_HEIGHT 40
#define BUTTON_SPACING 10
#define BACK_BUTTON_WIDTH 60
#define BACK_BUTTON_HEIGHT 30
#define START_BUTTON_WIDTH 60
#define START_BUTTON_HEIGHT 30
#define PULSE_DURATION_MS 300 // 300 ms pulse for noticeable movement
#define PULSE_DUTY_CYCLE 750 // 75% PWM duty cycle (0-999)
#define PULSE_DUTY_CYCLE_E2 1000 // 100% PWM duty cycle for E2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
uint16_t adc_buffer[2][FFT_SIZE] __attribute__((aligned(4))) = {{0xFFFF}, {0xFFFF}}; // Double buffer
float fft_input[FFT_SIZE]; // Input buffer for CMSIS-DSP FFT
float fft_output[FFT_SIZE]; // Output buffer for CMSIS-DSP FFT (complex interleaved)
arm_rfft_fast_instance_f32 fft_handler; // CMSIS-DSP FFT instance
volatile uint8_t adc_complete = 0;
volatile uint8_t buffer_idx = 0;
volatile uint8_t processing_buffer = 0; // Buffer index for main loop processing

float global_raw_freq = 0.0f; // Store raw frequency from find_dominant_freq

const float note_freqs[] = {82.41, 110.00, 146.83, 196.00, 246.94, 329.63}; // Standard guitar tuning frequencies
const float note_tolerances[] = {5.0f, 3.0f, 10.0f, 10.0f, 5.0f, 40.0f}; // Increased tolerance for E4
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
Button_t start_button = {90, 90 + START_BUTTON_WIDTH, 280, 280 + START_BUTTON_HEIGHT, "Start", 0.0f};
int selected_note_idx = -1; // No note selected initially
ScreenState screen_state = STATE_NOTE_SELECT; // Start with note selection
uint8_t tuning_active = 0; // 0: stopped, 1: tuning
TuneState tune_state = TUNE_IDLE;
uint32_t tune_timer = 0; // For pulse timing
int8_t tune_direction = 0; // 1: clockwise (sharpen), -1: counterclockwise (flatten), 0: none
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FSMC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void apply_hann_window(float *data, uint32_t N);
void high_pass_filter(float *data, uint32_t N);
float find_dominant_freq(float *fft_out, uint32_t N);
float find_dominant_freq_e2(float *fft_out, uint32_t N);
void map_to_note(float freq, float target_freq, char *note, char *status);
void draw_buttons(void);
void draw_data_screen(void);
void check_touch(void);
void motor_control(int8_t direction, uint8_t enable);
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
  // Simple first-order high-pass filter with cutoff ~30 Hz
  float alpha = 0.99f; // Increased to attenuate frequencies below 50 Hz
  float prev = 0.0f;
  for (uint32_t i = 0; i < N; i++) {
    float curr = data[i];
    data[i] = alpha * (prev + data[i] - (i > 0 ? data[i-1] : 0.0f));
    prev = curr;
  }
}

/**
  * @brief  Find dominant frequency for E2 without subharmonic correction
  * @param  fft_out: FFT output array (complex interleaved)
  * @param  N: FFT size
  * @retval Dominant frequency in Hz
  */
float find_dominant_freq_e2(float *fft_out, uint32_t N) {
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
  float threshold = (max_idx * SAMPLING_RATE / N > 300.0f) ? 0.02f : MIN_MAG_THRESHOLD;
  if (max_mag < threshold) {
    if (screen_state == STATE_DATA_DISPLAY) {
      LCD_DrawString_Color(20, 140, "No Signal          ", WHITE, BLACK);
    }
    global_raw_freq = 0.0f;
    return 0.0f; // Return 0 Hz for no valid signal
  }

  float freq = (float)max_idx * SAMPLING_RATE / N;
  if (max_idx > 0 && max_idx < N / 2 - 1) {
    float y1, y2, y3;
    arm_sqrt_f32(fft_out[2*(max_idx-1)] * fft_out[2*(max_idx-1)] + fft_out[2*(max_idx-1)+1] * fft_out[2*(max_idx-1)+1], &y1);
    y2 = max_mag;
    arm_sqrt_f32(fft_out[2*(max_idx+1)] * fft_out[2*(max_idx+1)] + fft_out[2*(max_idx+1)+1] * fft_out[2*(max_idx+1)+1], &y3);
    float p = (y3 - y1) / (2.0f * (2.0f * y2 - y1 - y3));
    if (p < -0.5f) p = -0.5f;
    if (p > 0.5f) p = 0.5f;
    freq = (float)(max_idx + p) * SAMPLING_RATE / N;
  }

  // Store raw frequency
  global_raw_freq = freq;

  // Harmonic correction: Check for second harmonic (freq > 150 Hz, fund ~82.41 Hz)
  if (freq > 150.0f) {
    float fund = freq / 2;
    if (fund < 100.0f && fabs(fund - note_freqs[0]) < note_tolerances[0]) { // Check E2 fundamental
      uint32_t fund_idx = (uint32_t)(fund * N / SAMPLING_RATE);
      float fund_mag;
      arm_sqrt_f32(fft_out[2*fund_idx] * fft_out[2*fund_idx] + fft_out[2*fund_idx+1] * fft_out[2*fund_idx+1], &fund_mag);
      if (fund_mag > MIN_MAG_THRESHOLD / 4.0f) {
        freq = fund;
      }
    }
  }

  // No subharmonic correction for E2 to prevent doubling of 70-80 Hz

  // Apply 2 Hz offset to final frequency
  global_raw_freq = freq + 2.0f;
  return freq + 2.0f;
}

/**
  * @brief  Find dominant frequency from FFT output with enhanced harmonic and sharp note correction
  * @param  fft_out: FFT output array (complex interleaved)
  * @param  N: FFT size
  * @retval Dominant frequency in Hz
  */
float find_dominant_freq(float *fft_out, uint32_t N) {
  // Use E2-specific function if E2 is selected
  if (selected_note_idx == 0) {
    return find_dominant_freq_e2(fft_out, N);
  }

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
  float threshold = (max_idx * SAMPLING_RATE / N > 300.0f) ? 0.02f : MIN_MAG_THRESHOLD;
  if (max_mag < threshold) {
    if (screen_state == STATE_DATA_DISPLAY) {
      LCD_DrawString_Color(20, 140, "No Signal          ", WHITE, BLACK);
    }
    global_raw_freq = 0.0f;
    return 0.0f; // Return 0 Hz for no valid signal
  }

  float freq = (float)max_idx * SAMPLING_RATE / N;
  if (max_idx > 0 && max_idx < N / 2 - 1) {
    float y1, y2, y3;
    arm_sqrt_f32(fft_out[2*(max_idx-1)] * fft_out[2*(max_idx-1)] + fft_out[2*(max_idx-1)+1] * fft_out[2*(max_idx-1)+1], &y1);
    y2 = max_mag;
    arm_sqrt_f32(fft_out[2*(max_idx+1)] * fft_out[2*(max_idx+1)] + fft_out[2*(max_idx+1)+1] * fft_out[2*(max_idx+1)+1], &y3);
    float p = (y3 - y1) / (2.0f * (2.0f * y2 - y1 - y3));
    if (p < -0.5f) p = -0.5f;
    if (p > 0.5f) p = 0.5f;
    freq = (float)(max_idx + p) * SAMPLING_RATE / N;
  }

  // Store raw frequency
  global_raw_freq = freq;

  // Early check for E4 fundamental (329.63 Hz, index 5)
  if (selected_note_idx == 5 && fabs(freq - note_freqs[5]) < note_tolerances[5]) {
    global_raw_freq = freq + 2.0f; // Apply 2 Hz offset
    return freq + 2.0f; // Accept E4 and nearby frequencies (including F4, F#4)
  }

  // Harmonic correction: Check for second harmonic (freq > 150 Hz, fund < 200 Hz)
  if (freq > 150.0f) {
    float fund = freq / 2;
    if (fund < 200.0f) { // Only check if fundamental is in plausible range
      for (int i = 0; i < 6; i++) {
        if (fabs(fund - note_freqs[i]) < note_tolerances[i]) {
          // Verify fundamental bin magnitude
          uint32_t fund_idx = (uint32_t)(fund * N / SAMPLING_RATE);
          float fund_mag;
          arm_sqrt_f32(fft_out[2*fund_idx] * fft_out[2*fund_idx] + fft_out[2*fund_idx+1] * fft_out[2*fund_idx+1], &fund_mag);
          float mag_threshold = (i == 0) ? MIN_MAG_THRESHOLD / 4.0f : MIN_MAG_THRESHOLD / 2.0f;
          if (fund_mag > mag_threshold) {
            freq = fund;
            break;
          }
        }
      }
    }
  }

  // Check for subharmonic (freq < 110 Hz, doubled freq matches a note)
  if (freq >= 50.0f && freq <= 400.0f) {
    if (freq < 110.0f) {
      float doubled = freq * 2;
      for (int i = 0; i < 6; i++) {
        if (fabs(doubled - note_freqs[i]) < note_tolerances[i]) {
          // Skip doubling for E2 (i == 0) if freq >= 60 Hz
          if (i == 0 && freq >= 60.0f) continue;
          uint32_t doubled_idx = (uint32_t)(doubled * N / SAMPLING_RATE);
          float doubled_mag;
          arm_sqrt_f32(fft_out[2*doubled_idx] * fft_out[2*doubled_idx] + fft_out[2*doubled_idx+1] * fft_out[2*doubled_idx+1], &doubled_mag);
          float subharmonic_threshold = (i == 0) ? MIN_MAG_THRESHOLD : MIN_MAG_THRESHOLD / 2.0f;
          // Ensure original freq magnitude is lower to confirm subharmonic
          uint32_t orig_idx = (uint32_t)(freq * N / SAMPLING_RATE);
          float orig_mag;
          arm_sqrt_f32(fft_out[2*orig_idx] * fft_out[2*orig_idx] + fft_out[2*orig_idx+1] * fft_out[2*orig_idx+1], &orig_mag);
          if (doubled_mag > subharmonic_threshold && doubled_mag > orig_mag) {
            freq = doubled;
            break;
          }
        }
      }
    }
  }

  // Sharp note correction for E4 (check for F4 ~349.23 Hz, F#4 ~369.99 Hz)
  if (freq >= 350.0f && freq <= 400.0f && selected_note_idx == 5) {
    const float sharp_notes[] = {349.23f, 369.99f}; // F4, F#4
    const float sharp_tolerances[] = {10.0f, 10.0f}; // ±10 Hz for sharp notes
    for (int i = 0; i < 2; i++) {
      if (fabs(freq - sharp_notes[i]) < sharp_tolerances[i]) {
        uint32_t sharp_idx = (uint32_t)(sharp_notes[i] * N / SAMPLING_RATE);
        float sharp_mag;
        arm_sqrt_f32(fft_out[2*sharp_idx] * fft_out[2*sharp_idx] + fft_out[2*sharp_idx+1] * fft_out[2*sharp_idx+1], &sharp_mag);
        if (sharp_mag > 0.02f) {
          global_raw_freq = sharp_notes[i] + 2.0f; // Apply 2 Hz offset
          return sharp_notes[i] + 2.0f; // Snap to known sharp note frequency
        }
      }
    }
  }

  // Apply 2 Hz offset to final frequency
  global_raw_freq = freq + 2.0f;
  return freq + 2.0f; // Return detected frequency with offset
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
  * @brief  Control L298N motor driver
  * @param  direction: 1 (clockwise), -1 (counterclockwise), 0 (stop)
  * @param  enable: 1 (motor on), 0 (motor off)
  * @retval None
  */
void motor_control(int8_t direction, uint8_t enable) {
  if (enable) {
    // Use 100% duty cycle for E2, 75% for others
    uint16_t duty_cycle = (selected_note_idx == 0) ? PULSE_DUTY_CYCLE_E2 : PULSE_DUTY_CYCLE;
    if (direction == 1) { // Clockwise (sharpen all strings)
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);   // IN1 high
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // IN2 low
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty_cycle);
    } else if (direction == -1) { // Counterclockwise (flatten all strings)
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // IN1 low
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   // IN2 high
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty_cycle);
    }
  } else {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | GPIO_PIN_8, GPIO_PIN_RESET); // Both low
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0); // PWM off
  }
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
    uint16_t text_x = buttons[i].x_min + (BUTTON_WIDTH - strlen(buttons[i].note) * 8) / 2;
    uint16_t text_y = buttons[i].y_min + (BUTTON_HEIGHT - 16) / 2;
    LCD_DrawString_Color(text_x, text_y, buttons[i].note, WHITE, BLACK);
  }
  __enable_irq();
}

/**
  * @brief  Draw data screen with tuning information, Back button, and Start/Stop button
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
  uint16_t text_x = back_button.x_min + (BACK_BUTTON_WIDTH - strlen(back_button.note) * 8) / 2;
  uint16_t text_y = back_button.y_min + (BACK_BUTTON_HEIGHT - 16) / 2;
  LCD_DrawString_Color(text_x, text_y, back_button.note, WHITE, BLACK);
  // Draw Start/Stop button
  LCD_DrawLine(start_button.x_min, start_button.y_min, start_button.x_max, start_button.y_min, BLACK);
  LCD_DrawLine(start_button.x_max, start_button.y_min, start_button.x_max, start_button.y_max, BLACK);
  LCD_DrawLine(start_button.x_max, start_button.y_max, start_button.x_min, start_button.y_max, BLACK);
  LCD_DrawLine(start_button.x_min, start_button.y_max, start_button.x_min, start_button.y_min, BLACK);
  text_x = start_button.x_min + (START_BUTTON_WIDTH - strlen(start_button.note) * 8) / 2;
  text_y = start_button.y_min + (START_BUTTON_HEIGHT - 16) / 2;
  LCD_DrawString_Color(text_x, text_y, start_button.note, WHITE, BLACK);
  // Draw "Tuning" if active
  if (tuning_active) {
    text_x = start_button.x_min + (START_BUTTON_WIDTH - 6 * 8) / 2; // "Tuning" = 6 chars
    LCD_DrawString_Color(text_x, 250, "Tuning", WHITE, BLACK);
  }
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
          tuning_active = 0; // Reset tuning
          start_button.note = "Start";
          tune_state = TUNE_IDLE;
          motor_control(0, 0); // Stop motor
          draw_data_screen();
          break;
        }
      }
    } else if (screen_state == STATE_DATA_DISPLAY) {
      // Check Back button
      if (touch_coord.x >= back_button.x_min && touch_coord.x <= back_button.x_max &&
          touch_coord.y >= back_button.y_min && touch_coord.y <= back_button.y_max) {
        selected_note_idx = -1;
        screen_state = STATE_NOTE_SELECT;
        adc_complete = 0; // Prevent immediate data redraw
        tuning_active = 0; // Stop tuning
        start_button.note = "Start";
        tune_state = TUNE_IDLE;
        motor_control(0, 0); // Stop motor
        draw_buttons();
      }
      // Check Start/Stop button
      else if (touch_coord.x >= start_button.x_min && touch_coord.x <= start_button.x_max &&
               touch_coord.y >= start_button.y_min && touch_coord.y <= start_button.y_max) {
        __disable_irq(); // Protect display update
        // Clear Start/Stop button area
        uint16_t text_x = start_button.x_min;
        uint16_t text_y = start_button.y_min + (START_BUTTON_HEIGHT - 16) / 2;
        LCD_DrawString_Color(text_x, text_y, "        ", WHITE, BLACK); // Clear 8 chars to cover button width
        // Clear Tuning text area
        text_x = start_button.x_min;
        LCD_DrawString_Color(text_x, 250, "        ", WHITE, BLACK); // Clear 8 chars to cover "Tuning"

        // Toggle tuning state
        tuning_active = !tuning_active;
        tune_state = TUNE_IDLE; // Reset pulse state
        if (tuning_active) {
          start_button.note = "Stop";
        } else {
          start_button.note = "Start";
          motor_control(0, 0); // Stop motor
        }

        // Redraw Start/Stop button
        LCD_DrawLine(start_button.x_min, start_button.y_min, start_button.x_max, start_button.y_min, BLACK);
        LCD_DrawLine(start_button.x_max, start_button.y_min, start_button.x_max, start_button.y_max, BLACK);
        LCD_DrawLine(start_button.x_max, start_button.y_max, start_button.x_min, start_button.y_max, BLACK);
        LCD_DrawLine(start_button.x_min, start_button.y_max, start_button.x_min, start_button.y_min, BLACK);
        text_x = start_button.x_min + (START_BUTTON_WIDTH - strlen(start_button.note) * 8) / 2;
        text_y = start_button.y_min + (START_BUTTON_HEIGHT - 16) / 2;
        LCD_DrawString_Color(text_x, text_y, start_button.note, WHITE, BLACK);

        // Update Tuning text
        if (tuning_active) {
          text_x = start_button.x_min + (START_BUTTON_WIDTH - 6 * 8) / 2; // "Tuning" = 6 chars
          LCD_DrawString_Color(text_x, 250, "Tuning", WHITE, BLACK);
        }
        __enable_irq();
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
  MX_TIM4_Init();
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
  while (!XPT2046_Touch_Calibrate()); // Run calibration until successful
  #define macXPT2046_Coordinate_GramScan 1 // Portrait 240x320, top-left origin
  LCD_GramScan(macXPT2046_Coordinate_GramScan); // Set orientation
  LCD_Clear(0, 0, 240, 320, BACKGROUND);
  draw_buttons();

  // Start ADC and timer
  HAL_TIM_Base_Start(&htim3);
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer[0], FFT_SIZE);

  // Start PWM for motor
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  motor_control(0, 0); // Ensure motor is off initially
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
      LCD_DrawString_Color(20, 20, buff, WHITE, BLACK);

      LCD_DrawString_Color(20, 50, "                    ", WHITE, BLACK); // Clear previous target
      int target_int_part = (int)buttons[selected_note_idx].freq;
      int target_frac_part = (int)((buttons[selected_note_idx].freq - target_int_part) * 100);
      sprintf(buff, "Target: %s %d.%02d Hz", note, target_int_part, target_frac_part);
      LCD_DrawString_Color(20, 50, buff, WHITE, BLACK);

      LCD_DrawString_Color(20, 80, "                    ", WHITE, BLACK); // Clear previous status
      sprintf(buff, "Status: %s", status);
      LCD_DrawString_Color(20, 80, buff, WHITE, BLACK);

      LCD_DrawString_Color(20, 110, "                    ", WHITE, BLACK); // Clear previous frequency
      int freq_int_part = (int)freq;
      int freq_frac_part = (int)((freq - freq_int_part) * 100);
      sprintf(buff, "Freq: %d.%02d Hz", freq_int_part, freq_frac_part);
      LCD_DrawString_Color(20, 110, buff, WHITE, BLACK);

      LCD_DrawString_Color(20, 140, "                    ", WHITE, BLACK); // Clear previous raw frequency
      if (global_raw_freq >= 50.0f) {
        sprintf(buff, "Raw Freq: %d.%02d Hz", (int)global_raw_freq, (int)((global_raw_freq - (int)global_raw_freq) * 100));
        LCD_DrawString_Color(20, 140, buff, WHITE, BLACK);
      } else {
        LCD_DrawString_Color(20, 140, "No Signal          ", WHITE, BLACK);
      }

      if (hadc1.Instance->SR & (1U << 5)) {
        LCD_DrawString_Color(20, 170, "ADC Overrun", WHITE, BLACK);
      } else {
        LCD_DrawString_Color(20, 170, "                ", WHITE, BLACK); // Clear overrun message
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

      // Tuning logic: Only turn motor if freq is within ±40 Hz of target and >= 60 Hz
      if (tuning_active && tune_state == TUNE_IDLE) {
        if (freq >= 60 && fabs(freq - buttons[selected_note_idx].freq) <= 40.0f) {
          if (strcmp(status, "Sharp") == 0) {
            tune_direction = -1; // Counterclockwise to flatten
            tune_state = TUNE_PULSE_ON;
            tune_timer = HAL_GetTick();
            motor_control(tune_direction, 1);
          } else if (strcmp(status, "Flat") == 0) {
            tune_direction = 1; // Clockwise to sharpen
            tune_state = TUNE_PULSE_ON;
            tune_timer = HAL_GetTick();
            motor_control(tune_direction, 1);
            HAL_Delay(500);
          }
        }
      }
    }

    // Handle tuning pulse
    if (tuning_active && tune_state != TUNE_IDLE) {
      uint32_t current_time = HAL_GetTick();
      switch (tune_state) {
        case TUNE_PULSE_ON:
          if (current_time - tune_timer >= PULSE_DURATION_MS) {
            motor_control(0, 0); // Stop motor
            tune_state = TUNE_PULSE_OFF;
            tune_timer = current_time;
          }
          break;
        case TUNE_PULSE_OFF:
          if (current_time - tune_timer >= PULSE_DURATION_MS) {
            tune_state = TUNE_IDLE; // Ready for next tuning cycle
            tune_direction = 0;
          }
          break;
        default:
          break;
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
  htim3.Init.Period = 487;  // 2 MHz / (487 + 1) ≈ 4098.36 Hz
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71; // 72 MHz / (71+1) = 1 MHz
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999; // 1 MHz / (999+1) = 1 kHz
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_8, GPIO_PIN_RESET); // Motor off

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

  /*Configure GPIO pins : PA2 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  /*Configure GPIO pin : PB6 (TIM4_CH1) */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
