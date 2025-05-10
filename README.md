# STM32F103VET6 Guitar Tuner

This project implements a guitar tuner on the STM32F103VET6 microcontroller using a 2048-point FFT (via CMSIS-DSP) to detect note frequencies from an ADC-sampled microphone input. The tuner features a 240x320 LCD with an XPT2046 touch screen for selecting guitar notes (E2, A2, D3, G3, B3, E4) and an L298N motor driver for automatic tuning. The system supports manual note selection, displays tuning status ("In Tune," "Flat," "Sharp"), and controls a motor to adjust string tension.

## Features
- **Frequency Detection**: Uses a 2048-point FFT with a sampling rate of 4098.36 Hz to detect frequencies between 50 Hz and 400 Hz.
- **Signal Processing**:
  - Hann window to reduce spectral leakage.
  - High-pass filter to remove low-frequency noise.
  - Parabolic interpolation for sub-bin frequency resolution.
  - Harmonic and subharmonic corrections to handle guitar note harmonics.
- **User Interface**:
  - 240x320 LCD with touch support for selecting notes and starting/stopping tuning.
  - Displays target note, detected frequency, raw frequency, and tuning status.
  - LED indicators: Green (In Tune), Red (Flat), Blue (Sharp).
- **Automatic Tuning**:
  - L298N motor driver adjusts string tension with PWM control (100% duty cycle for E2, 75% for others).
  - 300 ms motor pulses to prevent over-tuning.
- **Robustness**:
  - Double-buffered ADC DMA to prevent data loss.
  - 2 Hz frequency offset to compensate for systematic errors.
  - Configurable tolerances per note (e.g., ±40 Hz for E4).

## Hardware Requirements
- **Microcontroller**: STM32F103VET6 (72 MHz Cortex-M3).
- **Display**: 240x320 LCD with XPT2046 touch controller.
- **Audio Input**: Microphone connected to ADC1 (Channel 3) with TIM3 triggering at 4098.36 Hz.
- **Motor Driver**: L298N for controlling a tuning motor (connected to GPIOA pins 2 and 8, PWM via TIM4).
- **LEDs**: Connected to GPIOB pins 0 (Green), 1 (Blue), 5 (Red).
- **Power Supply**: 3.3V for STM32, 5V for LCD, 12V for motor.

## Software Dependencies
- **STM32CubeIDE**: For building and flashing the project.
- **CMSIS-DSP**: For FFT and mathematical operations (included via `arm_math.h`).
- **HAL Libraries**: For ADC, DMA, TIM, GPIO, and FSMC configuration.
- **Custom Libraries**:
  - `lcdtp.h`: LCD and touch screen drivers.
  - `xpt2046.h`: Touch controller interface.

## Installation
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/stm32-guitar-tuner.git
   ```
2. **Open in STM32CubeIDE**:
   - Import the project into STM32CubeIDE.
   - Ensure CMSIS-DSP is included in the project (via STM32CubeMX or manual setup).
3. **Configure Hardware**:
   - Connect the LCD, touch screen, microphone, motor driver, and LEDs as per the GPIO mappings in `MX_GPIO_Init`.
   - Verify ADC, TIM3, TIM4, and FSMC configurations in `main.c`.
4. **Build and Flash**:
   - Build the project in STM32CubeIDE.
   - Flash the firmware to the STM32F103VET6 using an ST-Link debugger.
5. **Calibrate Touch Screen**:
   - The program runs `XPT2046_Touch_Calibrate` on startup. Follow on-screen instructions to calibrate the touch interface.

## Usage
1. **Power On**:
   - The system starts in note selection mode, displaying six buttons for E2, A2, D3, G3, B3, and E4.
2. **Select a Note**:
   - Tap a note button on the touch screen to select the target note.
   - The screen switches to data display mode, showing the target frequency, detected frequency, and tuning status.
3. **Play a Note**:
   - Pluck the corresponding guitar string near the microphone.
   - The LCD displays the detected frequency and status ("In Tune," "Flat," "Sharp").
   - LEDs indicate the tuning status.
4. **Automatic Tuning**:
   - Tap the "Start" button to enable automatic tuning.
   - The motor adjusts the string tension (clockwise to sharpen, counterclockwise to flatten) in 300 ms pulses.
   - Tap "Stop" to halt tuning.
5. **Return to Note Selection**:
   - Tap the "Back" button to return to the note selection screen.

## Code Structure
- **main.c**: Core application logic, including:
  - ADC DMA double-buffering for continuous sampling.
  - FFT processing with Hann window and high-pass filter.
  - Frequency detection with harmonic/subharmonic corrections.
  - Touch screen handling and LCD updates.
  - Motor control with PWM.
- **Peripheral Initialization**:
  - `MX_ADC1_Init`: Configures ADC1 with TIM3 trigger.
  - `MX_TIM3_Init`: Sets up 4098.36 Hz sampling rate.
  - `MX_TIM4_Init`: Configures 1 kHz PWM for motor control.
  - `MX_GPIO_Init`: Sets up LEDs and motor control pins.
- **Key Functions**:
  - `apply_hann_window`: Reduces spectral leakage.
  - `high_pass_filter`: Removes low-frequency noise.
  - `find_dominant_freq`: Detects dominant frequency with corrections.
  - `map_to_note`: Determines tuning status.
  - `motor_control`: Drives the L298N motor.
  - `draw_buttons` and `draw_data_screen`: Manage LCD display.
  - `check_touch`: Handles touch input.

## Limitations
- **Frequency Range**: Limited to 50–400 Hz, covering standard guitar tuning (E2 to E4). Notes outside this range (e.g., C#2) require code modifications.
- **Motor Control**: Assumes a single motor direction per string. Complex guitars may need additional logic.
- **Touch Calibration**: Requires manual calibration on startup. Persistent calibration data is not stored.
- **Processing Speed**: The 2048-point FFT may cause slight delays on the 72 MHz STM32F103VET6.

## Extending the Project
To support additional notes (e.g., C#3 at 138.59 Hz):
1. Add to `note_freqs`, `note_names`, `note_tolerances`, and `buttons` arrays.
2. Adjust `num_buttons` and button positions in `draw_buttons`.
3. Update `find_dominant_freq` to handle new note tolerances and harmonics.

For general note detection:
1. Implement a 12-tone note table in `map_to_note`.
2. Modify `find_dominant_freq` to snap to the closest note in the chromatic scale.

## Contributing
Contributions are welcome! Please:
1. Fork the repository.
2. Create a feature branch (`git checkout -b feature/your-feature`).
3. Commit changes (`git commit -m 'Add your feature'`).
4. Push to the branch (`git push origin feature/your-feature`).
5. Open a pull request.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments
- STMicroelectronics for STM32 HAL libraries.
- ARM for CMSIS-DSP FFT implementation.
- Contributors to open-source LCD and XPT2046 drivers.