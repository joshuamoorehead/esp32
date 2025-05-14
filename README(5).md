# ESP32 Embedded Projects

This repository contains a collection of embedded systems projects implemented on the ESP32 platform. Each project explores a different area of low-level embedded development, including analog sensing, I2C peripheral interfacing, PWM-based audio output, LED visualization, and motor control using real-time scheduling.

All code is written in Arduino C/C++ and was tested on real ESP32 hardware.

---

## Included Projects

### `adc_voltage_reader/`
Reads analog voltage using the ESP32's ADC and calibrates the readings using a piecewise linear model. Designed for battery monitoring via UART feedback.

**Key Features:**
- 12-bit ADC sampling and real-world voltage conversion
- Serial I/O with power supply monitoring
- Real-time voltage display loop

---

### `mpu6050_tilt_sensor/`
Communicates with the MPU-6050 IMU via I2C to read acceleration data and compute orientation angles using `atan2`.

**Key Features:**
- I2C communication from scratch
- 3D tilt angle calculation from accelerometer data
- Outputs angles in XY, XZ, YZ planes at 1 Hz

---

### `audio_led_visualizer/`
Generates PWM sine wave audio output while driving WS2812 LED strips to visualize amplitude. Combines sound generation and LED control with efficient timing.

**Key Features:**
- Real-time PWM audio tone generation
- Amplitude detection and LED mapping
- Uses RMT and timer peripherals on ESP32

---

### `motor_pid_controller/`
Controls a DC motor’s speed using encoder feedback (PCNT) and a PID controller. Runs in parallel FreeRTOS tasks for speed measurement and motor output.

**Key Features:**
- RPM measurement via pulse counter
- PID loop for target speed tracking
- FreeRTOS-based task design

---

## License

All code in this repository is original and written by Joshua Moorehead.

You may reuse or adapt the code for educational or personal use. No course-distributed templates or university-provided materials are included.