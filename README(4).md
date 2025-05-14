# ESP32 Labs

This repository contains a collection of embedded systems labs implemented on the ESP32 platform. Each lab focuses on a different aspect of low-level system design and peripheral interaction, ranging from analog data acquisition to real-time motor control. All code is written in Arduino C/C++ and was developed on ESP32 boards using real hardware peripherals.

---

## Included Labs

### `lab_adc_monitor/`
Reads analog voltage from the ESP32's ADC and calibrates the readings using a piecewise linear model. Interfaces with a bench power supply via UART and converts readings back to real-world voltages for battery monitoring.

**Key Features:**
- 12-bit ADC sampling
- Serial communication with lab power supply
- Voltage divider compensation
- Real-time voltage display

---

### `lab_i2c_mpu6050/`
Implements I2C communication with the MPU-6050 motion sensor to extract acceleration data. Converts raw readings to orientation angles using trigonometric calculations.

**Key Features:**
- I2C register access to MPU-6050
- Raw accelerometer data acquisition
- Angle calculation via `atan2` in 3D planes
- 1 Hz serial output loop

---

### `lab_pwm_audio_led/`
Generates a PWM-based audio tone while using the amplitude to visualize sound with WS2812 RGB LEDs. Combines audio signal generation and LED strip animation.

**Key Features:**
- PWM sine wave generation for speaker output
- Real-time amplitude sampling
- Color mapping to WS2812 LED strip
- Timer and RMT peripherals for performance

---

### `lab_motor_control/`
Controls a DC motor’s speed using feedback from a pulse counter (PCNT) and adjusts motor power via PID controller. Runs in FreeRTOS tasks for real-time responsiveness.

**Key Features:**
- Motor encoder input via PCNT
- FreeRTOS-based task scheduling
- PID control loop for RPM regulation
- Serial reporting of speed and PWM duty

---

## License

All code in this repository is original and written by Joshua Moorehead. No university-distributed boilerplate or copyrighted instructional materials are included.

Feel free to explore, fork, and use these labs as a reference for your own ESP32 embedded projects.