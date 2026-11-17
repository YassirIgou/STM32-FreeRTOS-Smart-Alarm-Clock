# Smart Alarm Clock on STM32 with FreeRTOS

An embedded smart alarm clock developed in **C** on an **STM32L476RG**, using **FreeRTOS** to coordinate timekeeping, environmental monitoring, Bluetooth configuration, lighting effects, audio playback, and gesture-based alarm control.

The application was developed with **STM32CubeIDE** for the **CIBLE** development board.

## 1. Project Overview

The objective is to design a multitasking embedded system that combines a configurable alarm clock with sensor acquisition and actuator control. FreeRTOS tasks separate the application functions, while interrupts, binary semaphores, and a mutex coordinate asynchronous events and shared hardware access.

### Main features

- **Clock and alarm:** software timekeeping, configurable alarm, alarm enable/disable, stop, and snooze.
- **Wake-up lighting:** progressive RGB LED lighting to simulate sunrise.
- **Sleep mode:** gradual LED dimming to simulate sunset, accompanied by a soft melody.
- **Environmental monitoring:** temperature, VOC (air-quality indicator), and ambient light measurements.
- **Ventilation:** automatic fan control based on temperature or VOC thresholds, plus manual speed control via potentiometer.
- **OLED interface:** display of current time, alarm settings, temperature, and VOC readings.
- **Bluetooth control:** UART text commands for configuring the clock, alarm, and thresholds.
- **Custom ringtones:** loading up to three ringtone files from a microSD card through FatFs.
- **Touch controls:** capacitive touch buttons for ringtone preview and selection.
- **Gesture recognition:** two VL53L1 time-of-flight sensors for stopping or snoozing the alarm.

## 2. Hardware Platform

The project uses the **CIBLE development board** with an **STM32L476RG** microcontroller and the following peripherals:

| Component | Purpose | Interface |
| --- | --- | --- |
| SSD1306 OLED display | Time, alarm, temperature, and VOC display | I2C |
| SGP40 sensor | VOC measurement | I2C |
| Two VL53L1 ToF sensors | Hand-gesture detection | I2C + GPIO interrupts |
| Temperature sensor | Temperature measurement | SPI |
| LDR | Ambient light measurement | ADC |
| Potentiometer | Manual fan-speed setting | ADC |
| Fan / motor | Environmental response | PWM + GPIO |
| RGB LEDs | Sunrise and sunset effects | Timer PWM + DMA |
| Buzzer | Alarm tones and melodies | Timer PWM |
| Capacitive touch inputs | Ringtone preview and selection | TSC |
| microSD card | Custom ringtone storage | SDMMC + FatFs |
| Bluetooth serial module | Remote configuration | LPUART1 |
| Push buttons | User interaction | GPIO / EXTI |

## 3. Repository Structure

```text
freertos/
├── REVEIL/             # STM32CubeIDE firmware project
│
├── SD_FILES_SONG/
│   ├── SD_READ.c       # STM32CubeMX-generated SD driver requiring manual fixes
│   ├── SON1.txt        # Custom ringtone 1
│   ├── SON2.txt        # Custom ringtone 2
│   └── SON3.txt        # Custom ringtone 3
│
├── images/
│
└── README.md
```

## 4. FreeRTOS Software Architecture

The firmware creates **11 application tasks**, in addition to the default CMSIS-RTOS task.

| Task | Responsibility | Relative priority |
| --- | --- | --- |
| `TaskHorloge` | Update the software clock once per second | High (+3) |
| `TaskAlarme` | Trigger, stop, and snooze the alarm | High (+3) |
| `TaskAudio` | Generate tones, melodies, and ringtone previews | High (+3) |
| `TaskBluetooth` | Parse incoming Bluetooth commands | Medium (+2) |
| `TaskVentilo` | Control fan speed and direction | Medium (+2) |
| `TaskTOF` | Detect hand gestures from two distance sensors | Medium (+2) |
| `TaskInteraction` | Handle buttons and capacitive touch inputs | Medium (+2) |
| `TaskAffichage` | Update the OLED screen | Low (+1) |
| `TaskCapteurs` | Acquire temperature, VOC, LDR, and potentiometer data | Low (+1) |
| `TaskLED` | Control RGB lighting effects | Low (+1) |
| `TaskSD` | Detect microSD insertion/removal and load ringtones | Low (+1) |

### Task synchronization

- **I2C mutex (`mutexI2C`):** protects the shared bus during OLED, VOC, and ToF operations.
- **Bluetooth binary semaphore (`semaphBT`):** signals the Bluetooth task when a UART receive interrupt occurs.
- **Button binary semaphore (`semaphSW1`):** signals the interaction task from a GPIO interrupt.
- **Interrupt flags:** signal when ToF measurements are ready.
- **Periodic task delays:** define update rates without using a single blocking main loop.

## 5. Alarm and User Interaction

### Alarm behavior

1. The user sets the time and alarm using Bluetooth commands.
2. When the configured alarm time is reached, the alarm task activates the buzzer and the sunrise lighting sequence.
3. The alarm can be stopped or temporarily postponed using hand gestures detected by the two ToF sensors.
4. The user can also initiate a sunset lighting effect and soft melody using a push button.

### Gesture commands

| Gesture order | Action |
| --- | --- |
| Left sensor, then right sensor | Stop the alarm |
| Right sensor, then left sensor | Snooze the alarm |

### Ringtone selection

Three capacitive touch inputs are used to select among three ringtones:

- **Short touch:** preview a ringtone.
- **Long touch:** select a ringtone for the alarm.

If a ringtone cannot be loaded from the microSD card, the firmware has a built-in fallback melody.

## 6. Environmental Monitoring and Fan Control

The sensor task reads the potentiometer and LDR approximately every **100 ms**, and temperature and VOC approximately every **1 s**.

Fan-control behavior follows this priority:

1. **Manual mode:** when the potentiometer exceeds its threshold, its value controls fan speed.
2. **High temperature:** fan speed increases according to the difference between the measured temperature and the configured threshold.
3. **High VOC:** the fan runs and periodically changes direction.
4. **Normal conditions:** the fan is stopped.

The OLED display also adjusts its rendering color setting based on the LDR threshold.

> **Sensor note:** the current SGP40 measurement call uses the measured temperature and a fixed relative-humidity value of **50%**.

## 7. Bluetooth Command Interface

The Bluetooth module communicates over **LPUART1 at 9600 baud**. Commands are text-based and terminated by a newline (`\n`) or carriage return (`\r`).

| Command | Description |
| --- | --- |
| `heure : 08:30:00` | Set the current time |
| `alarme : 07:00:00` | Set and enable the alarm |
| `alarme on` | Enable the configured alarm |
| `alarme off` | Disable the alarm |
| `temperature : 30` | Set the temperature threshold to 30 °C |
| `voc : 500` | Set the VOC threshold |
| `ldr : 1500` | Set the ambient-light threshold |
| `status` | Display the current time, alarm, and thresholds |

Example session:

```text
heure : 08:30:00
OK heure : 08:30:00

alarme : 08:31:00
OK alarme : 08:31:00

status
Heure : 08:30:00
Alarme : 08:31:00 [ON]
Temperature seuil : 30 C
VOC seuil : 500
LDR seuil : 1500
```

The firmware accepts hours from 0 to 23, minutes/seconds from 0 to 59, temperature thresholds from -40 to 125 °C, VOC thresholds from 0 to 65535, and LDR thresholds from 0 to 4095.

## 8. microSD Ringtone Files

The `SD_FILES_SONG/` directory contains three sample ringtone files:

```text
SON1.txt
SON2.txt
SON3.txt
```

Each valid line describes one note using the following CSV format:

```text
frequency_hz,duration_ms,pause_ms
```

## Author

**Mohamed Yassir IGOUZOULENE**

Developed as part of an embedded systems project.