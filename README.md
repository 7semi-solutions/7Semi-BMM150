# 7Semi BMM150 Arduino Library

Arduino driver for the Bosch BMM150 3-axis geomagnetic sensor.

The BMM150 provides accurate magnetic field measurements along X, Y, and Z axes with low power consumption and supports both I²C and SPI interfaces, making it ideal for compass, navigation, and motion sensing applications.

---

## Features

- Full BMM150 register control
- 3-axis magnetic field measurement (µT)
- I²C and SPI support
- Configurable output data rate (ODR)
- Multiple operation modes (Normal, Forced, Sleep)
- Interrupt and threshold support
- Built-in compensation (Calibration)

---

## Connections

### PS pin  communication protocal select

- LOW  -> SPI
- HIGH -> I2C

### I2C

| BMM150 Pin | MCU Pin | Notes         |
| ---------- | ------- | ------------- |
| VCC        | 3.3V    | **3.3V only** |
| GND        | GND     | Common ground |
| SDA        | SDA     | I²C data      |
| SCL        | SCL     | I²C clock     |

**I2C Address:**

- Default I²C address: '0x13' (SDO = LOW)
- Supported bus speeds:
  - 100 kHz
  - 400 kHz (recommended)

---

### SPI

| BMM150 Pin | MCU Pin | Notes         |
| ---------- | ------- | ------------- |
| VCC        | 3.3V    | **3.3V only** |
| GND        | GND     | Common ground |
| SCK        | SCK     | SPI clock     |
| MISO       | MISO    | SPI data out  |
| MOSI       | MOSI    | SPI data in   |
| CS         | GPIO    | Chip select   |
| PS         | GND     | SPI select    |

- SPI Mode: 0 
- Recommended speed: 1 MHz

---

## Installation

### Arduino Library Manager

  1. Open Arduino IDE
  2. Go to Library Manager  
  3. Search for 7Semi BMM150
  4. Click Install

### Manual Installation

  1. Download this repository as ZIP
  2. Arduino IDE → Sketch → Include Library → Add .ZIP Library

---

## Library Overview

### Reading Magnetic Field

```cpp
float x, y, z;

sensor.readMag(x, y, z);
```

- Returns magnetic field in µT

### Combined Reading

```cpp
float x, y, z;

sensor.readMag(x, y, z);
```

- Reads all three axes (X, Y, Z)

### Operation Modes

```cpp
sensor.setOpMode(BMM150_MODE_NORMAL);
```

- NORMAL → Continuous measurement
- FORCED → One-shot measurement
- SLEEP → Low power mode

### Output Data Rate

```cpp
sensor.setODR(BMM150_ODR_10HZ);
```

- Controls measurement frequency

### Repetition Settings

```cpp
sensor.setRepetitions(9, 15);
```

- Adjusts accuracy vs speed
- Higher value → better accuracy, slower response

### Interrupt Configuration

```cpp
sensor.setHighThreshold(50);
sensor.enableInterrupt(BMM150_INT_HIGH_Z);
```

- Enables threshold-based interrupt

### Reading Interrupt Status

```cpp
uint8_t status;

sensor.getInterruptStatus(status);
```

- Returns interrupt flags

### Self Test

```cpp
float result;

sensor.performSelfTest(result);
```

- Adjusts accuracy vs speed
- Higher value → better accuracy, slower response

### Soft Reset

```cpp
sensor.softReset();
```

- Resets sensor to default state

---
