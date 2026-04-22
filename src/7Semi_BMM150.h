/**
 * 7Semi BMM150 Library
 *
 * - High-performance driver for Bosch BMM150 magnetometer
 * - Supports both I2C and SPI communication
 *
 * Features:
 * - Full sensor configuration support
 * - Built-in compensation algorithms
 * - Interrupt and threshold handling
 *
 * Notes:
 * - Calibration data is read internally
 * - Output values are compensated
 *
 * License:
 * - MIT License (7Semi)
 */

#pragma once
#include <Arduino.h>
#include "7Semi_Interface.h"
#include "7Semi_I2C_Interface.h"
#include "7Semi_SPI_Interface.h"
#include "BusIO_7Semi.h"

#define BMM150_REG_CHIP_ID          0x40
#define BMM150_REG_MAG_DATA         0x42
#define BMM150_REG_HALL_RESISTANCE  0x48
#define BMM150_REG_INTERRUPT_STATUS 0x4A

#define BMM150_REG_POWER_CONFIG     0x4B
#define BMM150_REG_CONTROL          0x4C

#define BMM150_REG_INT_CONFIG_0     0x4D
#define BMM150_REG_INT_CONFIG_1     0x4E

#define BMM150_REG_LOW_THRESHOLD    0x4F
#define BMM150_REG_HIGH_THRESHOLD   0x50

#define BMM150_REG_REP_XY           0x51
#define BMM150_REG_REP_Z            0x52

#define BMM150_CHIP_ID              0x32
#define BMM150_RESET_CONFIG         0x82

/**
 * Interrupt Status Bits 
 *
 * - Bit 0 → X low threshold triggered
 * - Bit 1 → Y low threshold triggered
 * - Bit 2 → Z low threshold triggered
 * - Bit 3 → X high threshold triggered
 * - Bit 4 → Y high threshold triggered
 * - Bit 5 → Z high threshold triggered
 * - Bit 6 → Overflow detected
 * - Bit 7 → Data overrun detected
 */
#define BMM150_INT_DATA_OVERRUN     (1 << 7)
#define BMM150_INT_STATUS_OVERFLOW  (1 << 6)

#define BMM150_INT_HIGH_Z           (1 << 5)
#define BMM150_INT_HIGH_Y           (1 << 4)
#define BMM150_INT_HIGH_X           (1 << 3)

#define BMM150_INT_LOW_Z            (1 << 2)
#define BMM150_INT_LOW_Y            (1 << 1)
#define BMM150_INT_LOW_X            (1 << 0)

/**
 * Interrupt Enable Bits 
 *
 * - Bit 0 → Enable X low threshold interrupt
 * - Bit 1 → Enable Y low threshold interrupt
 * - Bit 2 → Enable Z low threshold interrupt
 * - Bit 3 → Enable X high threshold interrupt
 * - Bit 4 → Enable Y high threshold interrupt
 * - Bit 5 → Enable Z high threshold interrupt
 * - Bit 6 → Enable overflow interrupt
 * - Bit 7 → Enable data overrun interrupt
 */
#define BMM150_INT_DATA_OVERRUN_EN  (1 << 7)
#define BMM150_INT_OVERFLOW_EN      (1 << 6)

#define BMM150_INT_HIGH_Z_EN        (1 << 5)
#define BMM150_INT_HIGH_Y_EN        (1 << 4)
#define BMM150_INT_HIGH_X_EN        (1 << 3)

#define BMM150_INT_LOW_Z_EN         (1 << 2)
#define BMM150_INT_LOW_Y_EN         (1 << 1)
#define BMM150_INT_LOW_X_EN         (1 << 0)

#define BMM150_INT_ENABLE_ALL       (0xFF)

/**
 * Output Data Rate (ODR)
 *
 * - Defines measurement frequency
 * - Higher ODR → faster updates
 * - Lower ODR → lower power consumption
 */
typedef enum BMM150_odr
{
    BMM150_ODR_10HZ =   0x00, 
    BMM150_ODR_2HZ  =   0x01,  
    BMM150_ODR_6HZ  =   0x02, 
    BMM150_ODR_8HZ  =   0x03,  
    BMM150_ODR_15HZ =   0x04, 
    BMM150_ODR_20HZ =   0x05, 
    BMM150_ODR_25HZ =   0x06, 
    BMM150_ODR_30HZ =   0x07  
};

/**
 * Operation Modes
 *
 * - NORMAL  → Continuous measurement
 * - FORCED  → Single measurement 
 * - SLEEP   → Low power mode
 */
typedef enum bmm150_op_mode_t
{
    BMM150_MODE_NORMAL      = 0x00,   
    BMM150_MODE_FORCED      = 0x01,   
    BMM150_MODE_RESERVED    = 0x02, 
    BMM150_MODE_SLEEP       = 0x03     
};

/**
 * Interrupt Pin Selection
 *
 * - DATA_READY_PIN → Data ready output 
 * - INTERRUPT_PIN  → Threshold / event interrupt
 */
typedef enum bmm150_select_pin
{
    BMM150_DATA_READY_PIN = 0x00, 
    BMM150_INTERRUPT_PIN  = 0x01
};

/**
 * Advanced Self-Test Modes
 *
 * - NORMAL → Standard operation
 * - NEG    → Apply negative magnetic field
 * - POS    → Apply positive magnetic field
 */
typedef enum bmm150_adv_selftest_t
{
    BMM150_ADV_ST_NORMAL = 0x00, 
    BMM150_ADV_ST_NEG    = 0x02,    
    BMM150_ADV_ST_POS    = 0x03     
};

class BMM150_7Semi
{

public:
    BMM150_7Semi();

    /**
     * beginI2C()
     *
     * - Initializes BMM150 over I2C
     * - Configures communication interface
     * - Performs full sensor startup sequence
     *
     * Sequence:
     * - Reset sensor
     * - Wake device
     * - Verify chip ID
     * - Read calibration (calib) registers
     * - Set operation mode
     * - Enable XYZ axes
     *
     * Notes:
     * - Sensor must be powered before calling
     * - Uses blocking delays for stability
     */
    bool beginI2C(uint8_t addr, TwoWire &wire, uint32_t speed);

    /**
     * beginSPI()
     *
     * - Initializes BMM150 over SPI
     * - Similar sequence to I2C initialization
     *
     * Sequence:
     * - Reset sensor
     * - Verify chip ID
     * - Wake device
     * - Read calibration registers
     * - Set operation mode
     * - Enable XYZ axes
     *
     * Notes:
     * - Chip ID check ensures communication is valid
     */
    bool BMM150_7Semi::beginSPI(uint8_t csPin, SPIClass &spiPort,
                                uint32_t spiClock);

    /**
     * getChipID()
     *
     * - Reads device chip ID register
     * - Used to verify correct sensor connection
     *
     * Output:
     * - id → Device ID value
     *
     * Notes:
     * - Expected value: 0x32
     * - Debug print included (can be disabled in production)
     */
    bool getChipID(uint8_t &id);

    /**
     * softReset()
     *
     * - Performs sensor reset
     * - Clears internal state and registers
     *
     * Notes:
     * - Requires delay after reset
     * - Sensor enters sleep after reset
     */
    bool softReset();

    /**
     * setSleepMode()
     *
     * - Enables or disables sleep mode
     *
     * Input:
     * - enable = true  → sleep mode
     * - enable = false → normal mode
     */
    bool setSleepMode(bool enable);

    
    /**
     * getSleepMode()
     *
     * - Reads current sleep mode state
     *
     * Output:
     * - enable → true if sensor in sleep
     */
    bool getSleepMode(bool &enable);

    /**
     * readRaw()
     *
     * - Reads raw magnetometer data 
     * - Extracts X, Y, Z axis values and hall resistance
     *
     * Output:
     * - x, y, z  → Raw magnetic values 
     * - rhall    → Hall resistance 
     */
    bool readRaw(int16_t &x, int16_t &y, int16_t &z, uint16_t &rhall);

    /**
     * readMag()
     *
     * - Reads compensated magnetic field values
     *
     * Output:
     * - x, y, z → Magnetic field values (microTesla)
     */
    bool readMag(float &x, float &y, float &z);

    /**
     * setODR()
     *
     * - Sets Output Data Rate (ODR) of sensor
     * - Affects measurement frequency
     *
     * Input:
     * - odr → One of BMM150_ODR_xx values
     */
    bool setODR(BMM150_odr odr);

    /**
     * getODR()
     *
     * - Reads current Output Data Rate
     *
     * Output:
     * - odr → Current ODR setting
     */
    bool getODR(uint8_t &odr);

    /**
     * setOpMode()
     *
     * - Sets sensor operation mode
     *
     * Input:
     * - opMode → NORMAL / FORCED / SLEEP
     */
    bool setOpMode(bmm150_op_mode_t opMode);

    /**
     * getOpMode()
     *
     * - Reads current operation mode
     *
     * Output:
     * - opMode → Current mode value
     */
    bool getOpMode(uint8_t &opMode);

    /**
     * enableInterrupt()
     *
     * - Enables interrupt sources using bitmask
     * - intMask → Combination of BMM150_INT_* flags
     *
     * Input:
     * Bit Mapping:
     * - Bit 0 → X-axis low threshold interrupt
     * - Bit 1 → Y-axis low threshold interrupt
     * - Bit 2 → Z-axis low threshold interrupt
     * - Bit 3 → X-axis high threshold interrupt
     * - Bit 4 → Y-axis high threshold interrupt
     * - Bit 5 → Z-axis high threshold interrupt
     * - Bit 6 → Overflow interrupt
     * - Bit 7 → Data overrun interrupt
     *
     * Notes:
     * - Written directly to INT_CONFIG_0 register
     */
    bool enableInterrupt(uint8_t intMask);

    /**
     * getEnabledInterrupt()
     *
     * - Reads enabled interrupt mask
     *
     * Output:
     * - intMask → Current interrupt configuration
     */
    bool getEnabledInterrupt(uint8_t &intMask);

    /**
     * setPins()
     *
     * - Configures interrupt or data-ready pin behavior
     *
     * Input:
     * - selectPin → DRDY or INT pin
     * - enablePin → Enable/disable pin
     * - polarity  → Active high / low
     * - latched   → Latched or non-latched mode
     */
    bool setPins(
        bmm150_select_pin selectPin,
        bool enablePin,
        bool polarity,
        bool latched);

    /**
     * getPins()
     *
     * - Reads pin configuration
     *
     * Output:
     * - selectPin → DRDY or INT pin
     * - enablePin → Pin enabled state
     * - polarity  → Active level
     * - latched   → Latched mode
     */
    bool BMM150_7Semi::getPins(
        bmm150_select_pin selectPin,
        bool &enablePin,
        bool &polarity,
        bool &latched);

    /**
     * setAxisEnable()
     *
     * - Enables or disables X, Y, Z axes
     *
     * Input:
     * - x, y, z → true = enable axis
     */
    bool setAxisEnable(bool x, bool y, bool z);

    /**
     * getAxisEnable()
     *
     * - Reads axis enable status
     *
     * Output:
     * - x, y, z → true if axis enabled
     */
    bool getAxisEnable(bool &x, bool &y, bool &z);

    /**
     * setLowThreshold()
     *
     * - Sets low threshold for interrupt detection
     * - Used for low-field interrupt triggering
     *
     * Input:
     * - threshold → Threshold value (0–255) 
     */
    bool setLowThreshold(uint8_t threshold);

    /**
     * getLowThreshold()
     *
     * - Reads low threshold value
     *
     * Output:
     * - threshold → Current low threshold
     */
    bool getLowThreshold(uint8_t &threshold);

    /**
     * setHighThreshold()
     *
     * - Sets high threshold for interrupt detection
     * - Used for high-field interrupt triggering
     *
     * Input:
     * - threshold → Threshold value (0–255)
     */
    bool setHighThreshold(uint8_t threshold);

    /**
     * getHighThreshold()
     *
     * - Reads high threshold value
     *
     * Output:
     * - threshold → Current high threshold
     */
    bool getHighThreshold(uint8_t &threshold);

    /**
     * setRepetitions()
     *
     * - Configures measurement repetitions
     * - Higher repetition → better accuracy, slower update
     *
     * Input:
     * - xy → XY repetition count (1–511)
     * - z  → Z repetition count (1–256)
     */
    bool setRepetitions(uint16_t xy, uint16_t z);

    /**
     * getRepetitions()
     *
     * - Reads configured repetition values
     *
     * Output:
     * - xy → XY repetition count
     * - z  → Z repetition count
     */
    bool getRepetitions(uint16_t &xy, uint16_t &z);

   /**
     * enableDataReadyInterrupt()
     *
     * - Enables data-ready interrupt on interrupt pin
     */
    bool enableDataReadyInterrupt();

    /**
     * getInterruptStatus()
     *
     * - Reads interrupt status register
     *
     * Output:
     * - status → Bitmask of interrupt flags
     *
     * Bit Mapping:
     * - Bit 0 → X-axis low threshold trigger
     * - Bit 1 → Y-axis low threshold trigger
     * - Bit 2 → Z-axis low threshold trigger
     * - Bit 3 → X-axis high threshold trigger
     * - Bit 4 → Y-axis high threshold trigger
     * - Bit 5 → Z-axis high threshold trigger
     * - Bit 6 → Overflow detected
     * - Bit 7 → Data overrun detected
     *
     * Notes:
     * - Multiple bits can be set simultaneously
     * - Read operation clear flags automatically
     */
    bool getInterruptStatus(uint8_t &status);

    /**
     * setAdvSelfTest()
     *
     * - Configures advanced self-test mode
     *
     * Input:
     * - selfTest → BMM150_ADV_ST_NORMAL / BMM150_ADV_ST_NEG / BMM150_ADV_ST_POS
     */
    bool setAdvSelfTest(bmm150_adv_selftest_t selfTest);

    /**
     * performSelfTest()
     *
     * - Executes advanced self-test sequence
     *
     * Output:
     * - selfTestResult → Difference between positive and negative field
     */
    bool performSelfTest(float &selfTestResult);

private:
    // Communication interface
    I2C_Interface i2c;
    SPI_Interface spi;

    // Abstract interface pointer
    Interface_7Semi *iface = nullptr;

    // BusIO wrapper 
    BusIO_7Semi<Interface_7Semi> *bus = nullptr;

    /** calibration data */
    struct calibrartionData
    {
        int8_t x_offset1;
        int8_t y_offset1;
        int8_t x_offset2;
        int8_t y_offset2;
        uint16_t z_sensitivity1;
        int16_t z_sensitivity2;
        int16_t z_sensitivity3;
        int16_t z_offset;
        uint8_t xy_sensitivity1;
        int8_t xy_sensitivity2;
        uint16_t hall_resistance;
    };

    calibrartionData calib;

    /**
     * readCalibrationData()
     *
     * - Reads factory calibration registers 
     * - Stores calibration parameters for compensation
     *
     * Notes:
     * - Must be called before using readMag()
     * - Values are unique per sensor
     */
    bool readCalibrartionData();

    /** Compensates raw X,Y, and Z axis value */
    float compensateX(int16_t x, uint16_t rhall);
    float compensateY(int16_t y, uint16_t rhall);
    float compensateZ(int16_t z, uint16_t rhall);
};