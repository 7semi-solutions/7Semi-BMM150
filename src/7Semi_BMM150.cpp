#include "7Semi_BMM150.h"

BMM150_7Semi::BMM150_7Semi()
{
}

bool BMM150_7Semi::beginI2C(uint8_t addr, TwoWire &i2cPort, uint32_t i2cClock)
{
    // Delete existing bus
    if (bus)
    {
        delete bus;
        bus = nullptr;
    }

    iface = &i2c;

    // Initialize I2C interface
    if (!i2c.beginI2C(addr, i2cPort, i2cClock))
        return false;

    // Create BusIO wrapper
    bus = new BusIO_7Semi<Interface_7Semi>(*iface);
    if (!bus)
        return false;

    // Soft reset
    if (!softReset())
        return false;

    // Wake sensor
    if (!setSleepMode(false))
        return false;

    delay(100);

    // Verify chip ID to check correct sensor is connected
    uint8_t chip_id = 0;
    if (!getChipID(chip_id))
        return false;

    if (chip_id != BMM150_CHIP_ID)
        return false;

    /**
     * Read calibration data
     *
     * - Required for compensation formulas
     * - Must be read before measurements
     */
    if (!readCalibrartionData())
        return false;

    delay(5);

    // Set operation mode
    if (!setOpMode(BMM150_MODE_NORMAL))
        return false;

    // Enable all axes
    if (!setAxisEnable(true, true, true))
        return false;

    return true;
}


bool BMM150_7Semi::beginSPI(uint8_t csPin, SPIClass &spiPort,
                            uint32_t spiClock)
{
    // Cleanup existing bus
    if (bus)
    {
        delete bus;
        bus = nullptr;
    }

    iface = &spi;

    // Initialize SPI interface
    if (!spi.beginSPI(csPin, spiPort, spiClock))
        return false;

    // Create BusIO wrapper
    bus = new BusIO_7Semi<Interface_7Semi>(*iface);
    if (!bus)
        return false;

    // Soft reset
    if (!softReset())
        return false;

    if (!setSleepMode(false))
      return false;

    delay(100);

    // Verify chip ID to check correct sensor is connected
    uint8_t chip_id = 0;
    if (!getChipID(chip_id))
        return false;

        delay(10);

    if (chip_id != BMM150_CHIP_ID)
        return false;

    delay(100);

    // Read calibration data
    if (!readCalibrartionData())
        return false;

    delay(5);

    // Set operation mode
    if (!setOpMode(BMM150_MODE_NORMAL))
        return false;

    // Enable all axes
    if (!setAxisEnable(true, true, true))
        return false;

    return true;
}

bool BMM150_7Semi::getChipID(uint8_t &id)
{
    if (!bus->read(BMM150_REG_CHIP_ID, id))
        return false;
        
    return true;
}

bool BMM150_7Semi::softReset()
{
    if (!bus->write(BMM150_REG_POWER_CONFIG, (uint8_t)BMM150_RESET_CONFIG))
        return false;

    delay(100);


    return bus->writeBit(BMM150_REG_POWER_CONFIG, 0, (uint8_t)0x00);
}

bool BMM150_7Semi::setSleepMode(bool enable)
{
    uint8_t enable_sleep = !enable;
    return bus->writeBit(BMM150_REG_POWER_CONFIG, 0, enable_sleep);
}

bool BMM150_7Semi::getSleepMode(bool &enable)
{
    uint8_t v = 0;

    if (!bus->readBit(BMM150_REG_POWER_CONFIG, 0, v))
        return false;

    enable = (v != 0);

    return true;
}

bool BMM150_7Semi::readRaw(int16_t &x, int16_t &y, int16_t &z, uint16_t &rhall)
{
    uint8_t data[8];

    if (!bus->read(BMM150_REG_MAG_DATA, data, 8))
        return false;

    /**
     * Extract raw axis data
     *
     * - X/Y: 13-bit values
     * - Z  : 15-bit value
     * - RH : 14-bit value
     */
    x = (int16_t)((((int16_t)data[1]) << 8) | data[0]) >> 3;
    y = (int16_t)((((int16_t)data[3]) << 8) | data[2]) >> 3;
    z = (int16_t)((((int16_t)data[5]) << 8) | data[4]) >> 1;
    rhall = (uint16_t)((((uint16_t)data[7]) << 8) | data[6]) >> 2;

    return true;
}

bool BMM150_7Semi::readMag(float &x, float &y, float &z)
{
    int16_t rx, ry, rz;
    uint16_t rh;

    if (!readRaw(rx, ry, rz, rh))
        return false;

    // Apply compensation
    x = compensateX(rx, rh);
    y = compensateY(ry, rh);
    z = compensateZ(rz, rh);

    return true;
}

float BMM150_7Semi::compensateX(int16_t x, uint16_t rh)
{
    // Overflow check
    if (x == -4096)
        return 0;

    // Validate hall resistance data
    float rhall = (rh != 0) ? (float)rh : (float)calib.hall_resistance;

    // Validate callibration data
    if (rhall == 0 || calib.hall_resistance == 0)
        return 0;

    // Compensation calculation
    float val = ((float)calib.hall_resistance * 16384.0f / rhall) - 16384.0f;
    float val_sq = val * val;

    float comp = ((float)calib.xy_sensitivity2 * (val_sq / 128.0f)) +
                 ((float)calib.xy_sensitivity1 * val * 128.0f);

    comp = (comp / 512.0f) + 1048576.0f;
    comp = (comp * ((float)calib.x_offset2 + 160.0f)) / 4096.0f;

    float res = ((float)x * comp) / 8192.0f;
    res = (res + ((float)calib.x_offset1 * 8.0f)) / 16.0f;

    return res;
}

float BMM150_7Semi::compensateY(int16_t y, uint16_t rh)
{
     // Overflow check
    if (y == -4096)
        return 0;

    // Validate hall resistance data
    float rhall = (rh != 0) ? (float)rh : (float)calib.hall_resistance;

    if (rhall == 0 || calib.hall_resistance == 0)
        return 0;

    float val = ((float)calib.hall_resistance * 16384.0f / rhall) - 16384.0f;
    float val_sq = val * val;

    float comp = ((float)calib.xy_sensitivity2 * (val_sq / 128.0f)) +
                 ((float)calib.xy_sensitivity1 * val * 128.0f);

    comp = (comp / 512.0f) + 1048576.0f;
    comp = (comp * ((float)calib.y_offset2 + 160.0f)) / 4096.0f;

    float res = ((float)y * comp) / 8192.0f;
    res = (res + ((float)calib.y_offset1 * 8.0f)) / 16.0f;

    return res;
}


float BMM150_7Semi::compensateZ(int16_t z, uint16_t rh)
{
     // Overflow check
    if (z == -16384)
        return 0;

    // Validate all required parameters
    if (calib.z_sensitivity2 == 0 || calib.z_sensitivity1 == 0 ||
        calib.hall_resistance == 0 || rh == 0)
        return 0;

    float rhall = (float)rh;

    // Compensation calculation
    float temp0 = rhall - (float)calib.hall_resistance;
    float temp1 = ((float)calib.z_sensitivity3 * temp0) / 4.0f;
    float temp2 = ((float)(z - calib.z_offset)) * 32768.0f;
    float temp3 = ((float)calib.z_sensitivity1 * rhall * 2.0f);

    float temp4 = (temp3 + 32768.0f) / 65536.0f;

    float denom = (float)calib.z_sensitivity2 + temp4;
    if (denom == 0)
        return 0;

    float res = (temp2 - temp1) / denom;

    // Clamp output range
    if (res > 32767.0f)
        res = 32767.0f;
    else if (res < -32768.0f)
        res = -32768.0f;

    return res / 16.0f;
}

bool BMM150_7Semi::readCalibrartionData()
{
    uint8_t buf[24];

    // Read XY offsets
    if (!bus->read(0x5D, buf, 2))
        return false;

    calib.x_offset1 = (int8_t)buf[0];
    calib.y_offset1 = (int8_t)buf[1];

    // Read Z offset and XY offset2
    if (!bus->read(0x62, buf, 4))
        return false;

    calib.z_offset = (int16_t)((((uint16_t)buf[1]) << 8) | buf[0]);
    calib.x_offset2 = (int8_t)buf[2];
    calib.y_offset2 = (int8_t)buf[3];

    // Read sensitivity and hall resistance
    if (!bus->read(0x68, buf, 10))
        return false;

    calib.z_sensitivity2 = (int16_t)((((uint16_t)buf[1]) << 8) | buf[0]);
    calib.z_sensitivity1 = (uint16_t)((((uint16_t)buf[3]) << 8) | buf[2]);
    calib.hall_resistance = ((uint16_t)buf[5] << 8 | buf[4]) & 0x7FFF;
    calib.z_sensitivity3 = (int16_t)(((uint16_t)buf[7] << 8) | buf[6]);
    calib.xy_sensitivity2 = (int8_t)buf[8];
    calib.xy_sensitivity1 = buf[9];

    return true;
}

bool BMM150_7Semi::setODR(BMM150_odr odr)
{
    // Validate input range
    if (odr > BMM150_ODR_30HZ)
        return false;

    return bus->writeBits(BMM150_REG_CONTROL, 3, 3, (uint8_t)odr);
}

bool BMM150_7Semi::getODR(uint8_t &odr)
{
    if (!bus->readBits(BMM150_REG_CONTROL, 3, 3, odr))
        return false;

    // Validate returned value
    if (odr > BMM150_ODR_30HZ)
        return false;

    return true;
}

bool BMM150_7Semi::setOpMode(bmm150_op_mode_t opMode)
{
    if (opMode > BMM150_MODE_SLEEP)
        return false;

    return bus->writeBits(BMM150_REG_CONTROL, 1, 2, (uint8_t)opMode);
}

bool BMM150_7Semi::getOpMode(uint8_t &opMode)
{
    if (!bus->readBits(BMM150_REG_CONTROL, 1, 2, opMode))
        return false;

    if (opMode > BMM150_MODE_SLEEP)
        return false;

    return true;
}

bool BMM150_7Semi::enableInterrupt(uint8_t intMask)
{
    return bus->write(BMM150_REG_INT_CONFIG_0, intMask);
}

bool BMM150_7Semi::getEnabledInterrupt(uint8_t &intMask)
{
    if (!bus->read(BMM150_REG_INT_CONFIG_0, intMask))
        return false;

    return true;
}

bool BMM150_7Semi::setPins(
    bmm150_select_pin selectPin,
    bool enablePin,
    bool polarity,
    bool latched)
{
    uint8_t v = 0;

    if (!bus->read(BMM150_REG_INT_CONFIG_1, v))
        return false;

    // Configure Data Ready pin
    if (selectPin == BMM150_DATA_READY_PIN)
    {
        v &= ~(1 << 7); // clear enable
        v &= ~(1 << 2); // clear polarity

        v |= (enablePin << 7);
        v |= (polarity << 2);
    }

    // Configure Interrupt pin
    else if (selectPin == BMM150_INTERRUPT_PIN)
    {
        v &= ~(1 << 6); // clear enable
        v &= ~(1 << 0); // clear polarity

        v |= (enablePin << 6);
        v |= (polarity << 0);
    }

    // Configure latch mode
    v &= ~(1 << 1);
    v |= (latched << 1);

    return bus->write(BMM150_REG_INT_CONFIG_1, v);
}


bool BMM150_7Semi::getPins(
    bmm150_select_pin selectPin,
    bool &enablePin,
    bool &polarity,
    bool &latched)
{
    uint8_t v;

    if (!bus->read(BMM150_REG_INT_CONFIG_1, v))
        return false;

    if (selectPin == BMM150_DATA_READY_PIN)
    {
        enablePin = (v >> 7) & 0x01;
        polarity  = (v >> 2) & 0x01;
    }
    else if (selectPin == BMM150_INTERRUPT_PIN)
    {
        enablePin = (v >> 6) & 0x01;
        polarity  = (v >> 0) & 0x01;
    }
    else
    {
        return false;
    }

    latched = (v >> 1) & 0x01;

    return true;
}

bool BMM150_7Semi::setAxisEnable(bool x, bool y, bool z)
{
    uint8_t reg;

    if (!bus->read(BMM150_REG_INT_CONFIG_1, reg))
        return false;

    // Clear axis bits 3, 4, 5
    reg &= ~((1 << 3) | (1 << 4) | (1 << 5));

    /*
     * - Sensor uses inverted logic:
     *   - 0 = enabled
     *   - 1 = disabled
     */
    reg |= (!x << 3);
    reg |= (!y << 4);
    reg |= (!z << 5);

    return bus->write(BMM150_REG_INT_CONFIG_1, reg);
}

bool BMM150_7Semi::getAxisEnable(bool &x, bool &y, bool &z)
{
    uint8_t reg = 0;

    if (!bus->read(BMM150_REG_INT_CONFIG_1, reg))
        return false;

    /*
     * - Sensor uses inverted logic:
     *   - 0 = enabled
     *   - 1 = disabled
     */
    x = !((reg >> 3) & 0x01);
    y = !((reg >> 4) & 0x01);
    z = !((reg >> 5) & 0x01);

    return true;
}

bool BMM150_7Semi::setLowThreshold(uint8_t threshold)
{
    return bus->write(BMM150_REG_LOW_THRESHOLD, threshold);
}

bool BMM150_7Semi::getLowThreshold(uint8_t &threshold)
{
    if (!bus->read(BMM150_REG_LOW_THRESHOLD, threshold))
        return false;

    return true;
}

bool BMM150_7Semi::setHighThreshold(uint8_t threshold)
{
    return bus->write(BMM150_REG_HIGH_THRESHOLD, threshold);
}

bool BMM150_7Semi::getHighThreshold(uint8_t &threshold)
{
    if (!bus->read(BMM150_REG_HIGH_THRESHOLD, threshold))
        return false;

    return true;
}

bool BMM150_7Semi::setRepetitions(uint16_t xy, uint16_t z)
{
    // Validate limits
    if (xy > 511 || z > 256)
        return false;

    /**
     * Convert to register format
     *
     * - XY: 1 + 2*REP
     * - Z : 1 + REP
     */
    uint8_t xy_rep = (xy - 1) / 2;
    uint8_t z_rep = (z - 1);

    if (!bus->write(BMM150_REG_REP_XY, xy_rep))
        return false;

    if (!bus->write(BMM150_REG_REP_Z, z_rep))
        return false;

    return true;
}

bool BMM150_7Semi::getRepetitions(uint16_t &xy, uint16_t &z)
{
    uint8_t xy_rep = 0;
    uint8_t z_rep = 0;

    if (!bus->read(BMM150_REG_REP_XY, xy_rep))
        return false;

    if (!bus->read(BMM150_REG_REP_Z, z_rep))
        return false;

    /**
     * Convert to actual values
     *
     * - XY: 1 + 2*REP
     * - Z : 1 + REP
     */
    xy = 1 + (2 * xy_rep);
    z = 1 + z_rep;

    return true;
}

bool BMM150_7Semi::enableDataReadyInterrupt()
{
    return bus->write(0x4E, (uint8_t)0x80);
}

bool BMM150_7Semi::getInterruptStatus(uint8_t &status)
{
    return bus->read(BMM150_REG_INTERRUPT_STATUS, status);
}

bool BMM150_7Semi::setAdvSelfTest(bmm150_adv_selftest_t selfTest)
{
    if (selfTest > BMM150_ADV_ST_POS)
        return false;

    if (!bus->writeBits(BMM150_REG_CONTROL, 6, 2, (uint8_t)selfTest))
        return false;

    return true;
}

bool BMM150_7Semi::performSelfTest(float &selfTestResult)
{
    float x, y, z_pos, z_neg;

    // Enter sleep mode
    if (!setSleepMode(true))
        return false;

    // Disable X/Y axes
    if (!setAxisEnable(false, false, true))
        return false;

    // Positive field test
    if (!setAdvSelfTest(BMM150_ADV_ST_POS))
        return false;

    if (!setOpMode(BMM150_MODE_FORCED))
        return false;

    delay(10);

    if (!readMag(x, y, z_pos))
        return false;

    // Negative field test
    if (!setAdvSelfTest(BMM150_ADV_ST_NEG))
        return false;

    if (!setOpMode(BMM150_MODE_FORCED))
        return false;

    delay(10);

    // Restore normal state
    if (!setAdvSelfTest(BMM150_ADV_ST_NORMAL))
        return false;

    if (!setOpMode(BMM150_MODE_SLEEP))
        return false;

    // Compute result
    selfTestResult = (float)(z_pos - z_neg);

    return true;
}