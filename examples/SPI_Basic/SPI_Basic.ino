/**
 * 7Semi BMM150 SPI Basic Read Example
 *
 * Basic Magnetic Field Reading (SPI Mode)
 *
 * Demonstrates how to initialize the sensor using SPI
 * and read magnetic field measurements.
 *
 * Measured Values
 * - X-axis magnetic field (uT)
 * - Y-axis magnetic field (uT)
 * - Z-axis magnetic field (uT)
 *
 * Sensor Connection (SPI)
 * - VCC  -> 3.3V
 * - GND  -> GND
 * - SCK  -> MCU SPI Clock
 * - MISO -> MCU MISO
 * - MOSI -> MCU MOSI
 * - CS   -> MCU Chip Select pin
 * - PS   -> GND
 */

#include <7Semi_BMM150.h>


BMM150_7Semi mag;

/**
 * Chip Select pin
 */
#define BMM150_CS 10

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("BMM150 7Semi SPI Example");

    /**
     * Initialize SPI sensor
     *
     * - CS pin
     * - SPI port
     * - SPI clock (1MHz recommended)
     */
    if (!mag.beginSPI(BMM150_CS, SPI, 1000000))
    {
        Serial.println("Sensor init FAILED");
        while (1);
    }

    Serial.println("Sensor init OK");

    // Configure sensor
    mag.setODR(BMM150_ODR_10HZ);        // Output data rate
    mag.setOpMode(BMM150_MODE_NORMAL);  // Continuous mode
    mag.setRepetitions(9, 15);          // Balanced accuracy
}

void loop()
{
    float x, y, z;

    if (mag.readMag(x, y, z))
    {
        Serial.print("X: ");
        Serial.print(x);
        Serial.print(" uT  |  ");

        Serial.print("Y: ");
        Serial.print(y);
        Serial.print(" uT  |  ");

        Serial.print("Z: ");
        Serial.print(z);
        Serial.println(" uT");
    }
    else
    {
        Serial.println("Read failed");
    }

    delay(200);
}