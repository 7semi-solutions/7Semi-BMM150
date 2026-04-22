/**
 * 7Semi BMM150 I2C Basic Read Example
 *
 * Basic Magnetic Field Reading
 *
 * Demonstrates how to initialize the sensor using I2C
 * and read magnetic field measurements.
 *
 * Measured Values
 * - X-axis magnetic field (uT)
 * - Y-axis magnetic field (uT)
 * - Z-axis magnetic field (uT)
 *
 * Sensor Connection (I2C)
 * - VCC  -> 3.3V
 * - GND  -> GND
 * - SDA  -> MCU SDA pin
 * - SCL  -> MCU SCL pin
 *
 * Default I2C Address
 * - 0x10 (SDO = LOW)
 * - 0x13 (SDO = HIGH)
 */

#include <7Semi_BMM150.h>

BMM150_7Semi mag;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("BMM150 7Semi I2C Example");

  // Initialize sensor
  if (!mag.beginI2C(0x13, Wire, 400000)) {
    Serial.println("Sensor init FAILED");
    while (1)
      ;
  }

  Serial.println("Sensor init OK");


  // Data output rate: 10 samples per second
  mag.setODR(BMM150_ODR_10HZ);

  // Continuous measurement mode
  mag.setOpMode(BMM150_MODE_NORMAL);

  // Repetition settings: XY low noise, Z higher stability
  mag.setRepetitions(9, 15);
}

void loop() {
  float x, y, z;

  if (mag.readMag(x, y, z)) {
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" uT  |  ");

    Serial.print("Y: ");
    Serial.print(y);
    Serial.print(" uT  |  ");

    Serial.print("Z: ");
    Serial.print(z);
    Serial.println(" uT");
  } else {
    Serial.println("Read failed");
  }

  delay(200);
}