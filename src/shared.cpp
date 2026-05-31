//shared.cpp
#include <Arduino.h>
#include "LSM6DSOXSensor.h"
#include <Adafruit_MMC56x3.h>

#include <SPI.h>
#include "Wire.h"
#include <shared.h>

#include <shared_data.h>

volatile bool lsm6_interrupt = false;


void setupLED() {
    pinMode(BUILTIN_LED_PIN, OUTPUT);
    digitalWrite(BUILTIN_LED_PIN, LOW);
}

void togle() {
    digitalWrite(BUILTIN_LED_PIN, !digitalRead(BUILTIN_LED_PIN));
}

void showStatusLED(uint16_t count) {
    for (uint16_t i = 0; i < count * 2; i ++) {
        togle();
        delay((int)(LEDTIME/count));
    }
}
// SPI
void setupSPIPins(SPIClassRP2040 *spi) {
    spi->setSCK(SPI_SCK_PIN);
    spi->setMOSI(SPI_MOSI_PIN);
    spi->setMISO(SPI_MISO_PIN);
}
void beginSPI(SPIClassRP2040 *spi) {
    spi->begin();
}
// Wire
void setupI2CPins(TwoWire *wire) {
    wire->setSCL(I2C_SCL);
    wire->setSDA(I2C_SDA);
}
void beginWire(TwoWire *wire) {
    wire->setClock(400000);
    wire->begin();
}

void beginLSM6(LSM6DSOXSensor *sensor) {
    sensor->begin();
}

bool checkLSM6(LSM6DSOXSensor *sensor) {
    if (sensor->Enable_G() == LSM6DSOX_OK && sensor->Enable_X() == LSM6DSOX_OK) {
        if (DEBUG) Serial.println("Success enabling accelero and gyro");
    } else {
        if (DEBUG) Serial.println("Error enabling accelero and gyro");
        showStatusLED(5);
        return false;
    }
    uint8_t id;
    sensor->ReadID(&id);
    if (id != LSM6DSOX_ID) {
        if (DEBUG) Serial.println("Wrong id for LSM6DSOX sensor. Check that device is plugged");
        showStatusLED(5);
        return false;
    } else {
        if (DEBUG) Serial.println("Success checking id for LSM6DSOX sensor");
    }
    return true;
}

void configreLSM6(LSM6DSOXSensor *sensor) {
    sensor->Enable_G();
    sensor->Enable_X();
    sensor->Set_X_FS(2);
    // Set gyroscope scale. Available values are: 125, 250, 500, 1000, 2000 dps
    sensor->Set_G_FS(250);
    // Set accelerometer Output Data Rate. Available values are: 1.6, 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
    sensor->Set_X_ODR(SR);
    // Set gyroscope Output Data Rate. Available values are 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
    sensor->Set_G_ODR(SR);
    uint8_t data = 0x03;
    sensor->IO_Write(&data, LSM6DSOX_INT1_CTRL, 1);
    sensor->Set_Interrupt_Latch(0);
}

void interrupthanlder() {
    lsm6_interrupt = true;
    static int count = 0;
    if (count++ < 5) Serial.println("IRQ fired");
}

void setuplsm6Interrupt() {
    pinMode(LSM6_IRQ, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LSM6_IRQ), interrupthanlder, RISING);

}

void beginMMCMag(Adafruit_MMC5603* mag, TwoWire* wire) {
  if (DEBUG) Serial.println("Adafruit_MMC5603 Continuous Mode Magnetometer Test");
  if (DEBUG) Serial.println("");
  if (!mag->begin(MMC56X3_DEFAULT_ADDRESS, wire)) {  // I2C mode
    /* There was a problem detecting the MMC5603 ... check your connections */
    if (DEBUG) Serial.println("Ooops, no MMC5603 detected ... Check your wiring!");
    while (1) delay(10);
  }
}

void setupMMCMag(Adafruit_MMC5603 * mag) {
  if (DEBUG) mag->printSensorDetails();

  mag->setDataRate(MAGOUTPUTRATE); // in Hz, from 1-255 or 1000
  mag->setContinuousMode(true);
}

void calibrateMagnetometer() {
  if (DEBUG) Serial.println("Calibrating Magnetometer... ROTATE !");
  //gotten from a code, that read values.
  float MagMinX = -37.92, MagMaxX = 55.17;
  float MagMinY = -74.05, MagMaxY = 16.84;
  float MagMinZ = -53.57, MagMaxZ = 40.36;

  
  
  
  // Calculate the midpoint offset (Hard iron offset)
  calibrationValuesData.mag_bias_x = (MagMaxX + MagMinX) / 2.0f;  // 7.57
  calibrationValuesData.mag_bias_y = (MagMaxY + MagMinY) / 2.0f;  // -18.92
  calibrationValuesData.mag_bias_z = (MagMaxZ + MagMinZ) / 2.0f;  // 22.96

  if (DEBUG) {
    Serial.println("Calibration Done!");
    Serial.print("Offsets -> X: "); Serial.print(calibrationValuesData.mag_bias_x );
    Serial.print(" Y: "); Serial.print(calibrationValuesData.mag_bias_y);
    Serial.print(" Z: "); Serial.println(calibrationValuesData.mag_bias_z);
    //Serial.print((MagMaxX )); Serial.print(" "); Serial.print((MagMinX)); Serial.print(" "); Serial.print((MagMaxZ + MagMinZ));
  }
}

void setupGPS(SerialUART *serial) {
    serial->setRX(GPSRX);
    serial->setTX(GPSTX);
}

void beginGPS(SerialUART *serial) {
    serial->begin(9600);
}