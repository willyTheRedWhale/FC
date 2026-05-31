//shared.h
#pragma once

#include <Arduino.h>
#include "LSM6DSOXSensor.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MMC56x3.h>

extern volatile bool lsm6_interrupt;

void setupLED();
void togle();
void showStatusLED(uint16_t count);

void setupSPIPins(SPIClassRP2040 *spi);
void beginSPI(SPIClassRP2040 *spi);

void setupI2CPins(TwoWire *wire);
void beginWire(TwoWire *wire);

void beginLSM6(LSM6DSOXSensor *sensor);
bool checkLSM6(LSM6DSOXSensor *sensor);
void configreLSM6(LSM6DSOXSensor *sensor);
void interrupthanlder();
void setuplsm6Interrupt();
void beginMMCMag(Adafruit_MMC5603* mag, TwoWire* wire);
void setupMMCMag(Adafruit_MMC5603 * mag);
void calibrateMagnetometer();

void setupGPS(SerialUART *serial);
void beginGPS(SerialUART *serial);