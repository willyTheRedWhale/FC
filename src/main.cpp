// Core 0
#include <Arduino.h>

#include <Adafruit_MMC56x3.h>
#include <TinyGPSPlus.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>

#include "Packet.hpp"
#include "Connection.hpp"
#include <RFSlave.hpp>
#include <shared.h>
#include <shared_data.h>



#include <malloc.h>
#include <stdio.h>
#include "pico/time.h"

constexpr float looprateMAG = 1.0f / MAGOUTPUTRATE * 1000000.0f;
volatile bool ISRMagnetometerValue = false;
repeating_timer_t Magnetometertimer;

#if DEBUG
constexpr float looprateSerialOutput = 1.0f / SERIALOUTRATE * 1000000.0f;
volatile bool ISRSerialOutputValue = false;
repeating_timer_t SerialOutputtimer;
#endif

Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);
sensors_event_t event;
TinyGPSPlus gps;

unsigned long output = 0;
unsigned long magread = 0;
float data[4];

void displayInfo();
bool ISRMagnetometer(repeating_timer_t* rt);

#if DEBUG
bool ISRSerialOutput(repeating_timer_t* rt);
#endif

void setup() {
  setupLED();
#if DEBUG
  Serial.begin(BaudRate); 
#endif
  setupGPS(&Serial1);
  beginGPS(&Serial1);
  
  showStatusLED(1);
  setupI2CPins(&Wire1);
  beginWire(&Wire1);

  beginMMCMag(&mmc, &Wire1);
  setupMMCMag(&mmc);
  calibrateMagnetometer();
  setComponentStatus(true, MAGID);
  setComponentStatus(true, GPSID);
#if DEBUG
  Serial.println("============== Components: ==============");
  Serial.print(F("NRF24: ")); Serial.println(readComponentStatus(NRF24ID) ? F("Active") : F("Inactive"));
  Serial.print(F("LSM6:  ")); Serial.println(readComponentStatus(LSM6ID) ? F("Active") : F("Inactive"));
  Serial.print(F("GPS:   ")); Serial.println(readComponentStatus(GPSID) ? F("Active") : F("Inactive"));
  Serial.print(F("Mag:   ")); Serial.println(readComponentStatus(MAGID) ? F("Active") : F("Inactive"));
  Serial.println(F("=========================================="));
#endif
  delay(1000);
  add_repeating_timer_us(-looprateMAG, ISRMagnetometer, nullptr, &Magnetometertimer);
#if DEBUG
  add_repeating_timer_us(-looprateSerialOutput, ISRSerialOutput, nullptr, &SerialOutputtimer);
#endif
}

void loop() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  } 
  
  if (ISRMagnetometerValue) {
    
    mmc.getEvent(&event);
    event.magnetic.x -= calibrationValuesData.mag_bias_x;
    event.magnetic.y -= calibrationValuesData.mag_bias_y;
    event.magnetic.z -= calibrationValuesData.mag_bias_z;
    writeMagnometer(event.magnetic.x, event.magnetic.y, event.magnetic.z);
    ISRMagnetometerValue = false;

  }
#if DEBUG
  if (ISRSerialOutputValue) {

    readSensorFusion(&data[0], &data[1], &data[2]);
    readloopRate(&data[3]);

    float yaw_display = 360.0f - data[2] - 0;
    if (yaw_display >= 360.0f) yaw_display -= 360.0f;
    //Serial.printf(">Roll:%.2f \n>Pitch:%.2f \n>Yaw:%.2f \n>HZ:%.2f\n", data[0], data[1], data[2], data[3]);
    Serial.print(">Roll:"); Serial.println(data[0]);
    Serial.print(">Pitch:"); Serial.println(data[1]);
    Serial.print(">Yaw:"); Serial.println(yaw_display);
    Serial.print(">Hz:"); Serial.println(data[3]);
    //Serial.printf("Mag X: %.5f Y: %.5f Z: %.5f uT\n", event.magnetic.x / 1000.0f, event.magnetic.y / 1000.0f, event.magnetic.z / 1000.0f);
    //displayInfo();
    ISRSerialOutputValue = false;
  }
#endif

  if (rp2040.fifo.available() > 0) {
    rp2040.fifo.pop(); 
    //Serial.println("\n====== SUCCESS: Core 1 Interrupted by Core 0! ======");
  }
}

#if DEBUG
void displayInfo() {

  Serial.print(F("Location: ")); 
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  } else {
    Serial.print(F("INVALID"));
  }
  Serial.print(F("\n"));
}
#endif

bool ISRMagnetometer(repeating_timer_t* rt) {
  ISRMagnetometerValue = true;
  return true;
}

#if DEBUG
bool ISRSerialOutput(repeating_timer_t* rt) {
  ISRSerialOutputValue = true;
  return true;
}
#endif