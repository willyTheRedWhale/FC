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
#include <pico/multicore.h>
#include <hardware/irq.h>
#include <doorbell_manager.h>

uint doorbell_nrf24_recieved_ready;
extern uint doorbell_nrf24_package_ready;
constexpr float looprateMAG = 1000.0f / MAGOUTPUTRATE;

Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);
TinyGPSPlus gps;

unsigned long output = 0;
unsigned long magread = 0;
float data[4];

void displayInfo();
void core0_doorbell_isr();
void setup() {

  setupLED();
  Serial.begin(BaudRate); 
  multicore_doorbell_claim(DOORBELL_CORE0_RECEIVES, 0b01);
  doorbell_nrf24_recieved_ready = DOORBELL_CORE0_RECEIVES;
  uint irq = multicore_doorbell_irq_num(doorbell_nrf24_recieved_ready);
  irq_add_shared_handler(irq, core0_doorbell_isr, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  irq_set_enabled(irq, true);

  setupGPS(&Serial1);
  beginGPS(&Serial1);
  
  showStatusLED(1);
  showStatusLED(1);
  setupI2CPins(&Wire1);
  beginWire(&Wire1);

  beginMMCMag(&mmc, &Wire1);
  setupMMCMag(&mmc);
  calibrateMagnetometer();
  setComponentStatus(true, MAGID);
  setComponentStatus(true, GPSID);

  Serial.println("============== Components: ==============");
  Serial.print(F("NRF24: ")); Serial.println(readComponentStatus(NRF24ID) ? F("Active") : F("Inactive"));
  Serial.print(F("LSM6:  ")); Serial.println(readComponentStatus(LSM6ID) ? F("Active") : F("Inactive"));
  Serial.print(F("GPS:   ")); Serial.println(readComponentStatus(GPSID) ? F("Active") : F("Inactive"));
  Serial.print(F("Mag:   ")); Serial.println(readComponentStatus(MAGID) ? F("Active") : F("Inactive"));
  Serial.println(F("=========================================="));

  delay(1000);
}
sensors_event_t event;
void loop() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  } 
  
  if (millis() - magread > looprateMAG) {
    
    mmc.getEvent(&event);
    event.magnetic.x -= calibrationValuesData.mag_bias_x;
    event.magnetic.y -= calibrationValuesData.mag_bias_y;
    event.magnetic.z -= calibrationValuesData.mag_bias_z;
    writeMagnometer(event.magnetic.x, event.magnetic.y, event.magnetic.z);
    magread = millis();

  }
  if (millis() - output > 100 ) {

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
    output = millis();
    multicore_doorbell_set_other_core(doorbell_nrf24_package_ready);
  }

  if (rp2040.fifo.available() > 0) {
    rp2040.fifo.pop(); 
    //Serial.println("\n====== SUCCESS: Core 1 Interrupted by Core 0! ======");
  }
}

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

void core0_doorbell_isr() {
    Serial.println("ISR fired on Core 0"); // outside the if
    if (multicore_doorbell_is_set_current_core(doorbell_nrf24_recieved_ready)) {
        multicore_doorbell_clear_current_core(doorbell_nrf24_recieved_ready);
        Serial.println("Core 0 got interrupted by Core 1");
    }
}