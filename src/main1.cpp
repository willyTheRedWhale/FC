//Core 1


#include <Arduino.h>
#include "LSM6DSOXSensor.h"
#include "SensorFusion.h" //SF
#include <RF24.h>
#include <SPI.h>

#include "Packet.hpp"
#include "Connection.hpp"
#include <RFSlave.hpp>
#include <shared.h>
#include <shared_data.h>

RF24      radio(NRF24_CE_PIN, NRF24_CSN_PIN);
RFSlave   slave(radio);
LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&SPI1, IMU_CE_PIN);
SF fusion;


int32_t acceleration[3];
int32_t rotation[3];


void setup1() {
  
  setupSPIPins(&SPI1);
  beginSPI(&SPI1);

  radio.setStatusFlags(RF24_RX_DR);
#if DEBUG
  //Serial.println("Initializing RFSlave...");
#endif
  Connection conn;
  uint8_t addr[5] = { 'R', 'F', 'T', 'E', 'S' };
  conn.setDataRate(RF24_250KBPS);
  conn.setPALevel(RF24_PA_MAX);
  conn.setAddress(addr);
  conn.setChannel(76);
  conn.setSPI(&SPI1);
  slave.init(2, conn);

  ConfirmationPacket* confirm = new ConfirmationPacket(1);
  slave.queueAckPacket(confirm);
  setComponentStatus(radio.isChipConnected(), NRF24ID);
#if DEBUG
  /*Serial.println(radio.isChipConnected()
      ? "RF24 detected and connected!"
      : "RF24 not detected or not connected!");*/
  //radio.printPrettyDetails();
#endif
  lsm6dsoxSensor.begin();

  bool lsm6_status = checkLSM6(&lsm6dsoxSensor);
  configreLSM6(&lsm6dsoxSensor);
  setuplsm6Interrupt();
  setComponentStatus(lsm6_status ,LSM6ID);
  lsm6dsoxSensor.Get_X_Axes(acceleration);
  lsm6dsoxSensor.Get_G_Axes(rotation);
}


unsigned long loopTime = 0;
unsigned long outputTimer = 0;
float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;

int32_t counter = 0;
unsigned long timer_hz = 0;


uint8_t last_mag_index = 0;

unsigned long lastInterruptTrigger = 0;

void loop1() {
  if (lsm6_interrupt) {
    lsm6_interrupt = false;
    lsm6dsoxSensor.Get_X_Axes(acceleration);
    lsm6dsoxSensor.Get_G_Axes(rotation);
    counter++;

    if (millis() - timer_hz > 1000) {
      float result = counter / ((millis() - timer_hz) / 1000.0f);
      writeLoopRate(result);
      timer_hz = millis();
      counter = 0;
    }
    
    // Convert IMU data
    float gx_rads = (float)rotation[0] / 1000.0f * DEG_TO_RAD;
    float gy_rads = (float)rotation[1] / 1000.0f * DEG_TO_RAD;
    float gz_rads = (float)rotation[2] / 1000.0f * DEG_TO_RAD;
    float ax_g = (float)acceleration[0] / 1000.0f;
    float ay_g = (float)acceleration[1] / 1000.0f;
    float az_g = (float)acceleration[2] / 1000.0f;

    deltat = fusion.deltatUpdate();
    uint8_t current_mag_index = *(volatile uint8_t*)&magnometerDataCurrent;
    readMagnometer(&mx, &my, &mz);
    float final_mx =  my / 100.0f;   // no negation
    float final_my = -mx / 100.0f;   // negated
    float final_mz =  mz / 100.0f;
    fusion.MadgwickUpdate(gx_rads, gy_rads, gz_rads, ax_g, ay_g, az_g, final_mx, final_my, final_mz, deltat);

    writeSensorFusion(fusion.getRoll(), fusion.getPitch(), fusion.getYaw());
  }
  
  if (millis() - lastInterruptTrigger > 1000) {
    rp2040.fifo.push_nb(1); 
    lastInterruptTrigger = millis();
  }
}