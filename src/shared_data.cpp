//shared_data.cpp
#include <shared_data.h>
#include <Packet.hpp>

volatile SensorFusion sensorFusionBuffer[2];
volatile MagnometerData magnometeDataBuffer[2];
volatile LoopRate loopRateBuffer[2];
volatile CommandData commandDataBuffer[2];
volatile NRF24Data nrf24Data[2];
volatile calibrationValues calibrationValuesData;
volatile SystemChecks componentStatus;

volatile uint8_t sensorFusionCurrent = 0;
volatile uint8_t magnometerDataCurrent = 0;
volatile uint8_t loopRateCurrent = 0;
volatile uint8_t commandCurrent = 0;
volatile uint8_t nrf24DataCurrent = 0;


void writeSensorFusion(float new_roll, float new_pitch, float new_yaw) {
    uint8_t current = 1 - sensorFusionCurrent;
    sensorFusionBuffer[current].roll = new_roll;
    sensorFusionBuffer[current].pitch = new_pitch; 
    sensorFusionBuffer[current].yaw = new_yaw;
    __dmb(); 
    sensorFusionCurrent = current;
}

void writeMagnometer(float new_x_value, float new_y_value, float new_z_value){
    uint8_t current = 1 - magnometerDataCurrent;
    magnometeDataBuffer[current].m_x = new_x_value;
    magnometeDataBuffer[current].m_y = new_y_value;
    magnometeDataBuffer[current].m_z = new_z_value;
    __dmb();
    magnometerDataCurrent = current;
}

void writeLoopRate(float hz_variable) {
    uint8_t currentLoop = 1 - loopRateCurrent;
    loopRateBuffer[currentLoop].HZ = hz_variable;
    __dmb();
    loopRateCurrent = currentLoop;
}


void readMagnometer(float* x_variable, float* y_variable, float* z_variable) {
    uint8_t current = *(volatile uint8_t*)&magnometerDataCurrent;
    __dmb(); 
    *x_variable = magnometeDataBuffer[current].m_x;
    *y_variable = magnometeDataBuffer[current].m_y;
    *z_variable = magnometeDataBuffer[current].m_z;
}

void readSensorFusion(float* roll_variable, float* pitch_variable, float* yaw_variable) {
    uint8_t current = *(volatile uint8_t*)&sensorFusionCurrent;
    __dmb(); 
    *roll_variable = sensorFusionBuffer[current].roll;
    *pitch_variable = sensorFusionBuffer[current].pitch;
    *yaw_variable = sensorFusionBuffer[current].yaw;
}

void readloopRate(float* hz_variable) {
    uint8_t currentLoop = *(volatile uint8_t*)&loopRateCurrent;
    __dmb(); 
    *hz_variable = (float)loopRateBuffer[currentLoop].HZ; 
}

unsigned long readNRF24Data(Packet** pkt_p) {
    uint8_t current = *(volatile uint8_t*)&nrf24DataCurrent;
    __dmb(); 
    *pkt_p = nrf24Data[current].pkt_pointer;
    return nrf24Data[current].time;
}

void writeNRF24Data(Packet* pkt_p) {
    uint8_t next = 1 - nrf24DataCurrent;
    nrf24Data[next].pkt_pointer = pkt_p;
    nrf24Data[next].time = millis();
    __dmb(); 
    nrf24DataCurrent = next;
}

void setComponentStatus(bool value, componentsIDs componentID) {
    switch (componentID) {
        case (NRF24ID): {
            componentStatus.NRF24 = value;
            break;
        } case (LSM6ID): {
            componentStatus.LSM6 = value;
            break;
        } case (MAGID): {
            componentStatus.MAG = value;
            break;
        } case (GPSID): {
            componentStatus.GPS = value;
            break;
        } case (BMPID): {
            componentStatus.BMP = value;
            break;
        } 
        default:
            break;
    }
}

bool readComponentStatus(componentsIDs componentID) {
    switch (componentID) {
        case (NRF24ID): {
            return componentStatus.NRF24;
            break;
        } case (LSM6ID): {
            return componentStatus.LSM6;
            break;
        } case (MAGID): {
            return componentStatus.MAG;
            break;
        } case (GPSID): {
            return componentStatus.GPS;
            break;
        } case (BMPID): {
            return componentStatus.BMP;
            break;
        } default:
            return false;
    }
}