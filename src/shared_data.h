//shared_data.h

#include <Arduino.h>
#include <Packet.hpp>
enum componentsIDs {
    NRF24ID,
    LSM6ID,
    MAGID,
    GPSID,
    BMPID
};


struct SensorFusion {
    float roll;
    float pitch;
    float yaw;
};

struct MagnometerData {
    float m_x;
    float m_y;
    float m_z;
};

struct LoopRate {
    double HZ;

};

struct CommandData {
    uint8_t pitch;
    uint8_t roll;
    uint8_t yaw;
    uint8_t throttle;
};

struct SystemChecks {
    uint8_t NRF24;
    uint8_t LSM6;
    uint8_t MAG;
    uint8_t GPS;
    uint8_t BMP;
};

struct calibrationValues {
    float mag_bias_x;
    float mag_bias_y;
    float mag_bias_z;
};

struct NRF24Data {
    Packet *pkt_pointer;
    unsigned long time;
};

extern volatile SensorFusion sensorFusionBuffer[2];
extern volatile MagnometerData magnometeDataBuffer[2];
extern volatile LoopRate loopRateBuffer[2];
extern volatile CommandData commandDataBuffer[2];
extern volatile NRF24Data nrf24Data[2];
extern volatile calibrationValues calibrationValuesData;
extern volatile SystemChecks componentStatus;
extern volatile uint8_t sensorFusionCurrent;
extern volatile uint8_t magnometerDataCurrent;
extern volatile uint8_t loopRateCurrent;
extern volatile uint8_t commandCurrent;
extern volatile uint8_t nrf24DataCurrent;

void writeMagnometer(float new_x_value, float new_y_value, float new_z_value);
void writeSensorFusion(float new_roll, float new_pitch, float new_yaw);
void writeLoopRate(float hz_variable);


void readMagnometer(float* x_variable, float* y_variable, float* z_variable);
void readSensorFusion(float* roll_variable, float* pitch_variable, float* yaw_variable);
void readloopRate(float* hz_variable);
void setComponentStatus(bool value, componentsIDs componentID);
bool readComponentStatus(componentsIDs componentID);

// Change this line inside shared_data.h
unsigned long readNRF24Data(Packet** pkt_p);
void writeNRF24Data(Packet* pkt_p);
// Update these lines inside shared_data.h:
