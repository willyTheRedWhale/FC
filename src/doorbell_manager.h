#include <Arduino.h>
#include <hardware/irq.h>
#include <pico/multicore.h>

#define DOORBELL_CORE0_RECEIVES  5
#define DOORBELL_CORE1_RECEIVES  6 

extern volatile uint doorbell_nrf24_recieved_ready;
extern volatile uint doorbell_nrf24_package_ready;

void setupISRCore0Reciev();
void setupISRCore1Reciev();
void enableISRCore0Reciev();
void enableISRCore1Reciev();
void disableISRCore0Reciev();
void disableISRCore1Reciev();