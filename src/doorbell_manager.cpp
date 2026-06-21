#include <Arduino.h>
#include <doorbell_manager.h>
#include <hardware/irq.h>
#include <pico/multicore.h>
volatile uint doorbell_nrf24_recieved_ready = 0;
volatile uint doorbell_nrf24_package_ready = 0;

void setupISRCore0Reciev();
void setupISRCore1Reciev();
void enableISRCore0Reciev();
void enableISRCore1Reciev();
void disableISRCore0Reciev();
void disableISRCore1Reciev();