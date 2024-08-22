/* ESP32 Pulse Current Source application header file
    Doug Wagner 5/24/2023 
*/

#ifndef APPLICATION_H
#define APPLICATION_H

#include <stdint.h>
#include <esp32-hal-adc.h>

typedef enum current_range_t{CURRENT_1A, CURRENT_5A, CURRENT_10A, CURRENT_20A} current_range_t;
void adcSetup();
void setCurrent(uint16_t current_mA);
uint32_t calculateCurrent(uint32_t voltage_mV, uint32_t gain, uint32_t resistor_uOhms);
void applicationWorker();

#endif
