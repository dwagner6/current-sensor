/* Implementation of ESP32 Pulse Current Source
    Doug Wagner 5/24/24 
 */

#include <stdint.h>
#include <Arduino.h>
#include "application.h"
#include "pins.h"
#include "tps55289.h"

#define ADC_BIT_WIDTH   12
#define ADC_ZERO_V_OFFSET   142     // Observed mV reading from ADC when measuring zero volts
#define ADC_CONVERSIONS_PER_PIN 10
#define ADC_READOUT_INTERVAL 5000
#define CS_GAIN 20
#define RES_UOHMS_1A 150000UL
#define RES_UOHMS_5A 30000UL
#define RES_UOHMS_10A 15000UL
#define RES_UOHMS_20A 7500UL
#define VCAP_STEP_MV    20U  // amount to step Vcap when trying to reach current set point
#define CURRENT_SET_MARGIN_MA   50U      // Set point considered achieved if within +/-

static uint8_t adcPins[] = {PIN_ADC_1A, PIN_ADC_5A, \
                    PIN_ADC_10A, PIN_ADC_20A};
static uint8_t adcPinCount = sizeof(adcPins) / sizeof(uint8_t);
adc_continuous_data_t *adcResult = NULL;
static bool adcConversionDone = false;

enum current_range_t{CURRENT_1A, CURRENT_5A, CURRENT_10A, CURRENT_20A};
static current_range_t currentRange = CURRENT_1A;
static uint16_t currentSetPoint = 0;
static uint32_t resistance = RES_UOHMS_1A;

enum current_set_state_t{OFF, TOO_LOW, TOO_HIGH, REACHED};
current_set_state_t currentStateMachineState = OFF;

bool readAdcData();

void ARDUINO_ISR_ATTR adcComplete() {
  adcConversionDone = true;
}


void adcSetup()
{
    // Set 12-bit ADC width
    analogContinuousSetWidth(ADC_BIT_WIDTH);
    // Set 11 dB atten == 3.1V max reading
    analogContinuousSetAtten(ADC_11db);
    // Setup ADC Continuous with following input:
    // array of pins, count of the pins, how many conversions 
    // per pin in one cycle will happen, sampling frequency, callback function
    analogContinuous(adcPins, adcPinCount, ADC_CONVERSIONS_PER_PIN, 20000, &adcComplete);
    // Start conversions
    analogContinuousStart();
}

void currentStateMachine()
{
    static uint16_t vcap_set = 800;     // Vcap output set point
    uint16_t current = 0;        // calculated current value
    // mV reading from ADC of current sensor output
    uint32_t current_mV;

    // If new ADC data is ready
    if(readAdcData())
    {
        // record adc mv value from selected current sensor
        current_mV = adcResult[currentRange].avg_read_mvolts;
        current = calculateCurrent(current_mV, CS_GAIN, resistance);
    }

    static uint32_t now = 0;
    static uint32_t prev;
    now = millis();
    if((now - prev) > 1000)
    {
        Serial.printf("ADC: \t%d mV\n\r", current_mV);
        Serial.printf("Current: %d mA\n\r", current);
        Serial.printf("state = %d\n\r", currentStateMachineState);
        Serial.printf("currentRange = %d\n\r", currentRange);
        Serial.printf("currentSetPoint = %d\n\r", currentSetPoint);
        Serial.printf("\n\r");
        prev = now;
    }

    switch(currentStateMachineState)
    {
        case OFF: 
            if(currentSetPoint > 0)
            {
                currentStateMachineState = TOO_LOW;
                tps55289_enable_output();
            }
            break;
        case TOO_LOW:
            if(currentSetPoint == 0)
            {
                currentStateMachineState = OFF;
                tps55289_set_vout(0);
                tps55289_disable_output();
                break;
            }
            // Check if current hasn't gone too high
            else if( (current > currentSetPoint + CURRENT_SET_MARGIN_MA) )
            {
                currentStateMachineState = TOO_HIGH;
                break;
            }
            // else if current is within set point margin
            else if( ((current-currentSetPoint) <CURRENT_SET_MARGIN_MA) \
                    || ((currentSetPoint-current) <CURRENT_SET_MARGIN_MA))
            {
                currentStateMachineState = REACHED;
                break;
            }
            else
            {
                vcap_set += VCAP_STEP_MV;
                tps55289_set_vout(vcap_set);
                break;
            }
        
        case TOO_HIGH:
            if(currentSetPoint == 0)
            {
                currentStateMachineState = OFF;
                tps55289_set_vout(0);
                tps55289_disable_output();
                break;
            }
            // Check if current hasn't gone too low
            else if( (current < currentSetPoint - CURRENT_SET_MARGIN_MA) )
            {
                currentStateMachineState = TOO_LOW;
                break;
            }
            // else if current is within set point margin
            else if( ((current-currentSetPoint) <CURRENT_SET_MARGIN_MA) \
                    || ((currentSetPoint-current) <CURRENT_SET_MARGIN_MA))
            {
                currentStateMachineState = REACHED;
                break;
            }
            else
            {
                vcap_set -= VCAP_STEP_MV;
                tps55289_set_vout(vcap_set);
                break;
            }

        case REACHED:
            if(currentSetPoint == 0)
            {
                currentStateMachineState = OFF;
                tps55289_disable_output();
                break;
            }
            // Check if current hasn't gone too high
            else if( (current > currentSetPoint + CURRENT_SET_MARGIN_MA) )
            {
                currentStateMachineState = TOO_HIGH;
                break;
            }
            // Check if current hasn't gone too low
            else if( (current < currentSetPoint - CURRENT_SET_MARGIN_MA) )
            {
                currentStateMachineState = TOO_LOW;
                break;
            }
            else
                break;

    }
}

void applicationWorker()
{
    currentStateMachine();
}

bool readAdcData()
{
    if(analogContinuousRead(&adcResult, 0))
    {
        return true;
    }
    else
        return false;
}

void setCurrent(uint16_t current_mA)
{
    currentSetPoint = current_mA;

    // Pick which current sense range to use
    // and specify sense resistor value
    if(currentSetPoint < 800)
    {
        currentRange = CURRENT_1A;
        resistance = RES_UOHMS_1A;
    }
    else if(currentSetPoint < 4200)
    {
        currentRange = CURRENT_5A;
        resistance = RES_UOHMS_5A;
    }
    else if(currentSetPoint < 8500)
    {
        currentRange = CURRENT_10A;
        resistance = RES_UOHMS_10A;
    }
    else 
    {
        currentRange = CURRENT_20A;
        resistance = RES_UOHMS_20A;
    }
}

// Function to convert amplifier output voltage to measured current using integer arithmetic
uint32_t calculateCurrent(uint32_t voltage_mV, uint32_t gain, uint32_t resistor_uOhms) 
{
    // Convert the voltage from millivolts to microvolts
    uint64_t voltage_uV = (uint64_t)voltage_mV * 1000;
    // Convert the resistor value from milliohms to microohms
    uint64_t current_mA = (voltage_uV * 1000) / ((uint64_t)gain * (uint64_t)resistor_uOhms);
    
    return (uint32_t)current_mA;
}