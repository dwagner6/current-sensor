#include <Arduino.h>
#include <driver/ledc.h>
#include <stdint.h> // Include stdint.h for fixed-width data types
#include <Bounce2.h>
#include <Wire.h>
#include <esp32-hal-adc.h>
#include <PID_v1.h>

#include "pins.h"
#include <pulser.h>
#include <tps55289.h>
#include <utils.h>
#include "application.h"

#define NUM_READINGS (uint8_t)50 // Number of ADC readings to average
#define BAUD_RATE 115200
#define DEBOUNCE_INTERVAL_MS 5
#define INPUT_SIZE 30
#define DELAY 1 // supposedly uS
#define VOUT_MV_MIN 800  // Min and max Vout on TPS55289 in mV
#define VOUT_MV_MAX 15000
#define LEDC_CHAN 0

// TODO: Find minimum timeout necessary for Labview to send commands
// in order to speed up main loop
#define SERIAL_TIMEOUT_MS 200 // Timeout when waiting for serial data

// Maximum bit resolution of LEDC timers (used for setting freq, duty)
#define LEDC_BIT_RESOLUTION 14
// Calculate max value of whatever LEDC bit width is chosen (used for
// calculating duty)
#define LEDC_BITWIDTH_MAX_VAL (UINT16_MAX >> (16U-LEDC_BIT_RESOLUTION))
//#define DEBUG_ON


volatile uint32_t pulseWidth = DEFAULT_PULSEWIDTH;
volatile bool ledcEnabled = true;
volatile bool transmitReading = false;
volatile bool flipbit = false;
enum {FREQ_SET, WIDTH_SET, CURR_SET, ONOFF_SET, \
                VOUT_SET, STATUS_SET, ADC_SET};

esp_err_t error = ESP_OK;

hw_timer_t *timer = NULL;
volatile bool timerEnabled = false;
bool settings_changed = false;
uint32_t freq = DEFAULT_FREQ_HZ;
uint8_t adc_chan = 0;
uint16_t vout = 0;
uint16_t current = 0;
uint16_t vout_mv = 0;
int8_t setting = -1;
uint32_t settings_reg[7] = {DEFAULT_FREQ_HZ, DEFAULT_PULSEWIDTH, \
                            0, 0, 0, 0, 0};
extern current_range_t currentRange;
extern uint32_t current_mA[4];

uint64_t lastAdcReadout = 0;
uint64_t currentAdcReadout;

uint32_t calculateDuty(uint32_t freq, uint32_t pulseWidth_ns);
void int_to_hex_str(uint8_t num_digits, uint32_t value, char * hex_string);

void setup()
{
    pinMode(TIMER_PIN, OUTPUT);
    Serial.begin(BAUD_RATE);

// Shorter timeout when not manually sending commands/debugging
#ifndef DEBUG_ON
    Serial.setTimeout(SERIAL_TIMEOUT_MS);
#endif
    if(!ledcAttach(PIN_PULSE_OUTPUT, DEFAULT_FREQ_HZ, LEDC_BIT_RESOLUTION))
    {
        Serial.println("ledc peripheral config error!");
        while(1){}
    };    

    pinMode(TPS55289_EN_PIN, OUTPUT);
    digitalWrite(TPS55289_EN_PIN, HIGH);

    applicationSetup();
}

void loop()
{
    // Variables to hold the command and command value from UART string
    char command[10];
    char value[10];
    bool valid_command = true;

    applicationWorker();

    char input_str[INPUT_SIZE];
    uint8_t size = 0;
    if(Serial.available() > 0)
    {
        size = Serial.readBytes(input_str, INPUT_SIZE);
    
    } 

    if (size != 0)
    {
        // Scan string for command (ie: string followed by number, 
        // seperated by a space) 
        int8_t matches = sscanf(input_str, "%s %s", &command, &value);

        // Convert to Arduino String to make things easier
        String command_str(command);
        String value_str(value);
        uint16_t value_int = value_str.toInt();
        //Serial.printf("sscanf result: %s %s %d\n\r", command_str, value_str, value_int);

        if (command_str.equals("freq"))
        {
            setting = FREQ_SET;
        }
        else if (command_str.equals("width"))
        {
            setting = WIDTH_SET; 
        }
        else if (command_str.equals("curr"))
        {
            setting = CURR_SET;
        }
        else if (command_str.equals("onoff"))
        {
            setting = ONOFF_SET; 
        }
        else if (command_str.equals("vout"))
        {
            setting = VOUT_SET;
        }
        else if (command_str.equals("status"))
        {
            setting = STATUS_SET;
        }
        else if (command_str.equals("adc"))
        {
            setting = ADC_SET;
            if(value_int > 3)
            {
                // Default to adc chan 3 if invalid channel value
                value_int = 3;
            }
            adc_chan = (uint8_t) value_int;
        }
        else
        {
            Serial.println("Error: no matching command!");
            valid_command = false;
        }

        // If we're not reading a setting's value, update setting register
        if ( valid_command && !value_str.startsWith("?"))
        {
            Serial.println("OK");
            settings_reg[setting] = value_int;
            settings_changed = true;
        }
        
        // Else if we're reading a setting's value, print it
        else if (valid_command && value_str.equals("?"))
        {
            Serial.println("OK");
            Serial.println(settings_reg[setting]);
        }
        // Anything else is an error
        else
            Serial.println("Error!");

        // Special case: adc command only prints the most recent adc channel value
        if(command_str.equals("adc"))
        {
            Serial.printf("%d\n\r", current_mA[adc_chan]);
        }

    }

    if (settings_changed)
    {
        settings_changed = false;

        freq = settings_reg[FREQ_SET];
        if (freq < DEFAULT_MIN_FREQ)
            freq = DEFAULT_MIN_FREQ;
        if (freq > DEFAULT_MAX_FREQ)
            freq = DEFAULT_MAX_FREQ;
        settings_reg[FREQ_SET] = freq;

        pulseWidth = settings_reg[WIDTH_SET];
        if (pulseWidth < DEFAULT_MIN_PULSEWIDTH)
            pulseWidth = DEFAULT_MIN_PULSEWIDTH;
        if (pulseWidth > DEFAULT_MAX_PULSEWIDTH)
            pulseWidth = DEFAULT_MAX_PULSEWIDTH;
        settings_reg[WIDTH_SET] = pulseWidth;

        uint16_t prevCurrent = current;
        current = settings_reg[CURR_SET];
        if (current > 20000U)
            current = 20000U;
        settings_reg[CURR_SET] = current;

        if(prevCurrent != current)
            setCurrentRange(current);

        if (!settings_reg[ONOFF_SET])
        {
            pulseWidth = 0;
            tps55289_disable_output();
            //setCurrent(0);
        }
        else
        {
            //setCurrent(current);
            tps55289_enable_output();
        }
       
        vout = settings_reg[VOUT_SET];
        if(vout < VOUT_MV_MIN)
        {
            vout = VOUT_MV_MIN;
        }
        else if(vout > VOUT_MV_MAX)
        {
            vout = VOUT_MV_MAX;
        }
        tps55289_set_vout(vout);

        //adc_chan = settings_reg[ADC_SET];
        //Serial.println(current_mA[adc_chan], DEC);

        // Change pulse output frequency to freq, actual frequency after
        // calculation at bit resolution stored in frequency
        uint32_t frequency;
        frequency = ledcChangeFrequency(PIN_PULSE_OUTPUT, freq, LEDC_BIT_RESOLUTION);
        // DEBUG
        // Serial.printf("ledc frequency: %d\n\r", frequency);

        // Calculate duty cycle based on current frequency and pulse width
        uint32_t duty = calculateDuty(frequency, pulseWidth);
        ledcWrite(PIN_PULSE_OUTPUT, duty);
        // DEBUG
        // Serial.printf("Calculated duty: %d\n\r", duty);
    }
    
}

uint32_t calculateDuty(uint32_t freq, uint32_t pulseWidth_ns)
{
    uint64_t period_ns = (1000000000ULL / (uint64_t) freq); 
    uint64_t duty = ((uint64_t) pulseWidth_ns * (LEDC_BITWIDTH_MAX_VAL) )/ period_ns;
    return (uint32_t) duty;
}
