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

//***********************
// Uncomment to turn on debug features
#define DEBUG_ON
//***********************


volatile uint32_t pulseWidth = DEFAULT_PULSEWIDTH;
volatile bool ledcEnabled = true;
volatile bool transmitReading = false;
volatile bool flipbit = false;
enum   {NONE, FREQ_SET, WIDTH_SET, CURR_SET, ONOFF_SET, \
        VOUT_SET, STATUS_SET, ADC_GET, RAW_ADC_GET, \
        GET_ALL_SETTINGS, SET_ALL_SETTINGS, GET_ALL_READINGS};

esp_err_t error = ESP_OK;

hw_timer_t *timer = NULL;
volatile bool timerEnabled = false;
bool settings_changed = false;
uint32_t freq = DEFAULT_FREQ_HZ;
uint8_t adc_chan = 0;
uint16_t vout = 0;
uint16_t current = 0;
uint16_t vout_mv = 0;
int8_t setting = NONE;
uint32_t settings_reg[7] = {DEFAULT_FREQ_HZ, DEFAULT_PULSEWIDTH, \
                            0, 0, 0, 0, 0};
extern current_range_t currentRange;
extern uint32_t current_mA[4];
extern uint32_t current_mV[4];

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
    bool is_reading = false;
    bool valid_command = true;
    uint16_t value_int;
    setting = NONE;

    applicationWorker();

    String input_str; 

    if(Serial.available() > 0)
    {
        input_str = Serial.readString();
        input_str.trim();
    } 

    if (input_str.length() > 3)
    {
        // Find the index of first <Space> character 
        uint8_t space_loc = input_str.indexOf(' ');
        uint8_t input_len = input_str.length();

        // Put anything after the <Space> into its own string
        String value_str(input_str.substring( (space_loc), (input_len) ));
        value_str.trim();

        // Make a command string
        String command_str(input_str.substring(0, (space_loc) ));

        // If there's a value, turn it into an integer
        value_int = value_str.toInt();

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
            setting = ADC_GET;
        }
        else if (command_str.equals("raw_adc"))
        {
            setting = RAW_ADC_GET;
        }
        else if (command_str.equals("get_settings"))
        {
            setting = GET_ALL_SETTINGS;
        }
        else if (command_str.equals("get_readings"))
        {
            setting = GET_ALL_READINGS;
        }
        else if (command_str.equals("set_readings"))
        {
            setting = SET_ALL_SETTINGS;
        }
        else
        {
            Serial.println("Error: no matching command!");
            valid_command = false;
        }

        if(!valid_command)
        {
            Serial.println("ERROR!");
        }
        else
        {
            Serial.println("OK");
        }

        // If we're just reading something
        if(value_str.equals("?") || (setting == GET_ALL_SETTINGS) \
                                 || (setting == GET_ALL_READINGS) )
        {
            is_reading = true;
        }
        // Else we are actually setting something, so record the value
        else
        {
            settings_reg[setting] = value_int;
        }
    }

    uint32_t frequency = 0;
    uint32_t duty = 0;
    uint16_t prevCurrent = 0; 

    switch (setting)
    {

    case FREQ_SET:
        if(is_reading)
        {
            Serial.println(settings_reg[setting], DEC);
            break;
        }
        freq = settings_reg[FREQ_SET];
        if (freq < DEFAULT_MIN_FREQ)
            freq = DEFAULT_MIN_FREQ;
        if (freq > DEFAULT_MAX_FREQ)
            freq = DEFAULT_MAX_FREQ;
        settings_reg[FREQ_SET] = freq;
        frequency = ledcChangeFrequency(PIN_PULSE_OUTPUT, freq, LEDC_BIT_RESOLUTION);
        duty = calculateDuty(frequency, pulseWidth);
        ledcWrite(PIN_PULSE_OUTPUT, duty);
        break;
    
    case WIDTH_SET:
        if(is_reading)
        {
            Serial.println(settings_reg[setting], DEC);
            break;
        }
        pulseWidth = settings_reg[WIDTH_SET];
        if (pulseWidth < DEFAULT_MIN_PULSEWIDTH)
            pulseWidth = DEFAULT_MIN_PULSEWIDTH;
        if (pulseWidth > DEFAULT_MAX_PULSEWIDTH)
            pulseWidth = DEFAULT_MAX_PULSEWIDTH;
        settings_reg[WIDTH_SET] = pulseWidth;

        // For either FREQ_SET or WIDTH_SET case, update ledc
        frequency = ledcChangeFrequency(PIN_PULSE_OUTPUT, freq, LEDC_BIT_RESOLUTION);
        duty = calculateDuty(frequency, pulseWidth);
        ledcWrite(PIN_PULSE_OUTPUT, duty);
        break;

    case CURR_SET:
        if(is_reading)
        {
            Serial.println(settings_reg[setting], DEC);
            break;
        }
        prevCurrent = current;
        current = settings_reg[CURR_SET];
        if (current > 20000U)
            current = 20000U;
        settings_reg[CURR_SET] = current;

        if(prevCurrent != current)
            setCurrentRange(current);
        break;

    case ONOFF_SET:
        if(is_reading)
        {
            Serial.println(settings_reg[setting], DEC);
            break;
        }
        if (!settings_reg[ONOFF_SET])
        {
            //pulseWidth = 0;
            tps55289_disable_output();
            //setCurrent(0);
        }
        else
        {
            //setCurrent(current);
            tps55289_enable_output();
        }
        break;
       
    case VOUT_SET:
        if(is_reading)
        {
            Serial.println(settings_reg[setting], DEC);
            break;
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
        break;

    case ADC_GET:
        if(value_int > 3)
        {
            // Default to adc chan 3 if invalid channel value
            value_int = 3;
        }
        adc_chan = (uint8_t) value_int;
        //adc_chan = settings_reg[ADC_SET];
        Serial.println(current_mA[adc_chan], DEC);
        break;

    case RAW_ADC_GET:
        if(value_int > 3)
        {
            value_int = 3;
        }
        adc_chan = (uint8_t) value_int;
        Serial.println(current_mV[adc_chan], DEC);
        break;

    case GET_ALL_READINGS:
    case GET_ALL_SETTINGS:
    case SET_ALL_SETTINGS:
    case NONE:
    default:
        break;
    }
}

uint32_t calculateDuty(uint32_t freq, uint32_t pulseWidth_ns)
{
    uint64_t period_ns = (1000000000ULL / (uint64_t) freq); 
    uint64_t duty = ((uint64_t) pulseWidth_ns * (LEDC_BITWIDTH_MAX_VAL) )/ period_ns;
    return (uint32_t) duty;
}
