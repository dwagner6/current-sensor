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
#define VOUT_MV_MAX 20000
#define LEDC_CHAN 0

// TODO: Find minimum timeout necessary for Labview to send commands
// in order to speed up main loop
#define SERIAL_TIMEOUT_MS 1000 // Timeout when waiting for serial data

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
enum   {NONE=-1, FREQ_SET, WIDTH_SET, CURR_SET, ONOFF_SET, \
        VOUT_SET, STATUS_SET, ADC_GET, RAW_ADC_GET, \
        GET_ALL_SETTINGS, SEND_ALL_SETTINGS, SEND_ALL_READINGS};

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
uint32_t settings_reg[5] = {DEFAULT_FREQ_HZ, DEFAULT_PULSEWIDTH, \
                            0, 0, VOUT_MV_MIN};
extern current_range_t currentRange;
extern uint32_t current_mA[4];
extern uint32_t current_mV[4];
extern uint32_t opt_pwr_mV;
String value_str;

uint64_t lastAdcReadout = 0;
uint64_t currentAdcReadout;

uint32_t calculateDuty(uint32_t freq, uint32_t pulseWidth_ns);
void int_to_hex_str(uint8_t num_digits, uint32_t value, char * hex_string);
void setFreq (uint16_t freq_hz);
void setWidth (uint16_t width_ns);
void turnOnOff(uint8_t on_or_off);
void setVout(uint16_t vout_mv);
void sendAllReadings();
void sendAllSettings();
void getAndSetAllSettings(String * str);

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
        value_str = String(input_str.substring( (space_loc), (input_len) ));
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
            setting = SEND_ALL_SETTINGS;
        }
        else if (command_str.equals("get_readings"))
        {
            setting = SEND_ALL_READINGS;
        }
        else if (command_str.equals("set_settings"))
        {
            setting = GET_ALL_SETTINGS;
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
        if(value_str.equals("?") || (setting == SEND_ALL_SETTINGS) \
                                 || (setting == SEND_ALL_READINGS) )
        {
            is_reading = true;
        }
        // Else we are actually setting something, so record the value
        else if(setting != GET_ALL_SETTINGS)
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
        setFreq((uint16_t) freq);
        break;
    
    case WIDTH_SET:
        if(is_reading)
        {
            Serial.println(settings_reg[setting], DEC);
            break;
        }
        pulseWidth = settings_reg[WIDTH_SET];
        setWidth((uint16_t) pulseWidth);
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

        turnOnOff( (uint8_t) settings_reg[ONOFF_SET]);
        break;
       
    case VOUT_SET:
        if(is_reading)
        {
            Serial.println(settings_reg[setting], DEC);
            break;
        }
        vout = (uint16_t) settings_reg[VOUT_SET];
        setVout(vout);
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

    case SEND_ALL_READINGS:
        sendAllReadings();
        break;

    case GET_ALL_SETTINGS:
        getAndSetAllSettings(&value_str);
        break;
    case SEND_ALL_SETTINGS:
        sendAllSettings();
        break;
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

void sendAllReadings()
{
    String adc_readings;

    for(uint8_t i = 0; i < 4; i++)
    {
        adc_readings.concat(String(current_mA[i]));
        adc_readings.concat(" ");
        adc_readings.concat(String(current_mV[i]));
        adc_readings.concat(" ");
    }
    
    // Also get optical power reading
    adc_readings.concat(String(opt_pwr_mV));
    
    Serial.println(adc_readings);
}


void sendAllSettings()
{
    String all_settings;
    for(uint8_t i = 0; i < 5; i++)
    {
        all_settings.concat(settings_reg[i]);
        all_settings.concat(" ");
    }
    all_settings.trim();
    Serial.println(all_settings);

}


void getAndSetAllSettings(String * str)
{
    uint16_t freq_val;
    uint16_t width_val;
    uint16_t curr_val;
    uint8_t onoff_val;
    uint16_t vout_val;

    String freq_str = str->substring(0, str->indexOf(" "));
    freq_val = freq_str.toInt();
    str->remove(0, str->indexOf(" ")+1);

    String width_str = str->substring(0, str->indexOf(" "));
    width_val = width_str.toInt();
    str->remove(0, str->indexOf(" ")+1);

    String curr_str = str->substring(0, str->indexOf(" "));
    curr_val = curr_str.toInt();
    str->remove(0, str->indexOf(" ")+1);

    String onoff_str = str->substring(0, str->indexOf(" "));
    onoff_val = onoff_str.toInt();
    str->remove(0, str->indexOf(" ")+1);

    String vout_str = str->substring(0, str->indexOf(" "));
    vout_val = vout_str.toInt();

    setFreq(freq_val);
    setWidth(width_val);
    setVout(vout_val);
    turnOnOff(onoff_val);
}

void turnOnOff(uint8_t on_or_off)
{
    settings_reg[ONOFF_SET] = on_or_off; 
    if (!settings_reg[ONOFF_SET])
    {
        tps55289_disable_output();
    }
    else
    {
        tps55289_enable_output();
    }
}

void setVout(uint16_t vout_mv)
{
    if(vout_mv < VOUT_MV_MIN)
    {
        vout_mv = VOUT_MV_MIN;
    }
    else if(vout_mv > VOUT_MV_MAX)
    {
        vout_mv = VOUT_MV_MAX;
    }
    settings_reg[VOUT_SET] = (uint16_t) vout_mv;
    vout = vout_mv;
    tps55289_set_vout(vout_mv);
}

void setFreq(uint16_t freq_hz)
{
    if (freq_hz < DEFAULT_MIN_FREQ)
        freq_hz = DEFAULT_MIN_FREQ;
    if (freq_hz > DEFAULT_MAX_FREQ)
        freq_hz = DEFAULT_MAX_FREQ;
    settings_reg[FREQ_SET] = freq_hz;
    freq = freq_hz;
    ledcChangeFrequency(PIN_PULSE_OUTPUT, freq_hz, LEDC_BIT_RESOLUTION);
    uint32_t duty = calculateDuty(freq_hz, pulseWidth);
    ledcWrite(PIN_PULSE_OUTPUT, duty);
}


void setWidth (uint16_t width_ns)
{
    pulseWidth = width_ns;
    if (pulseWidth < DEFAULT_MIN_PULSEWIDTH)
        pulseWidth = DEFAULT_MIN_PULSEWIDTH;
    if (pulseWidth > DEFAULT_MAX_PULSEWIDTH)
        pulseWidth = DEFAULT_MAX_PULSEWIDTH;
    settings_reg[WIDTH_SET] = pulseWidth;

    // For either FREQ_SET or WIDTH_SET case, update ledc
    ledcChangeFrequency(PIN_PULSE_OUTPUT, freq, LEDC_BIT_RESOLUTION);
    uint32_t duty = calculateDuty(freq, pulseWidth);
    ledcWrite(PIN_PULSE_OUTPUT, duty);
}