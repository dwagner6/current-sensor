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


volatile uint32_t pulseWidth = DEFAULT_PULSEWIDTH;
volatile bool ledcEnabled = true;
volatile bool transmitReading = false;
volatile bool flipbit = false;
enum {FREQ_SET, WIDTH_SET, CURR_SET, ONOFF_SET, \
                VOUT_SET, STATUS_SET, ADC_SET};

esp_err_t error = ESP_OK;

hw_timer_t *timer = NULL;
volatile bool timerEnabled = false;
bool changed = false;
uint32_t freq = DEFAULT_FREQ_HZ;
uint8_t adc_chan = 0;
uint16_t vout = 0;
uint16_t current = 0;
uint16_t vout_mv = 0;
int8_t setting = -1;
uint16_t setting_reg[7] = {DEFAULT_FREQ_HZ, DEFAULT_PULSEWIDTH, \
                    0, 0, 0, 0, 0};
extern current_range_t currentRange;
extern uint32_t current_mA[4];
uint16_t num = 666;
String outputstring;

uint64_t lastAdcReadout = 0;
uint64_t currentAdcReadout;

void updatePulseWidth();
void set_ledc_timer();
void int_to_hex_str(uint8_t num_digits, uint32_t value, char * hex_string);

void setup()
{
    //pinMode(PIN_PULSE_OUTPUT, OUTPUT);
    pinMode(TIMER_PIN, OUTPUT);
    Serial.begin(BAUD_RATE);

    ledcAttach(PIN_PULSE_OUTPUT, DEFAULT_FREQ_HZ, 16);    

    pinMode(TPS55289_EN_PIN, OUTPUT);
    digitalWrite(TPS55289_EN_PIN, HIGH);

    tps55289_initialize();
    adcSetup();
    
/*     Serial.println("ESP32 Pulser setup passed!");
    Serial.println("\nAvailable commands:");
    Serial.println("\tfreq <Hz> \tset frequency");
    Serial.println("\twidth <ns> \tset pulse width");
    Serial.println("\tvout <mV> \tset capacitor output voltage");
    Serial.println("\tonoff <0/1> \tenable/disable output");
    Serial.println("'<cmd> ?' to view current setting"); */
}

void loop()
{
    applicationWorker();

    char teststr[INPUT_SIZE + 1];
    byte size = 0;
    if(Serial.available() > 0)
    {
        size = Serial.readBytes(teststr, INPUT_SIZE);
    
    } 
    // Add the final 0 to end the C string
    teststr[size] = 0;

    outputstring = teststr;
    String tokens = "";
    if (size != 0)
    {
        //    remove any \r \n whitespace at the end of the String
        char *token = strtok(teststr, "     ");

        tokens = token;
        tokens.trim();
        

        setting = 99;
        if (tokens.equals("freq"))
        {
            setting = FREQ_SET;
        }
        else if (tokens.equals("width"))
        {
           setting = WIDTH_SET; 
        }
        else if (tokens.equals("curr"))
        {
            setting = CURR_SET;
        }
        else if (tokens.equals("onoff"))
        {
            setting = ONOFF_SET; 
        }
        else if (tokens.equals("vout"))
        {
            setting = VOUT_SET;
        }
        else if (tokens.equals("status"))
        {
            setting = STATUS_SET;
        }
        else if (tokens.equals("adc"))
        {
            setting = ADC_SET;
        }
        else
            Serial.println("Error!");

        token = strtok(NULL, "     ");
        tokens = token;
        tokens.trim();
        if (tokens.length() > 0)
        {
            if (tokens.equals("?"))
                Serial.println(setting_reg[setting]);
            else
            {
                num = tokens.toInt();
                setting_reg[setting] = num;
                Serial.println("OK");
                changed = true;
            }
        }
    }

    if (changed)
    {
        changed = false;

        freq = setting_reg[FREQ_SET];
        if (freq < DEFAULT_MIN_FREQ)
            freq = DEFAULT_MIN_FREQ;
        if (freq > DEFAULT_MAX_FREQ)
            freq = DEFAULT_MAX_FREQ;
        setting_reg[FREQ_SET] = freq;

        pulseWidth = setting_reg[WIDTH_SET];
        if (pulseWidth < DEFAULT_MIN_PULSEWIDTH)
            pulseWidth = DEFAULT_MIN_PULSEWIDTH;
        if (pulseWidth > DEFAULT_MAX_PULSEWIDTH)
            pulseWidth = DEFAULT_MAX_PULSEWIDTH;
        setting_reg[WIDTH_SET] = pulseWidth;

        uint16_t prevCurrent = current;
        current = setting_reg[CURR_SET];
        if (current > 20000U)
            current = 20000U;
        setting_reg[CURR_SET] = current;

        if(prevCurrent != current)
            setCurrent(current);

        if (!setting_reg[ONOFF_SET])
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
       
        vout = setting_reg[VOUT_SET];
        if(vout < VOUT_MV_MIN)
        {
            vout = VOUT_MV_MIN;
        }
        else if(vout > VOUT_MV_MAX)
        {
            vout = VOUT_MV_MAX;
        }
        tps55289_set_vout(vout);

        //adc_chan = setting_reg[ADC_SET];

        //Serial.println(current_mA[adc_chan], DEC);


        set_ledc_timer();   // set new ledc frequency
        updatePulseWidth(); // set new pulsewidth
    }
    
}

void updatePulseWidth()
{
    uint64_t period_ns = (1000000000ULL / freq); // Period in nanoseconds
    uint64_t duty = (pulseWidth * UINT16_MAX) / period_ns;
    error = ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
    if(error != ESP_OK)
        Serial.println("ledc_set_duty parameter error!");

    if (ledcEnabled)
    {
        error = ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        if(error != ESP_OK)
            Serial.println("ledc_update_duty parameter error!");
    }
}

void set_ledc_timer()
{
    static ledc_timer_config_t ledc_timer;
    ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_16_BIT, // 16-bit duty resolution
        .timer_num = LEDC_TIMER_0,
        .freq_hz = freq};
    error = ledc_timer_config(&ledc_timer);
    if (error != ESP_OK)
        Serial.println("ledc_timer_config error!");

    // Configure LEDC channel with a fixed duty cycle (e.g., 50%) for a 1ms period
    static ledc_channel_config_t ledc_channel;
    ledc_channel = {
        .gpio_num = PIN_PULSE_OUTPUT,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0};
    ledc_channel_config(&ledc_channel);
    updatePulseWidth();

    /* // Attach an interrupt to the rising edge of the LEDC signal
    attachInterrupt(PIN_PULSE_OUTPUT, &onFallingedge, FALLING); */
}


