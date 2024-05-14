#include <Arduino.h>
#include <soc/ledc_reg.h>
#include <driver/ledc.h>
#include <stdint.h> // Include stdint.h for fixed-width data types
#include <Bounce2.h>
#include <Wire.h>
#include <esp32-hal-adc.h>

#include "pins.h"
#include <pulser.h>
#include <tps55289.h>
#include <utils.h>

#define NUM_READINGS (uint8_t)50 // Number of ADC readings to average
#define BAUD_RATE 115200
#define DEBOUNCE_INTERVAL_MS 5
#define INPUT_SIZE 30
#define DELAY 1 // supposedly uS
#define ADC_CONVERSIONS_PER_PIN 5

uint8_t adcPins[] = {PIN_ADC_1A, PIN_ADC_5A, \
                    PIN_ADC_10A, PIN_ADC_20A};
uint8_t adcPinCount = sizeof(adcPins) / sizeof(uint8_t);
volatile bool adcConversionDone = false;
adc_continuous_data_t *adcResult = NULL;

volatile bool ledcFallingEdgeDetected = false;
volatile uint16_t adcValues5A[NUM_READINGS] = {0};
volatile uint8_t adcIndex5A = 0;
volatile int32_t adcSum5A = 0;
volatile uint16_t adcValues1A[NUM_READINGS] = {0};
volatile uint8_t adcIndex1A = 0;
volatile int32_t adcSum1A = 0;
volatile uint16_t adcValues20A[NUM_READINGS] = {0};
volatile uint8_t adcIndex20A = 0;
volatile int32_t adcSum20A = 0;
volatile uint64_t isr_count = 0;
volatile bool isr_indicator_status = LOW;
volatile uint32_t pulseWidth = DEFAULT_PULSEWIDTH;
volatile bool ledcEnabled = true;
volatile bool transmitReading = false;
volatile bool flipbit = false;
uint32_t averageMillivolts1A;
uint32_t averageMillivolts5A;
uint32_t averageMillivolts20A;
hw_timer_t *timer = NULL;
volatile bool timerEnabled = false;
bool sampleinterruptdetected = false;
bool changed = false;
uint32_t freq = DEFAULT_FREQ_HZ;
uint16_t current = 0;
uint16_t vout_mv = 0;
char teststr[INPUT_SIZE + 1];
int8_t i = -1;
uint16_t reg[7] = {DEFAULT_FREQ_HZ, DEFAULT_PULSEWIDTH, \
                    0, 0, 0, 0, 0};
uint16_t num = 666;
String outputstring;

void updatePulseWidth();
void set_ledc_timer();
void int_to_hex_str(uint8_t num_digits, uint32_t value, char * hex_string);

// TODO: Faster ISR?
void IRAM_ATTR onFallingedge()
{ // on the rising edge of the LEDC pulse
    // on rising edge starts another timer alarm
    sampleinterruptdetected = true;
}

void setup()
{
    pinMode(PIN_PULSE_OUTPUT, OUTPUT);
    pinMode(TIMER_PIN, OUTPUT);
    Serial.begin(BAUD_RATE);

    // initialize_pulser();
    //  Attach an interrupt to the falling edge of the LEDC signal
    attachInterrupt(PIN_PULSE_OUTPUT, &onFallingedge, FALLING);

    pinMode(TPS55289_EN_PIN, OUTPUT);
    digitalWrite(TPS55289_EN_PIN, HIGH);

    tps55289_initialize();

    Serial.println("ESP32 Pulser setup passed!");

    Serial.println("\nAvailable commands:");
    Serial.println("\tfreq <Hz> \tset frequency");
    Serial.println("\twidth <ns> \tset pulse width");
    Serial.println("\tcurr <mA> \tset current");
    Serial.println("\tonoff <0/1> \tenable/disable output");
    Serial.println("\tvout <mV> \tset compliance voltage");
    Serial.println("\tstatus <0/1> \tread TPS status regs");
    Serial.println("\tadc <0/1> \tdisplay adc raw readings");
    Serial.println("'<cmd> ?' to view current setting");
}

void loop()
{
    // TODO: test time to run 5 ADC conversions back to back

    if (sampleinterruptdetected)
    {
        sampleinterruptdetected = false;

        digitalWrite(TIMER_PIN, HIGH);

        adcValues1A[adcIndex1A] = analogReadMilliVolts(PIN_ADC_1A);
        adcSum1A = adcSum1A + adcValues1A[adcIndex1A] - adcValues1A[(adcIndex1A + 1) % NUM_READINGS];
        averageMillivolts1A = adcSum1A / NUM_READINGS;
        adcIndex1A = (adcIndex1A + 1) % NUM_READINGS;

        adcValues5A[adcIndex5A] = analogReadMilliVolts(PIN_ADC_5A);
        adcSum5A = adcSum5A + adcValues5A[adcIndex5A] - adcValues5A[(adcIndex5A + 1) % NUM_READINGS];
        averageMillivolts5A = adcSum5A / NUM_READINGS;
        adcIndex5A = (adcIndex5A + 1) % NUM_READINGS;

        adcValues20A[adcIndex20A] = analogReadMilliVolts(PIN_ADC_20A);
        adcSum20A = adcSum20A + adcValues20A[adcIndex20A] - adcValues20A[(adcIndex20A + 1) % NUM_READINGS];
        averageMillivolts20A = adcSum20A / NUM_READINGS;
        adcIndex20A = (adcIndex20A + 1) % NUM_READINGS;
        digitalWrite(TIMER_PIN, LOW);
    }
    else
    {
        // TODO: Main menu FSM
        byte size = Serial.readBytes(teststr, INPUT_SIZE);
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

            i = 99;
            if (tokens.equals("freq"))
            {
                i = 1;
            }
            else if (tokens.equals("width"))
            {
                i = 2;
            }
            else if (tokens.equals("curr"))
            {
                i = 3;
            }
            else if (tokens.equals("onoff"))
            {
                i = 4;
            }
            else if (tokens.equals("vout"))
            {
                i = 5;
            }
            else if (tokens.equals("status"))
            {
                i = 6;
            }
            else if (tokens.equals("adc"))
            {
                i = 7;
            }
            else
                Serial.println("Error!");

            token = strtok(NULL, "     ");
            tokens = token;
            tokens.trim();
            if (tokens.length() > 0)
            {
                if (tokens.equals("?"))
                    Serial.println(reg[i - 1]);
                else
                {
                    num = tokens.toInt();
                    reg[i - 1] = num;
                    Serial.println("OK");
                    changed = true;
                }
            }
        }

        if (changed)
        {
            changed = false;

            freq = reg[0];
            if (freq < DEFAULT_MIN_FREQ)
                freq = DEFAULT_MIN_FREQ;
            if (freq > DEFAULT_MAX_FREQ)
                freq = DEFAULT_MAX_FREQ;
            reg[0] = freq;
            Serial.print("freq: ");
            Serial.println(freq);

            pulseWidth = reg[1];
            if (pulseWidth < DEFAULT_MIN_PULSEWIDTH)
                pulseWidth = DEFAULT_MIN_PULSEWIDTH;
            if (pulseWidth > DEFAULT_MAX_PULSEWIDTH)
                pulseWidth = DEFAULT_MAX_PULSEWIDTH;
            reg[1] = pulseWidth;
            Serial.print("pulseWidth: ");
            Serial.println(pulseWidth);

            current = reg[2];
            if (current < 0)
                current = 0;
            if (current > 20000U)
                current = 20000U;
            reg[2] = current;
            Serial.print("current: ");
            Serial.println(current);

            if (!reg[3])
            {
                pulseWidth = 0;
                tps55289_disable_output();
            }
            else
                tps55289_enable_output();
            Serial.print("onoff: ");
            Serial.println(reg[3]);

            if (reg[4] > 15000U)
                reg[4] = 15000U;
            if (reg[4] < 0)
                reg[4] = 0;
            vout_mv = reg[4];
            tps55289_set_vout(vout_mv);
            Serial.print("vout_mv: ");
            Serial.println(vout_mv);

            if (reg[5])
            {
                tps55289_status_report();
                reg[5] = 0;
            }
            
            if (reg[6])
            {
                Serial.print("ADC 5A:   ");
                Serial.print(averageMillivolts5A, DEC);
                Serial.println();

                Serial.print("ADC 20A: ");
                Serial.print(averageMillivolts20A, DEC);
                Serial.println();
                reg[6] = 0;
            }

            set_ledc_timer();   // set new ledc frequence
            updatePulseWidth(); // set new pulsewidth
        }
    }
}

void updatePulseWidth()
{
    uint64_t duty = (pulseWidth * UINT16_MAX) / (MAX_PULSE_WIDTH);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
    if (ledcEnabled)
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

void set_ledc_timer()
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_16_BIT, // 16-bit duty resolution
        .timer_num = LEDC_TIMER_0,
        .freq_hz = freq};
    ledc_timer_config(&ledc_timer);

    // Configure LEDC channel with a fixed duty cycle (e.g., 50%) for a 1ms period
    ledc_channel_config_t ledc_channel = {
        .gpio_num = PIN_PULSE_OUTPUT,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0};
    ledc_channel_config(&ledc_channel);
    updatePulseWidth();

    // Attach an interrupt to the rising edge of the LEDC signal
    attachInterrupt(PIN_PULSE_OUTPUT, &onFallingedge, FALLING);
}
