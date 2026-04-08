// main .ino file to run the hovercraft
#ifndef F_CPU
#define F_CPU 16000000UL // UL = unsigned long
#endif
// the brightness needs to be reversed and the LED light dosent flash
#define baud_val 9600UL
#define UBRR ((F_CPU) / ((baud_val) * (16UL)) - 1) // 104 datasheet

// define pin numbers here
// Pins to define
#define USPin
#define IRPin
#define thrust_fan
#define lift_fan
#define servo_pin

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

#include "US_Sensor.h"

typedef struct
{
    US_Sensor us_sensor = us_sensor->US_init(&us_sensor, PB0, PD2, PB2, PB1);
    IMU_Data imu_data;
    IRSensor ir_sensor;

} Hovercraft;

ISR(TIMER0_COMPA_vect)
{
    system_millis++;
}

uint32_t millis()
{
    return system_millis;
}

void hvc_init(Hovercraft *hvc)
{
}

void uart_init()
{
    // setup baud rate (9600)
    UBRR0H = 0;
    UBRR0L = (F_CPU / (16 * baud_val)) - 1; // ubrr is a 16-bit register, val instde is near 104 (0x0068) so ubbr high is 0
    // Enable uart communication through register B
    UCSR0B = (1 << TXEN0);

    // Set frame format: 8 data bits (Z02, Z01, Z00 are 011 for 8 bits data)
    UCSR0C = (0 << UCSZ02) | (1 << UCSZ01) | (1 << UCSZ00); // format is called 8N1

    // setup ADC controls
    ADMUX = (1 << REFS0);                                  // ADC scaling (reading = Vin/5V x 10), also sets ADC0 as the default adc input channel
    ADCSRA = (1 << ADEN)                                   // adc enabled
             | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler 2^8 = 128, datasheet says adc clock best between 200khz and 50khz, ~125khz here

    // Example Pin setups...
    DDRB |= (1 << mainLED); // D11 output (PWM LED), OC2A
    DDRB |= (1 << yloLED);  // D13 output (Yellow LED)
    DDRC &= ~(1 << IRPin);  // ADC0 input analog

    // setup PWM
    TCCR2A = (1 << COM2A1) | (1 << COM2A0)  // use inverting pwm since arduino nano is active low and inverts the signal
             | (1 << WGM20) | (1 << WGM21); // simple pwm channel (mode 3), 8-bits, counts up only, super simple
    TCCR2B = (1 << CS20) | (1 << CS21);     // don't need insanely high pwm clock rate for a LED
}

uint16_t ADC_read(void)
{                          // 10-bit resolution fits in uint16_t
    ADCSRA |= (1 << ADSC); // flip the adsc bit to start conversion
    while (ADCSRA & (1 << ADSC))
        ; // conversion ongoing
    return ADC;
}

void timer0_init()
{
    // CTC mode, prescaler 64
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 249;            // 16MHz/64/250 = 1ms interrupt
    TIMSK0 = (1 << OCIE0A); // enable compare match interrupt
}
// timer0 interrupt fires every 1ms
ISR(TIMER0_COMPA_vect)
{
    system_millis++; // just increment a counter
}

int main(void)
{
    // setup all in this func
    uart_init();

    while (1)
    {
        reading = 0;
        // get multiple data points since the sensor is suceptible to noise
        for (int i = 0; i < ADC_sample_max; i++)
        {
            reading += ADC_read();
        }
        reading = reading / ADC_sample_max;

        // account for false readings below 20, this uses the logic from SharpIR.cpp in the SharpIR ghithub library at https://github.com/qub1750ul/Arduino_SharpIR/blob/master/src/SharpIR.cpp
        // Sensor is GP2Y0A21YK0F, added a layer of protection in case of a very low reading bug
        if (reading > 20)
        {
            distance = 4800.0 / (reading - 20);
        }
        else
        {
            distance = d2; // force out-of-bounds high, low reading means farther away as per the data sheet
        }

        // account for the edge cases
        if (distance >= d2)
        {
            distance = d2;
            edges_indicator = true; // flash yellow led
        }
        else if (distance <= d1)
        {
            distance = d1;
            edges_indicator = true; // flash yellow led
        }

        // scale the brightness linearly from 14cm to 42cm
        pwm = 255 * ((distance - d1) / (d2 - d1));
        // if the brightness ever inverts for some reason, add failsafe to make sure brightness never goes past edges, as brightness is uint8_t
        // ex: brightness = -4 --> becomes brightness = 252

        // control Fans pwm
        OCR2A = pwm;
    }
}
