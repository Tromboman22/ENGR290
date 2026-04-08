// main .ino file to run the hovercraft
#ifndef F_CPU
#define F_CPU 16000000UL // UL = unsigned long
#endif
// the brightness needs to be reversed and the LED light dosent flash
#define baud_val 9600UL
#define UBRR ((F_CPU) / ((baud_val) * (16UL)) - 1) // 104 datasheet

// define pin numbers here
// Pins to define
#define USPin_trig PB3
#define USPin_echo PD2
#define IRPin PB0
#define IMUPin
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
#include "IMU.h"
#include "IR_Sensor.h"

typedef struct
{
    US_Sensor us_sensor;
    IMU_Data imu_data;
    IRSensor ir_sensor;

} Hovercraft;

Hovercraft hvc;

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
    US_init(&hvc->us_sensor, USPin_trig, USPin_echo, thrust_fan, lift_fan);
    IMU_Data_init(&hvc->imu_data, servo_pin, IMUPin);
    // IRSensor_init(&hvc->ir_sensor, IRPin, ADC_sample_max);
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
    DDRB |= (1 << thrust_fan);
    DDRB |= (1 << lift_fan);
    DDRB |= (1 << servo_pin);
    DDRC &= ~(1 << IRPin);
    DDRC &= ~(1 << IMUPin);

    // PB3, PD3 → PWM Timer 2
    // PB1, PB2 → PWM Timer 1 (Servo)
    // PD5, PD6 → PWM Timer 0 (fans?)

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

void setup()
{
    uart_init();
    hvc_init(&hvc);
    hvc.imu_data.setup(&hvc.imu_data);
}

void loop()
{
    if (hvc.us_sensor.control_fans(&hvc.us_sensor))
    {
        // look around
    }

    hvc.imu_data.IMU_calcs(&hvc.imu_data, 0);
}