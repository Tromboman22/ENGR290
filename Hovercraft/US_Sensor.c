// US sensor .c file

#include "US_Sensor.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

void US_init(US_Sensor *sensor, uint8_t trig_pin, uint8_t echo_pin, uint8_t thrust_fan, uint8_t lift_fan) {
    // sensor and pwm pins
    sensor->trig_pin = trig_pin;
    sensor->echo_pin = echo_pin;
    sensor->thrust_fan = thrust_fan;
    sensor->lift_fan = lift_fan;

    sensor->distance = 0;
    sensor->pulse_width = 0;
    sensor->min_distance = 14.0;
    sensor->max_distance = 42.0;
    sensor->min_pulse = 411;     // Closest object 14 * 58 cm (distance * 58) time of flight sound (411)
    sensor->max_pulse = 1233;    // Furthest object 42 * 58 cm (1233)
    // PWM ranges for fan control
    const int min_brightness = 0;   // approx 0%
    const int max_brightness = 255;  // 100%
    
    // Set trig_pin as output and echo_pin as input
    DDRB |= (1 << sensor->trig_pin); // Set trig_pin as output
    DDRD &= ~(1 << sensor->echo_pin); // Set echo_pin as input
}

void trigger_pulse();

uint32_t pulse_length();

uint32_t getterDistance(uint32_t pulseVal);
