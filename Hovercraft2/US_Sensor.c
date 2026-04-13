// US sensor .c file

#include "US_Sensor.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>


#ifdef __cplusplus
extern "C" {
#endif

void US_init(US_Sensor *sensor, uint8_t trig_pin, uint8_t echo_pin, uint8_t thrust_fan, uint8_t lift_fan)
{
    // sensor and pwm pins
    sensor->trig_pin = trig_pin;
    sensor->echo_pin = echo_pin;
    sensor->thrust_fan = thrust_fan;
    sensor->lift_fan = lift_fan;

    sensor->distance = 0;
    sensor->pulse_width = 0;
    sensor->min_distance = 20.0;
    sensor->max_distance = 40.0;
    sensor->min_pulse = 411;  // Closest object 14 * 58 cm (distance * 58) time of flight sound (411)
    sensor->max_pulse = 1233; // Furthest object 42 * 58 cm (1233)
    // PWM ranges for fan control
    sensor->min_fan_speed = 0;   // approx 0%
    sensor->max_fan_speed = 255; // 100%

    sensor->thrust_pwm = 0;
    sensor->lift_pwm = 0;

    // Set trig_pin as output and echo_pin as input
    DDRB |= (1 << sensor->trig_pin);  // Set trig_pin as output
    DDRD &= ~(1 << sensor->echo_pin); // Set echo_pin as input
}

void trigger_pulse(US_Sensor *sensor)
{
    PORTB &= ~(1 << sensor->trig_pin); // sets trigger pin to low, this ensure our high pulse will have a rising edge
    _delay_us(2);
    PORTB |= (1 << sensor->trig_pin);  // sets trigger pin to high, to send pulse
    _delay_us(10);                     // if the signal is to short, sensor might ignore it, we add a delay to make sure that does not happen
    PORTB &= ~(1 << sensor->trig_pin); // low
}

uint32_t pulse_length(US_Sensor *sensor)
{
    long duration = 0;
    while (!(PIND & (1 << sensor->echo_pin)))
        ; // isolate PD2
    while (PIND & (1 << sensor->echo_pin))
    {
        duration++; // increments duration which represents how long it takes before the pulse comes back to the echo pin
        _delay_us(1);
        if (duration == 40000)
        { // arbitrary number, for our purposes, 40 000 would be over 6 meters which is not currently necessary,
            return duration;
        }
    }
    return duration;
}

uint32_t getterDistance(uint32_t pulseVal)
{                                // using the duration from pulse_length(), convert that into a distance using a formula
    uint32_t time_us = pulseVal; // 0.5us per tick
    return (uint32_t)(time_us / 29);
} // end of getDistance

// use the code for the LED pwm in the US sensor to dictate how much power to give to the fans
bool set_fanspeed(US_Sensor *sensor)
{ // function that will use distance to linearly increase brightness from 0-255
    bool stopped = false;
    if (sensor->distance <= sensor->min_distance) // edge condition low
    {
        sensor->distance = sensor->min_distance;
        stopped = true;
    }

    if (sensor->distance >= sensor->max_distance) // edege condition high
        sensor->distance = sensor->max_distance;

    sensor->thrust_pwm = sensor->max_fan_speed -
                 (sensor->max_fan_speed - sensor->min_fan_speed) * // linearization over distance range
                     (sensor->distance - sensor->min_distance) /
                     (sensor->max_distance - sensor->min_distance); // basically find the fraction of max that you're at

    // lift pwm
    sensor->lift_pwm = sensor->thrust_pwm * 0.9;

    return stopped;
}

bool control_fans(US_Sensor *sensor)
{
    int avg = 0;
    int values[] = {0,0,0};
    int largest = 0;
    int large_index = 0;
    int smallest = 999;
    int small_index = 0;
    for (int i = 0; i < 3; i++){
        trigger_pulse(sensor);
        sensor->pulse_width = pulse_length(sensor);
        sensor->distance = getterDistance(sensor->pulse_width);
        values[i] = sensor->distance;
        _delay_ms(30);
        if(values[i] < smallest){
            smallest = values[i];
            small_index = i;
        }
        if(values[i] > largest){
            largest = values[i];
            large_index = i;
        }
    }
    for(int i = 0; i < 3; i++)
    {
        if(i == small_index){
            // nothing
        } else if(i == large_index){
            // nothing
        } else {
            avg = values[i];
        }
    }
    _delay_us(5);
    sensor->distance = avg;
    _delay_us(5);
    if (set_fanspeed(sensor)) // true while not zero
    {
        return true; // signal to the driver file that you are stopped and need to look around
    }
    else
    {
        return false;
        // implement a "look around" algorithm --> done in the .ino driver file
    }
}


void searching(US_Sensor *sensor)
{
    int values[] = {0,0,0};
    int avg = 0;
    int largest = 0;
    int large_index = 0;
    int smallest = 999;
    int small_index = 0;
    for (int i = 0; i < 3; i++){
        trigger_pulse(sensor);
        sensor->pulse_width = pulse_length(sensor);
        sensor->distance = getterDistance(sensor->pulse_width);
        values[i] = sensor->distance;
        _delay_ms(30);
        if(values[i] < smallest){
            smallest = values[i];
            small_index = i;
        }
        if(values[i] < largest){
            largest = values[i];
            large_index = i;
        }
    }
    for(int i = 0; i < 3; i++)
    {
        if(i == small_index){
            // nothing
        } else if(i == large_index){
            // nothing
        } else {
            avg = values[i];
        }
    }
    _delay_us(5);
    sensor->distance = avg;
    _delay_us(5);
}

#ifdef __cplusplus
}
#endif
