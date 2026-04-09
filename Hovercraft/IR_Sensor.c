#include "IR_Sensor.h"

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <avr/io.h>

void IR_sensor_initialize(IRSensor *sensor, uint8_t irpin)
{
    // ADC reference voltage = AVcc
    ADMUX = (1 << REFS0); 

    DDRC &= ~(1 << irpin);

    // Enable ADC, prescaler = 128 (~125kHz ADC clock at 16MHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    sensor->reading = 0;
    sensor->distance = 0;
    sensor->IRPin = irpin; 
}

uint16_t IR_ADC_read(IRSensor *sensor)
{
    // Start ADC conversion on ADC0
    ADCSRA |= (1 << ADSC);

    // Wait until conversion completes
    while (ADCSRA & (1 << ADSC));

    return ADC; // 10-bit result
}

void IR_sensor_update(IRSensor *sensor)
{
    // Take a single reading
    sensor->reading = IR_ADC_read(sensor);

    // Convert ADC reading to distance in cm
    if (sensor->reading > 20)
    {
        sensor->distance = 4800.0f / (sensor->reading - 20);
    }  
}

