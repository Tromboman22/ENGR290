// main .ino file to run the hovercraft
#ifndef F_CPU
#define F_CPU 16000000UL //UL = unsigned long
#endif 
// the brightness needs to be reversed and the LED light dosent flash 
#define BAUD 9600UL
#define UBRR ((F_CPU)/((BAUD)*(16UL))-1) // 104 datasheet 

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

#include "US_Sensor.h"


