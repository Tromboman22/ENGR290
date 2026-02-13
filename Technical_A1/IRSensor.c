#include <stdio.h>
#include <math.h>  // for math operations used for distance

#define F_CPU 16000000UL  // UL is unsigned long, 16k is Arduino Nano cpu frequency
#define BAUD 9600         // Desired Baud rate
uint16_t baud_val = UART_BAUD_SELECT(BAUD, F_CPU);  // baud select macro from avr-uart library


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

// define pins and limits

//Pins
#define IRPin A0
#define mainLED 11  // pwm led pin
#define yloLED 13     // yellow led

//limits and variables
const float d2 = 42;
const float d1 = 14;
int reading;
float distance;
bool edges_indicator = false;
uint8_t brightness, ADC0;  // 8 bits unisgned

// If you use an IR sensor, you can set it to a reasonable value, something between 4 and 10 should work well.
#define ADC_sample_max 4

volatile uint8_t RX_buff, ADC_sample;
volatile uint16_t time, delay_ms, ADC_acc; 

void init() {
    // set baud rate to same as Serial(9600)
    UBRR0 = baud_val;  // uart baud rate registers getting their baud rate set
                       // ubrr is a 16-bit register, tbut baud_val is near 103 so we dont need to bit shift anything

    // Enable receiver and transmitter through register B
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);  

    // Set frame format: 8 data bits, 1 stop bit (parity is disabled by default)
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// no arduino libs

int main(void) {
  // setup
  init();


  while(TRUE){
    // loop
    
    }
}












// using arduino libraries:

void setup() {
  Serial.begin(9600);
  pinMode(mainLED, OUTPUT);
  pinMode(yloLED, OUTPUT);

  Serial.print("Sensor reading start:\n");
}

void loop() {
  // get data and convert using github equation
  reading = analogRead(IRPin);

  // account for false readings below 20, this uses the logic from SharpIR.cpp in the SharpIR ghithub library at https://github.com/qub1750ul/Arduino_SharpIR/blob/master/src/SharpIR.cpp
  // Sensor is GP2Y0A21YK0F, added a layer of protection in case of a very low reading bug
  if (reading > 20) {
    distance = 4800.0 / (reading - 20);
  } else {
    distance = max_dist + 1;   // force out-of-bounds
  }

  // account for the edge cases, keeping sensor 10cm from front of vehicle might prove useful...
  if(distance > 80){
    distance = 81;
    edges_indicator = true; // flash yellow led
  }
  else if(distance < 10){
    distance = 9;
    edges_indicator = true; // flash yellow led
  } 

  // brightness 
  if(distance < d1) brightness = 0;   // edge cases
  else if(distance > d2) brightness = 255;
  else{     // scale the brightness linearly from 14cm to 42cm
    brightness = 255 * (1 - (distance - d1)/(d2-d1)); 
    // if the brightness ever inverts for some reason, add failsafe to make sure brightness never goes past edges, as brightness is uint8_t
    // ex: brightness = -4 --> becomes brightness = 252
  }
  // note: some sites recommend using a filter to reduce noise in the sensor, ex: distance = prevDistance * 0.5 + newDistance * 0.5;
  //       presumably this prevents sudden jumps in brightness due to noise

  // control LED brightness for main LED
  analogWrite(mainLED, brightness);

  // serial show data
  Serial.print("Analog reading: ");
  Serial.print(reading);
  Serial.print(" | Distance: ");
  Serial.print(distance);
  Serial.print(" cm | Brightness: ");
  Serial.println(brightness);
  Serial.print(" /255");

  // yellow LED control, also delay of 1s between readings
  if(!edges_indicator)
  {
    // distance is within bounds
    delay(1000);
  }else{
    // flash led with T=1s when distance out of bounds
    digitalWrite(yloLED, HIGH);
    delay(500);
    digitalWrite(yloLED, LOW);
    edges_indicator = false;
    delay(500);
  }
}
