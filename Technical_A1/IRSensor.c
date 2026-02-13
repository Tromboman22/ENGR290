#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define F_CPU 16000000UL  // UL is unsigned, 16Mhz is arduino nano clock rate
#define baud_val 9600UL
// define clock freq for delay function
#include <util/delay.h>

// define pins and limits

//Pins but avr
#define IRPin PC0     // A0
#define mainLED PB3   // 11
#define yloLED PB5    // yellow led (13)

//limits and variables
const uint8_t d2 = 42;
const uint8_t d1 = 14;
uint16_t reading; // to contain 10-bit value from ADCSRA reading macro
float distance;
bool edges_indicator = false;
uint8_t brightness;  // 8 bits unisgned

// If you use an IR sensor, you can set it to a reasonable value, something between 4 and 10 should work well.
#define ADC_sample_max 4


// cpu freq is 16Mhz, desired baud rate is 9600, 
// uart baud register rate (ubrr) is 16Mhz(16bits * 9600) - 1 = 103 
void uart_init() {
    // set baud rate to same as Serial(9600)
    UBRR0H = 0;  
    UBRR0L = (F_CPU / (16 * baud_val)) - 1;    // ubrr is a 16-bit register, val instde is near 104 (0x0068) so ubbr high is 0
    // Enable uart communication through register B
    UCSR0B = (1 << TXEN0);  

    // Set frame format: 8 data bits (Z02, Z01, Z00 are 011 for 8 bits data)
    UCSR0C = (0 << UCSZ02) | (1 << UCSZ01) | (1 << UCSZ00); //format is called 8N1

    // set the pins using the atmel layout
    DDRB |= (1 << mainLED);  // D11 output (PWM LED)
    DDRB |= (1 << yloLED);   // D13 output (Yellow LED)
    DDRC &= ~(1 << IRPin);   // A0 input (ADC)

}

// PWM setup
void PWM_init(void){
    DDRB |= (1 << mainLED); // output

    TCCR2A = (1 << COM2A1) | (1 << WGM20) | (1 << WGM21); // Fast PWM
    TCCR2B = (1 << CS21); // prescaler 8
}


// ADC control
void ADC_init(void){
    ADMUX = (1 << REFS0); // AVcc reference, ADC0 default
    ADCSRA = (1 << ADEN)  // Enable ADC
           | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler 128
}

uint16_t ADC_read(void){  // 10-bit resolution
    ADCSRA |= (1 << ADSC);        // Start conversion
    while (ADCSRA & (1 << ADSC)); // Wait
    return ADC;
}


// Printing with uart (no printf)
void uart_putchar(char c){
    while (!(UCSR0A & (1 << UDRE0))); // Wait until buffer empty
    UDR0 = c;
}

void print_string(const char *str){
    while (*str)
    {
        uart_putchar(*str);
        str++;  // cycke through the string
    }
}

void print_int(int value){
    char buffer[12];
    itoa(value, buffer, 10);  // convert int to string (base 10)
    print_string(buffer);
}


int main(void) {
  // setup all in this func
  uart_init();
  ADC_init();
  PWM_init();

  while(1){
    reading = 0; 
    // get multiple data points since the sensor is suceptible to noise
    for(int i = 0; i < ADC_sample_max; i++){
      reading += ADC_read();
    }
    reading = reading/ADC_sample_max;
    

    // account for false readings below 20, this uses the logic from SharpIR.cpp in the SharpIR ghithub library at https://github.com/qub1750ul/Arduino_SharpIR/blob/master/src/SharpIR.cpp
    // Sensor is GP2Y0A21YK0F, added a layer of protection in case of a very low reading bug
    if (reading > 20) {
      distance = 4800.0 / (reading - 20); 
    } else {
      distance = d2;   // force out-of-bounds high, low reading means farther away as per the data sheet
    }


    // account for the edge cases, keeping sensor 10cm from front of vehicle might prove useful...
    if(distance >= d2){
      distance = d2;
      edges_indicator = true; // flash yellow led
    }
    else if(distance <= d1){
      distance = d1;
      edges_indicator = true; // flash yellow led
    } 

    // scale the brightness linearly from 14cm to 42cm
    brightness = 255 * (1 - (distance - d1)/(d2-d1)); 
    // if the brightness ever inverts for some reason, add failsafe to make sure brightness never goes past edges, as brightness is uint8_t
    // ex: brightness = -4 --> becomes brightness = 252
  
    
    // control LED brightness for main LED
    OCR2A = brightness;

    // show data
    print_string("ADC reading: ");
    print_int(reading);
    print_string(" | Distance: ");
    print_int((int)distance);
    print_string(" cm | Brightness: ");
    print_int((int)brightness);
    uart_putchar('\n');


    // control blinker

    if(!edges_indicator)
    {
      // distance within bounds
      _delay_ms(1000);
    }else{
      // distance out of bounds
      PORTB |= (1 << yloLED); // set to high
      _delay_ms(500);
      PORTB &= ~(1 << yloLED);  // set to low
      _delay_ms(500);
      edges_indicator = false;
    }

  } 

}
