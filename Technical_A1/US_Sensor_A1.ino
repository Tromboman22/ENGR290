#ifndef F_CPU
#define F_CPU 16000000UL //UL = unsigned long
#endif 
// the brightness needs to be reversed and the LED light dosent flash 
#define BAUD 9600UL
#define UBRR ((F_CPU)/((BAUD)*(16UL))-1) // 104 datasheet 

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
//#include "US_ver2.h"

//P13 on board
#define TRIG_PIN PB5
#define ECHO_PIN PD3

//P6 did not work 


/*Control the LED D3*/
#define LED_PIN PB3             // OC2A = digital pin 15, Controller board = P6
/* Toggle once out of range e.g 12 - 46 cm*/
#define LED_L PB5      // 'L' LED = digital pin 13

/* PIND -> input reg of D reg*/
volatile uint32_t distance = 0;
volatile uint32_t pulse_width = 0;
//Distance thresholds (in cm)
const float min_distance = 14.0;
const float max_distance = 42.0;

const uint32_t min_pulse = 411;     // Closest object 14 * 58 cm (distance * 58) time of flight sound (411)
const uint32_t max_pulse = 1233;    // Furthest object 42 * 58 cm (1233)

//LED brightness range (8bit PWM)
const int min_brightness = 0;   // approx 0%
const int max_brightness = 255;  // 100%


//ultrasonic initialized, this function sets up our pins for our ultrasonic sensor
void US_init(){
  DDRB |= (1 << TRIG_PIN); // Set TRIG_PIN as output
  DDRD &= ~(1 << ECHO_PIN); // Set ECHO_PIN as input 
} //end of US_init
//initialize uart, this function enables serial data transmission bu enabling the CPU to send serial data through the tx pin using the UART hardware
void uart_init(){
 UBRR0H = (uint8_t)((UBRR)>>8);
 UBRR0L = (uint8_t)UBRR;
 UCSR0B |= (1 << TXEN0); //Only TX enabled
 UCSR0C = (3 << UCSZ00); //8bit 1 stop no parity (look at datasheet) 8N1
}//end of uart_init

//this function gives us the ability to send one character at a time to printout
void uart_send_reading(char data){
  while (!(UCSR0A & (1 << UDRE0))); //buffer is empty then sends
    UDR0 = data; 
}//end of uart send reading

//this functon reccursively calls uart_send_reading to send a combination of characters which become a string.
void uart_string(const char* str) { 
	while (*str) { // while string not empty
		uart_send_reading(*str++); //send string to transmission fx UART
	}
}//end of uart string

//Trigger a pulse through PB5 Trigger Pin
void trigger_pulse(){
    PORTB &= ~(1 << TRIG_PIN); //sets trigger pin to low, this ensure our high pulse will have a rising edge
	  _delay_us(2);
    PORTB |= (1 << TRIG_PIN); //sets trigger pin to high, to send pulse
    _delay_us(10); //if the signal is to short, sensor might ignore it, we add a delay to make sure that does not happen
    PORTB &= ~(1 << TRIG_PIN); //low
}//end of trigger pulse

//Determine the Pulse Length / duration in microsecond (us)s
uint32_t pulse_length(){ 
	long duration = 0;
	while (!(PIND & (1 << ECHO_PIN))); //isolate PD2
	while (PIND & (1 << ECHO_PIN)) {
		duration++; //increments duration which represents how long it takes before the pulse comes back to the echo pin
		_delay_us(1);    
    if(duration == 40000){ //arbitrary number, for our purposes, 40 000 would be over 6 meters which is not currently necessary, 
      return duration;
    }
   
	}
	return duration;
} //end of pulse_length

//Get distance, return distance in (cm)
uint32_t getterDistance(uint32_t pulseVal){ //using the duration from pulse_length(), convert that into a distance using a formula 
  uint32_t time_us = pulseVal; // 0.5us per tick
  return (uint32_t)(time_us / 29);
} //end of getDistance


//PWM Setup on the LED_PIN so the micro controller can control the brightness
void pwm_init(void) {
    DDRB |= (1 << LED_PIN);  //LED pin as output
    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS22);    //Prescale to 64 -> 16Mhz/64 -> 250Khz, this slows the clock cycle so the PWM signals happens at a speed that the LED light can respond to
}
//this function will be used to control the brightness of our LED, OCR2A is a 8 bit register, if its 0 the brightness will be 0, at 128 it'll be 50%, and 255 it will be 100%
void set_pwm(uint8_t duty_cycle) { 
    OCR2A = duty_cycle; 
    
}

//LED control/brightness CHECK THIS 
void L_led_control(float distance) { //function to meet blinking condition when sensor is out of range (distance <14 or distance >42)

    if (distance < min_distance || distance > max_distance) {
      _delay_ms(100); //LED and trig share a pin, this delay is to make sure the functions using trig pin do not interfere with our LED blink.
      PORTB ^= (1 << LED_L); //XOR operation (low becomes high becomes low. 

    } else {
        PORTB &= ~(1 << LED_L); //clears bits, we want to drive low if the distance condition is not met
    }
}

int calculate_brightness(float distance) { // function that will use distance to linearly increase brightness from 0-255

    if (distance <= min_distance) //edge condition low
        return max_brightness;

    if (distance >= max_distance) //edege condition high
        return min_brightness;

    return max_brightness -
           (max_brightness - min_brightness) * //linearization from 14-42
           (distance - min_distance) /
           (max_distance - min_distance);
}




/* ==MAIN CODE== */
int main(void) {
	//initializes the arduino using functions defined earlier
  US_init();
  uart_init();
  pwm_init();
  
    char *start = "Starting...\r\n";
      uart_string(start);
  //Init L LED pin
  DDRB |= (1 << LED_L); //init with data direciton reg... output
  char buffer[32];

    while(1){ //will always repeat
          	trigger_pulse();
              pulse_width = pulse_length();
              // Wait for echo to complete (falling edge sets capturing = 0)
              uint32_t distance = getterDistance(pulse_width);
              
              //p_len = (float)(pulse_width/ (1000.0f));
        //Calculate & setting PWM brightness
        int brightness = calculate_brightness(distance); //get brightness value (0-255)
        set_pwm(brightness); //set brightness
        //Control L LED blink
        L_led_control(distance); //checks distance to see if we need to blink our LED

              //snprintf(buffer, sizeof(buffer), "Pulse width (in ms): %.2f\r\n", p_len);
              //uart_string(buffer);
              snprintf(buffer, sizeof(buffer), "Pulse width (in us): %u\r\n", pulse_width); //printing for debugging and checking accuracy.
              uart_string(buffer);
              snprintf(buffer, sizeof(buffer), "Distance: %lu cm\r\n", distance);
              uart_string(buffer);
   
              _delay_ms(500); //slows down the program so we can see our print functions
           
    }
    return 0;
}
