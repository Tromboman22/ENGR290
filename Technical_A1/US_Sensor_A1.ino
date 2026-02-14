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


//ultrasonic initialized
void US_init(){
  DDRB |= (1 << TRIG_PIN); // Set TRIG_PIN as output
  DDRD &= ~(1 << ECHO_PIN); // Set ECHO_PIN as input 
} //end of US_init

//initialize uart
void uart_init(){
 UBRR0H = (uint8_t)((UBRR)>>8);
 UBRR0L = (uint8_t)UBRR;
 UCSR0B |= (1 << TXEN0); //Only TX enabled
 UCSR0C = (3 << UCSZ00); //8bit 1 stop no parity (look at datasheet) 8N1
}//end of uart_init

//UART transmit f(x)
void uart_send_reading(char data){
  while (!(UCSR0A & (1 << UDRE0))); //buffer is empty then sends
    UDR0 = data; 
}//end of uart send reading

void uart_string(const char* str) { 
	while (*str) { // while string not empty
		uart_send_reading(*str++); //send string to transmission fx UART
	}
}//end of uart string

//Trigger a pulse through PB5 Trigger Pin
void trigger_pulse(){
    PORTB &= ~(1 << TRIG_PIN); //trigger is low
	  _delay_us(2);
    //send pulse
    PORTB |= (1 << TRIG_PIN); //high
    _delay_us(10); 
    PORTB &= ~(1 << TRIG_PIN); //low
}//end of trigger pulse

//Determine the Pulse Length / duration in microsecond (us)s
uint32_t pulse_length(){ 
	long duration = 0;
	while (!(PIND & (1 << ECHO_PIN))); //isolate PD2
	while (PIND & (1 << ECHO_PIN)) {
		duration++;
		_delay_us(1);    
    if(duration == 40000){ //duration of distance 6.86m max
      return duration;
    }
    /* Self Note:
      We note that the duration is affected by distance directly
      therefore we can say a long enough distance return pulse to be:
      Speed of Sound: 343 m/s
      if let's say we have 40000 duration, with each duration being us
      that is 343 m/s * 0.02s = 13.72 m distance that would be covered in both directions
    */ 
	}
	return duration;
} //end of pulse_length

//Get distance, return distance in (cm)
uint32_t getterDistance(uint32_t pulseVal){
  uint32_t time_us = pulseVal; // 0.5us per tick
  return (uint32_t)(time_us / 29);
} //end of getDistance


//PWM Setup
void pwm_init(void) {
    DDRB |= (1 << LED_PIN);  //LED pin as output
    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS22);    //Prescale to 64 -> 16Mhz/64 -> 250Khz
}

void set_pwm(uint8_t duty_cycle) {
    OCR2A = duty_cycle; //for fast pwm mode 8bit
    //Check TCNT2
    //if 128 -> duty cycle 50%
    /*
      Near LED -> higher ADC -> brightness increase 
    */
}

//LED control/brightness CHECK THIS 
void L_led_control(float distance) {

    if (distance < min_distance || distance > max_distance) {
      _delay_ms(200);
      PORTB ^= (1 << LED_L); 
      //_delay_ms(500);       

    } else {
        PORTB &= ~(1 << LED_L);
    }
}

int calculate_brightness(float distance) {

    if (distance <= min_distance)
        return max_brightness;

    if (distance >= max_distance)
        return min_brightness;

    return max_brightness -
           (max_brightness - min_brightness) *
           (distance - min_distance) /
           (max_distance - min_distance);
}




/* ==MAIN CODE== */
int main(void) {
  US_init();
  uart_init();
  pwm_init();
  
    char *start = "Starting...\r\n";
      uart_string(start);
  //Init L LED pin
  DDRB |= (1 << LED_L); //init with data direciton reg... output
  char buffer[32];
  //uint32_t p_len;
  //int count = 0;
  
    while(1){
      
         //_delay_ms(100);
          trigger_pulse();
              pulse_width = pulse_length();
              // Wait for echo to complete (falling edge sets capturing = 0)
              uint32_t distance = getterDistance(pulse_width);
              
              //p_len = (float)(pulse_width/ (1000.0f));
        //Calculate & setting PWM brightness
        int brightness = calculate_brightness(distance);
        set_pwm(brightness);
        //Control L LED blink
        L_led_control(distance);

              //snprintf(buffer, sizeof(buffer), "Pulse width (in ms): %.2f\r\n", p_len);
              //uart_string(buffer);
              snprintf(buffer, sizeof(buffer), "Pulse width (in us): %u\r\n", pulse_width);
              uart_string(buffer);
              snprintf(buffer, sizeof(buffer), "Distance: %lu cm\r\n", distance);
              uart_string(buffer);
   
              _delay_ms(500);
   
        //count++;
        
    }
    return 0;
}
