    // main .ino file to run the hovercraft 
    #ifndef F_CPU
    #define F_CPU 16000000UL // UL = unsigned long
    #endif
    // the brightness needs to be reversed and the LED light dosent flash
    #define baud_val 9600UL
    #define UBRR ((F_CPU) / ((baud_val) * (16UL)) - 1) // 104 datasheet

    // define pin numbers here
    // Pins to define


    #include <Arduino.h>
    #include <avr/interrupt.h>
    #include <avr/io.h>
    #include <util/delay.h>
    #include <stdlib.h>
    #include <stdio.h>

    #include "US_Sensor.h"
    #include "IMU.h"
    #include "IR_Sensor.h"
    #define USPin_trig PB3
    #define USPin_echo PD2
    #define irpin PC0
    #define THRUSTfan PD6
    #define LIFTfan PD5
    #define servo_pin PB1

    #ifdef __cplusplus
    extern "C" {
    #endif

    typedef struct
    {
        US_Sensor us_sensor;
        IMU_Data imu_data;
        IRSensor irsensor;

    } Hovercraft;

    Hovercraft hvc;
    int system_millis = 0;

    ISR(TIMER2_COMPA_vect)
    {
        system_millis++;
    }

    uint32_t sys_millis()
    {
        return system_millis;
    }

    void hvc_init(Hovercraft *hvc)
    {
        US_init(&hvc->us_sensor, USPin_trig, USPin_echo, THRUSTfan, LIFTfan);
        IMU_Data_init(&hvc->imu_data);
        IR_sensor_initialize(&hvc->irsensor, irpin);
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
        DDRD |= (1 << THRUSTfan) | (1 << LIFTfan);
        DDRB |= (1 << servo_pin);
        DDRC &= ~(1 << irpin);

        // PB3, PD3 → PWM Timer 2
        // PB1, PB2 → PWM Timer 1 (Servo)
        // PD5, PD6 → PWM Timer 0 (fans?)

        // setup PWM    
        TCCR1A = (1 << COM1A1) | (1 << WGM10); // Fast PWM 8-bit, OCR1A
        TCCR1B = (1 << WGM12) | (1 << CS10); 
        
        //timer 0 for lift, thrust fan
        TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1) | (1 << COM0B1); 
        TCCR0B = (1 << CS00); 
    }
    void timer2_init()
    {
        TCCR2A = (1 << WGM21);           // CTC mode
        TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // prescaler 1024
        OCR2A = 15;                       // 16MHz / 1024 / (15+1) ≈ 1 kHz → 1 ms
        TIMSK2 = (1 << OCIE2A);           // enable compare match interrupt
    }

    uint16_t ADC_read(void)
    {                          // 10-bit resolution fits in uint16_t
        ADCSRA |= (1 << ADSC); // flip the adsc bit to start conversion
        while (ADCSRA & (1 << ADSC))
            ; // conversion ongoing
        return ADC;
    }


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

    int offset;
    void setup()
    {
        
        uart_init();
        hvc_init(&hvc);
        setupimu(&hvc.imu_data);
        offset = -90;
        timer2_init();

    }
    void loop()
    {
        int control = 0;
        if (control_fans(&hvc.us_sensor))
        {
            control = 100;
            // look around
            
        }
        OCR0A = hvc.us_sensor.thrust_pwm;
        OCR0B = hvc.us_sensor.lift_pwm;

        if(hvc.us_sensor.thrust_pwm > 0 && hvc.us_sensor.thrust_pwm < 255){
            char buffer[32];
        snprintf(buffer, sizeof(buffer), "Distance: %lu cm\r\n", hvc.us_sensor.distance);
        uart_string(buffer);
        }

        IMU_calcs(&hvc.imu_data, offset, sys_millis());
    }


    #ifdef __cplusplus
    }
    #endif
