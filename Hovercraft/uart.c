
// #ifndef F_CPU
// #define F_CPU 16000000UL // UL = unsigned long
// #endif

// #include "uart.h"

// #include <avr/interrupt.h>
// #include <avr/io.h>
// #include <util/delay.h>
// #include <stdlib.h>
// #include <stdio.h>

// void uart_init(uart *uart)
// {
// #define BAUD 9600UL
// #define UBRR ((F_CPU) / ((BAUD) * (16UL)) - 1) // 104 datasheet

//     // setup baud rate (9600)
//     UBRR0H = (uint8_t)((UBRR) >> 8);
//     UBRR0L = (uint8_t)UBRR;
//     UCSR0B |= (1 << TXEN0); // Only TX enabled
//     UCSR0C = (3 << UCSZ00); // 8bit 1 stop no parity (look at datasheet) 8N1
// }