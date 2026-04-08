// #ifndef UART_H_
// #define UART_H_

// #include <stdint.h>

// /**
//  * @brief actually all of this could be in the main .ino file, and it sould be...
//  *
//  */

// typedef struct
// {
//     // US sensor
//     uint8_t trig_pin;
//     uint8_t echo_pin;

//     volatile uint32_t distance;
//     volatile uint32_t pulse_width;
//     const float min_distance;
//     const float max_distance;
//     const uint32_t min_pulse;
//     const uint32_t max_pulse;

//     // fans
//     uint8_t thrust_fan;
//     uint8_t lift_fan;

//     // IMU
//     uint8_t imu;

//     // servo
//     uint8_t servo;

//     // IR sensor
//     uint8_t ir_sensor;
// } uart;

// void uart_init(uart *uart);