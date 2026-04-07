#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>

struct {
    uint8_t TX_new_data;      // got new data to send
    uint8_t TX_finishe1;      // done sending
    uint8_t TX_buffer1_empty; // first buffer is empty
    uint8_t TX_buffer2_empty; // second buffer is empty
    uint8_t RX_flag;           // receiving status
    uint8_t TWI_ACK;          // did we get an acknowledge?
} flags;

uint8_t TWI_start(uint8_t twi_addr, uint8_t read_write);
void inline TWI_stop();
uint8_t TWI_write(uint8_t tx_data);
uint8_t TWI_ack_read();
uint8_t TWI_nack_read();
uint8_t Read_Reg(uint8_t TWI_addr, uint8_t reg_addr, int16_t* data);
uint8_t Read_Reg_N(uint8_t TWI_addr, uint8_t reg_addr, uint8_t bytes, int16_t* data);
uint8_t Write_Reg(uint8_t TWI_addr, uint8_t reg_addr, uint8_t value);
void Wire_begin();
void imu_initialize();
void imu_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, 
                    int16_t* gx, int16_t* gy, int16_t* gz);
void timer1_servo_init();
void servoMotor_attach(int pin);
void servoMotor_write(int angle);
ISR(TIMER1_COMPA_vect);

void Wire_begin();

#endif /* IMU_H_ */