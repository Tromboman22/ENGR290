#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>

typedef struct {
    uint8_t MPU6050_ADDR;
    uint8_t MPU6050_PWR_MGMT_1;
    uint8_t MPU6050_ACCEL_XOUT_H;

    float ACCEL_SCALE;
    float GYRO_SCALE;

    float GRAVITY;
    float PI;
    int CALIBRATION_SAMPLES;

    uint8_t SERVO_PIN;

} IMU_Data;

typedef struct {
    uint8_t TX_new_data;      // got new data to send
    uint8_t TX_finishe1;      // done sending
    uint8_t TX_buffer1_empty; // first buffer is empty
    uint8_t TX_buffer2_empty; // second buffer is empty
    uint8_t RX_flag;           // receiving status
    uint8_t TWI_ACK;          // did we get an acknowledge?
} flags;

void IMU_Data_init(IMU_Data *data);
uint8_t TWI_start(uint8_t twi_addr, uint8_t read_write);
void inline TWI_stop();
uint8_t TWI_write(uint8_t tx_data);
uint8_t TWI_ack_read();
uint8_t TWI_nack_read();
uint8_t Read_Reg(uint8_t TWI_addr, uint8_t reg_addr, int16_t* data);
uint8_t Read_Reg_N(uint8_t TWI_addr, uint8_t reg_addr, uint8_t bytes, int16_t* data);
uint8_t Write_Reg(uint8_t TWI_addr, uint8_t reg_addr, uint8_t value);
void imu_initialize();
void imu_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, 
                    int16_t* gx, int16_t* gy, int16_t* gz);
void timer1_servo_init();
void servoMotor_attach(int pin);
void servoMotor_write(int angle);
ISR(TIMER1_COMPA_vect);

void I2C_begin();

void mpu_calibrate(IMU_Data *data);

bool IMU_calcs(IMU_Data *data, int offset);

#endif /* IMU_H_ */