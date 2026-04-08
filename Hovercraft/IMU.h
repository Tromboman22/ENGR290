#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>

typedef struct {
    static volatile uint32_t system_millis;

    int16_t ax_raw, ay_raw, az_raw; // raw accelerometer readings
    int16_t gx_raw, gy_raw, gz_raw; // raw gyroscope readings

    float ax, ay, az; // accelerometer in g's
    float gx, gy, gz; // gyroscope in degrees per second

    float ax_offset, ay_offset, az_offset; // accelerometer offsets
    float gx_offset, gy_offset, gz_offset; // gyroscope offsets

    float roll, pitch, yaw;                               // orientation angles in degrees
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;    // gyro calibration offsets
    float accel_offset_x, accel_offset_y, accel_offset_z; // accel calibration offsets
} IMU_Data;

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

void IMU_calcs(IMU_Data *data, int offset);

#endif /* IMU_H_ */