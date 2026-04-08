#include "IMU.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE 131.0f

#define GRAVITY 9.81f
#define PI 3.14159265359f
#define CALIBRATION_SAMPLES 1000

#define SERVO_PIN PD6




volatile struct
{
    uint8_t TX_new_data : 1;      // got new data to send
    uint8_t TX_finished : 1;      // done sending
    uint8_t TX_buffer1_empty : 1; // first buffer is empty
    uint8_t TX_buffer2_empty : 1; // second buffer is empty
    uint8_t RX_flag : 3;          // receiving status
    uint8_t TWI_ACK : 1;          // did we get an acknowledge?
} flags;

void IMU_Data_init(IMU_Data *data)
{
    data->ax = data->ay = data->az = 0;
    data->gx = data->gy = data->gz = 0;
    data->ax_offset = data->ay_offset = data->az_offset = 0;
    data->gx_offset = data->gy_offset = data->gz_offset = 0;
    data->roll = data->pitch = data->yaw = 0;
    data->accel_offset_x = data->accel_offset_y = data->accel_offset_z = 0;
    data->gyro_offset_x = data->gyro_offset_y = data->gyro_offset_z = 0;
    data->lastTime = 0;

}

uint8_t TWI_status, TWI_byte; // status codes and data byte

uint8_t TWI_start(uint8_t twi_addr, uint8_t read_write)
{
    TWCR = ((1 << TWINT) | (1 << TWSTA) | (1 << TWEN)); // send START condition
    while (!(TWCR & (1 << TWINT)))
        ; // wait until transmission completed
    if (((TWSR & 0xF8) != TW_START) && ((TWSR & 0xF8) != TW_REP_START))
        return 1; // something went wrong

    twi_addr = ((twi_addr << 1) | (read_write & 1)); // setting r/w bit (0=write,1=read)
    TWDR = twi_addr;                                 // send device address
    TWCR = ((1 << TWINT) | (1 << TWEN));             // reset the flag

    while (!(TWCR & (1 << TWINT)))
        ; // wait until transmission completed and ACK/NACK has been received
    if (((TWSR & 0xF8) != TW_MT_SLA_ACK) && ((TWSR & 0xF8) != TW_MR_SLA_ACK))
        return 2;
    return 0;
}

void TWI_stop()
{
    TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWSTO)); // send stop condition
    while (TWCR & (1 << TWSTO))
        ; // wait until stop condition is executed and bus released
}

uint8_t TWI_write(uint8_t tx_data)
{
    TWDR = tx_data;
    TWCR = ((1 << TWINT) | (1 << TWEN));
    while (!(TWCR & (1 << TWINT)))
        ; // wait until transmission completed
    if ((TWSR & 0xF8) != TW_MT_DATA_ACK)
        return 1; // check TWI Status Register, write failed if no ACK
    return 0;
}

uint8_t TWI_ack_read()
{
    TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWEA)); // Start read cycle with ACK
    while (!(TWCR & (1 << TWINT)))
        ;
    flags.TWI_ACK = 1;
    if ((TWSR & 0xF8) != TW_MR_DATA_ACK)
        flags.TWI_ACK = 0; // check status, read failed if no ACK
    return TWDR;
}

uint8_t TWI_nack_read()
{
    TWCR = ((1 << TWINT) | (1 << TWEN));
    while (!(TWCR & (1 << TWINT)))
        ;
    flags.TWI_ACK = 1;
    if ((TWSR & 0xF8) != TW_MR_DATA_NACK)
    {
        flags.TWI_ACK = 0;
        return 0;
    } // read failed
    return TWDR;
}
// reads 1 register
uint8_t Read_Reg(uint8_t TWI_addr, uint8_t reg_addr, int16_t *data)
{
    TWI_status = TWI_start(TWI_addr, 0); // TW_WRITE
    if (TWI_status)
        return 1;

    TWI_status = TWI_write(reg_addr); // which register to read
    if (TWI_status)
        return 2;

    TWI_status = TWI_start(TWI_addr, 1); // TW_READ (repeated start)
}
// reads multiple registers
uint8_t Read_Reg_N(uint8_t TWI_addr, uint8_t reg_addr, uint8_t bytes, int16_t *data)
{
    uint8_t *p_data = (uint8_t *)data;
    TWI_status = TWI_start(TWI_addr, 0); // TW_WRITE
    if (TWI_status)
        return 1;

    TWI_status = TWI_write(reg_addr); // specify which register we want
    if (TWI_status)
        return 2;

    TWI_status = TWI_start(TWI_addr, 1); // TW_READ (repeated start)
    if (TWI_status)
        return 3;

    for (uint8_t i = 0; i < bytes - 1; i++)
    {
        *p_data = TWI_ack_read(); // read with ACK (more bytes coming)
        if (!flags.TWI_ACK)
            return 5;
        p_data++;
    }
    *p_data = TWI_nack_read(); // read last byte with NACK
    TWI_stop();
}

uint8_t Write_Reg(uint8_t TWI_addr, uint8_t reg_addr, uint8_t value)
{
    TWI_status = TWI_start(TWI_addr, 0); // TW_WRITE
    if (TWI_status)
        return 1;

    TWI_status = TWI_write(reg_addr); // which register to write to
    if (TWI_status)
        return 2;

    TWI_status = TWI_write(value); // write the actual value
    if (TWI_status)
        return 3;

    TWI_stop();
    if (!flags.TWI_ACK)
        return 4;

    return 0;
}

void I2C_begin()
{
    // Initialize TWI hardware
    TWSR = 0;  // Prescaler = 1 (no division)
    TWBR = 72; // Bit rate = 100kHz (for 16MHz clock) - slower is more reliable

    // Initialize flags to known states
    flags.TX_new_data = 0;
    flags.TX_finished = 0;
    flags.TX_buffer1_empty = 1;
    flags.TX_buffer2_empty = 1;
    flags.RX_flag = 0;
    flags.TWI_ACK = 0;
    TWI_status = 0;
    TWI_byte = 0;
}

void imu_initialize()
{
    // Reset MPU6050 - clears any weird states it might be in
    Write_Reg(MPU6050_ADDR, 0x6B, 0x80); // PWR_MGMT_1 reset bit
    _delay_ms(100);                      // give it time to reboot

    // Wake up MPU6050 and set clock source to X gyro (more stable)
    Write_Reg(MPU6050_ADDR, 0x6B, 0x01); // PWR_MGMT_1 wake up
    _delay_ms(10);

    // Configure accelerometer for ±2g range (good balance of sensitivity and range)
    Write_Reg(MPU6050_ADDR, 0x1C, 0x00); // ACCEL_CONFIG

    // Configure gyroscope for ±250°/s range (sensitive enough for our needs)
    Write_Reg(MPU6050_ADDR, 0x1B, 0x00); // GYRO_CONFIG

    // Set sample rate divider to get about 50Hz readings
    Write_Reg(MPU6050_ADDR, 0x19, 0x13); // SMPLRT_DIV (19 = 50Hz sample rate)
    
}

void imu_getMotion6(int16_t *ax, int16_t *ay, int16_t *az,
                    int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buffer[14];

    // Read 14 bytes starting from ACCEL_XOUT_H register

    if (Read_Reg_N(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 14, (int16_t *)buffer) == 0)
    {
        // The data comes in big-endian format (high byte then low byte)
        // so we need to shift and combine them properly

        // Accelerometer data (6 bytes total)
        *ax = (int16_t)((buffer[0] << 8) | buffer[1]); // ACCEL_XOUT_H/L
        *ay = (int16_t)((buffer[2] << 8) | buffer[3]); // ACCEL_YOUT_H/L
        *az = (int16_t)((buffer[4] << 8) | buffer[5]); // ACCEL_ZOUT_H/L

        // Skip temperature data (buffer[6] and buffer[7]) - not needed

        // Gyroscope data (6 bytes total)
        *gx = (int16_t)((buffer[8] << 8) | buffer[9]);   // GYRO_XOUT_H/L
        *gy = (int16_t)((buffer[10] << 8) | buffer[11]); // GYRO_YOUT_H/L
        *gz = (int16_t)((buffer[12] << 8) | buffer[13]); // GYRO_ZOUT_H/L
    }
}

void timer1_servo_init()
{
    TCCR1B = (1 << WGM12) | (1 << CS11); // CTC mode, prescaler 8
    OCR1A = 20000;                       // 20ms period (50Hz)
    TIMSK1 = (1 << OCIE1A);              // Enable compare match interrupt

    DDRB |= (1 << SERVO_PIN);   // Set servo pin as output
    PORTB &= ~(1 << SERVO_PIN); // Start with pin low
}

ISR(TIMER1_COMPA_vect)
{
    // End of pulse - set pin low
    PORTB &= ~(1 << SERVO_PIN);
    TIMSK1 &= ~(1 << OCIE1A); // disable interrupt until next pulse
}

void servoMotor_attach(int pin)
{
    // pin parameter is here for compatibility but we ignore it
    timer1_servo_init();
}

void servoMotor_write(int angle)
{
    // Convert angle to pulse width in microseconds
    // 0° = 600µs, 90° = 1500µs, 180° = 2400µs
    uint16_t pulse = 600 + (angle * 10); // each degree adds 10µs

    // Start the pulse (pin high)
    PORTB |= (1 << SERVO_PIN);
    OCR1A = pulse * 2;       // convert to timer counts (2 counts per µs)
    TCNT1 = 0;               // reset timer counter
    TIMSK1 |= (1 << OCIE1A); // enable the interrupt that will turn it off
}

// this figures out the sensor offsets when the device is stationary
void mpu_calibrate(IMU_Data *data)
{
    // sums for averaging
    long sum_gx = 0;
    long sum_gy = 0;
    long sum_gz = 0;

    long sum_ax = 0;
    long sum_ay = 0;
    long sum_az = 0;

    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    _delay_ms(200); // give the sensor time to settle down, just a little...

    // take lots of samples and average them
    for (int i = 0; i < CALIBRATION_SAMPLES + 100; i++)
    {
        imu_getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

        if (i >= 100) // discard first 100 samples (sensor might be warming up)
        {
            sum_ax += ax_raw;
            sum_ay += ay_raw;
            sum_az += az_raw;

            sum_gx += gx_raw;
            sum_gy += gy_raw;
            sum_gz += gz_raw;
        }
        // optionnal delay
        // _delay_ms(2);
    }

    // calculate offsets
    data->gyro_offset_x = sum_gx / CALIBRATION_SAMPLES;
    data->gyro_offset_y = sum_gy / CALIBRATION_SAMPLES;
    data->gyro_offset_z = sum_gz / CALIBRATION_SAMPLES;

    data->accel_offset_x = sum_ax / CALIBRATION_SAMPLES;
    data->accel_offset_y = sum_ay / CALIBRATION_SAMPLES;
    data->accel_offset_z = sum_az / CALIBRATION_SAMPLES;
}

void setupimu(IMU_Data *data)
{
    I2C_begin();     // start I2C for the IMU
    imu_initialize(); // configure the IMU

    servoMotor_attach(SERVO_PIN); // get the servo ready

    mpu_calibrate(data); // find the sensor offsets
}

void IMU_calcs(IMU_Data *data, int offset, float system_millis)
{
    imu_getMotion6(&data->ax_raw, &data->ay_raw, &data->az_raw,
                   &data->gx_raw, &data->gy_raw, &data->gz_raw);

    float ax = (data->ax_raw - data->accel_offset_x) / ACCEL_SCALE;
    float ay = (data->ay_raw - data->accel_offset_y) / ACCEL_SCALE;
    float az = (data->az_raw - data->accel_offset_z) / ACCEL_SCALE;
    float ax_raw, ay_raw, az_raw;
    float gx = (data->gx_raw - data->gyro_offset_x) / GYRO_SCALE;
    float gy = (data->gy_raw - data->gyro_offset_y) / GYRO_SCALE;
    float gz = (data->gz_raw - data->gyro_offset_z) / GYRO_SCALE;

    data->roll = atan2(ay, az) * 180 / PI;
    data->pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

    ax = (data->ax_raw - data->accel_offset_x) / ACCEL_SCALE;
    ay = (data->ay_raw - data->accel_offset_y) / ACCEL_SCALE;
    az = (data->az_raw - data->accel_offset_z) / ACCEL_SCALE;





    unsigned long now = system_millis;
    float dt = (now - data->lastTime) / 1000.0;
    data->lastTime = now;

    data->yaw += gz * dt;

    // make the servo follow the yaw angle, but keep it safe
    float servoYaw = data->yaw + offset;
    int outOfRange = 0; // flag for LED L

    if (servoYaw > 85)
    {
        servoYaw = 85; // clamp to max
    }

    if (servoYaw < -85)
    {
        servoYaw = -85; // clamp to min
    }

    int servoAngle = servoYaw + 90; // convert -85/85 to 5/175 for servo
    servoMotor_write(servoAngle);
}


#ifdef __cplusplus
}
#endif