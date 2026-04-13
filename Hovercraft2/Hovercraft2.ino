// =============================================================================
// Hovercraft Main Controller
// ATmega328P @ 16MHz
//
// Timer assignments (DO NOT double-assign these):
//   Timer 0 (8-bit)  → Fan PWM: OCR0A = Thrust (PD6), OCR0B = Lift (PD5)
//   Timer 1 (16-bit) → Servo PWM: OCR1A (PB1), Mode 14, 20ms period
//   Timer 2 (8-bit)  → millis() tick via ISR, 1ms interval
// =============================================================================

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "US_Sensor.h"
#include "IR_Sensor.h"
typedef struct {
    US_Sensor us_sensor;
    IRSensor  ir_sensor;
} Hovercraft;

static Hovercraft hvc;

// =============================================================================
// Pin Definitions
// =============================================================================
#define USPin_trig      PB3
#define USPin_echo      PD2
#define IR_PIN          PC0
#define THRUST_FAN_PIN  PD6   // OC0A
#define LIFT_FAN_PIN    PD5   // OC0B
#define SERVO_PIN       PB1   // OC1A

// =============================================================================
// IMU / MPU6050 Definitions
// =============================================================================
#define MPU6050_ADDR         0x68
#define MPU6050_ACCEL_XOUT_H 0x3B

#define ACCEL_SCALE          16384.0f
#define GYRO_SCALE           131.0f
#define PI                   3.14159265359f
#define CALIBRATION_SAMPLES  1000

// =============================================================================
// UART — single implementation, 9600 baud
// =============================================================================
#define BAUD     9600UL
#define UBRR_VAL ((F_CPU / (16UL * BAUD)) - 1)   // = 103 at 16MHz

void uart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)(UBRR_VAL);
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8N1
}

void uart_putchar(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void uart_puts(const char *str)
{
    while (*str) {
        if (*str == '\n') uart_putchar('\r');
        uart_putchar(*str++);
    }
}

void uart_putint(int16_t value)
{
    char buf[8];
    itoa(value, buf, 10);
    uart_puts(buf);
}

void uart_putfloat(float value, uint8_t decimals)
{
    if (value < 0.0f) {
        uart_putchar('-');
        value = -value;
    }
    int16_t int_part = (int16_t)value;
    uart_putint(int_part);
    uart_putchar('.');
    float frac = value - (float)int_part;
    for (uint8_t i = 0; i < decimals; i++) {
        frac *= 10.0f;
        int16_t digit = (int16_t)frac;
        uart_putchar('0' + digit);
        frac -= (float)digit;
    }
}

// =============================================================================
// Timer 2 — millis() counter
//   Prescaler 64, CTC, OCR2A = 249 → exactly 1kHz
// =============================================================================
static volatile uint32_t system_millis = 0;

ISR(TIMER2_COMPA_vect)
{
    system_millis++;
}

void timer2_millis_init(void)
{
    TCCR2A = (1 << WGM21);    // CTC mode
    TCCR2B = (1 << CS22);     // prescaler = 64
    OCR2A  = 249;              // 16MHz / 64 / 250 = 1kHz exactly
    TIMSK2 = (1 << OCIE2A);
}

unsigned long millis_now(void)
{
    unsigned long ms;
    cli();
    ms = system_millis;
    sei();
    return ms;
}

// =============================================================================
// Timer 0 — Fan PWM (Phase-correct 8-bit)
//   OC0A = Thrust (PD6), OC0B = Lift (PD5)
// =============================================================================
void timer0_fans_init(void)
{
    TCCR0A = (1 << WGM00)
           | (1 << COM0A1)
           | (1 << COM0B1);
    TCCR0B = (1 << CS00);     // prescaler = 1

    DDRD |= (1 << THRUST_FAN_PIN) | (1 << LIFT_FAN_PIN);
    OCR0A = 0;
    OCR0B = 0;
}

void set_thrust(uint8_t speed) { OCR0A = speed; }
void set_lift(uint8_t speed)   { OCR0B = speed; }

// =============================================================================
// Timer 1 — Servo PWM (Fast PWM Mode 14, TOP = ICR1)
//   Prescaler 8 → 0.5µs/tick, ICR1=40000 → 20ms period
// =============================================================================
void timer1_servo_init(void)
{
    DDRB |= (1 << SERVO_PIN);
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13)  | (1 << WGM12) | (1 << CS11);
    ICR1   = 40000;
    OCR1A  = 3000;  // centre (1.5ms)
}

void servo_write(int angle)
{
    if (angle < 5)   angle = 5;
    if (angle > 175) angle = 175;
    OCR1A = (uint16_t)(1200 + ((uint32_t)angle * 3600UL) / 180UL);
}

// =============================================================================
// I2C (TWI)
// =============================================================================
static volatile struct { uint8_t TWI_ACK : 1; } twi_flags;

void twi_begin(void)
{
    TWSR = 0;
    TWBR = 72;  // ~100kHz at 16MHz
    twi_flags.TWI_ACK = 0;
}

static uint8_t twi_start(uint8_t addr, uint8_t rw)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    if (((TWSR & 0xF8) != TW_START) && ((TWSR & 0xF8) != TW_REP_START))
        return 1;
    TWDR = (uint8_t)((addr << 1) | (rw & 1));
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    if (((TWSR & 0xF8) != TW_MT_SLA_ACK) && ((TWSR & 0xF8) != TW_MR_SLA_ACK))
        return 2;
    return 0;
}

static void twi_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    while (TWCR & (1 << TWSTO));
}

static uint8_t twi_write_byte(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return ((TWSR & 0xF8) != TW_MT_DATA_ACK) ? 1 : 0;
}

static uint8_t twi_read_ack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    twi_flags.TWI_ACK = ((TWSR & 0xF8) == TW_MR_DATA_ACK) ? 1 : 0;
    return TWDR;
}

static uint8_t twi_read_nack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    twi_flags.TWI_ACK = ((TWSR & 0xF8) == TW_MR_DATA_NACK) ? 1 : 0;
    return TWDR;
}

static uint8_t reg_write(uint8_t dev, uint8_t reg, uint8_t val)
{
    if (twi_start(dev, 0)) return 1;
    if (twi_write_byte(reg)) { twi_stop(); return 2; }
    if (twi_write_byte(val)) { twi_stop(); return 3; }
    twi_stop();
    return 0;
}

static uint8_t reg_read_n(uint8_t dev, uint8_t reg, uint8_t n, uint8_t *buf)
{
    if (twi_start(dev, 0)) return 1;
    if (twi_write_byte(reg)) { twi_stop(); return 2; }
    if (twi_start(dev, 1))  { twi_stop(); return 3; }
    for (uint8_t i = 0; i < n - 1; i++) {
        buf[i] = twi_read_ack();
        if (!twi_flags.TWI_ACK) { twi_stop(); return 4; }
    }
    buf[n - 1] = twi_read_nack();
    twi_stop();
    return 0;
}

// =============================================================================
// MPU-6050
// =============================================================================
void imu_init(void)
{
    reg_write(MPU6050_ADDR, 0x6B, 0x80);  // reset
    _delay_ms(100);
    reg_write(MPU6050_ADDR, 0x6B, 0x01);  // wake, PLL on X gyro
    _delay_ms(10);
    reg_write(MPU6050_ADDR, 0x1C, 0x00);  // accel ±2g
    reg_write(MPU6050_ADDR, 0x1B, 0x00);  // gyro ±250°/s
    reg_write(MPU6050_ADDR, 0x19, 0x13);  // ~50Hz sample rate
}

void imu_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                  int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14];
    if (reg_read_n(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 14, buf) != 0) return;
    *ax = (int16_t)((buf[0]  << 8) | buf[1]);
    *ay = (int16_t)((buf[2]  << 8) | buf[3]);
    *az = (int16_t)((buf[4]  << 8) | buf[5]);
    // buf[6..7] = temperature, skipped
    *gx = (int16_t)((buf[8]  << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);
}

// =============================================================================
// Calibration
// =============================================================================
static float gyro_off_x  = 0, gyro_off_y  = 0, gyro_off_z  = 0;
static float accel_off_x = 0, accel_off_y = 0, accel_off_z = 0;

void imu_calibrate(void)
{
    uart_puts("\nCalibrating IMU — keep still...\n");
    _delay_ms(600);

    long sax=0,say=0,saz=0,sgx=0,sgy=0,sgz=0;
    int16_t ax,ay,az,gx,gy,gz;

    for (int i = 0; i < CALIBRATION_SAMPLES + 100; i++) {
        imu_read_raw(&ax,&ay,&az,&gx,&gy,&gz);
        if (i >= 100) {
            sax+=ax; say+=ay; saz+=az;
            sgx+=gx; sgy+=gy; sgz+=gz;
        }
        _delay_ms(2);
    }

    accel_off_x = (float)sax / CALIBRATION_SAMPLES;
    accel_off_y = (float)say / CALIBRATION_SAMPLES;
    accel_off_z = (float)saz / CALIBRATION_SAMPLES;
    gyro_off_x  = (float)sgx / CALIBRATION_SAMPLES;
    gyro_off_y  = (float)sgy / CALIBRATION_SAMPLES;
    gyro_off_z  = (float)sgz / CALIBRATION_SAMPLES;

    uart_puts("Calibration done.\n");
}

// =============================================================================
// IMU update — offsets applied BEFORE angle calculation (bug fix)
// =============================================================================
static float roll=0, pitch=0, yaw=0;
static unsigned long last_imu_time  = 0;
static unsigned long last_print_time = 0;

void imu_update(void)
{
    int16_t ax_r,ay_r,az_r,gx_r,gy_r,gz_r;
    imu_read_raw(&ax_r,&ay_r,&az_r,&gx_r,&gy_r,&gz_r);

    // 1. Apply offsets first
    float ax = (ax_r - accel_off_x) / ACCEL_SCALE;
    float ay = (ay_r - accel_off_y) / ACCEL_SCALE;
    float az = (az_r - accel_off_z) / ACCEL_SCALE;
    float gz = (gz_r - gyro_off_z)  / GYRO_SCALE;

    // 2. Compute angles from corrected data
    roll  = atan2f(ay, az) * (180.0f / PI);
    pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * (180.0f / PI);

    // 3. Integrate yaw
    unsigned long now = millis_now();
    float dt = (now - last_imu_time) / 1000.0f;
    last_imu_time = now;
    yaw += gz * dt;

    // 4. Debug print every second
    if (now - last_print_time >= 5000) {
        last_print_time = now;
        uart_puts("Roll: ");  uart_putfloat(roll,  1);
        uart_puts("  Pitch: "); uart_putfloat(pitch, 1);
        uart_puts("  Yaw: ");   uart_putfloat(yaw,   1);
        uart_putchar('\n');
    }
}

// =============================================================================
// Servo heading-hold
// =============================================================================
void servo_update_from_yaw(float target_yaw)
{
    // checkpoint
    float error = target_yaw + yaw; 

    // while (error >  180.0f) error -= 360.0f;
    // while (error < -180.0f) error += 360.0f;

    float correction = error;

    // Optional: reduce saturation OR scale instead
    if (correction >  70.0f) correction =  70.0f;
    if (correction < -70.0f) correction = -70.0f;

    servo_write((int)(90.0f + correction));
}

// =============================================================================
// Hovercraft struct
// =============================================================================


void hvc_init(Hovercraft *h)
{
    US_init(&h->us_sensor, USPin_trig, USPin_echo, THRUST_FAN_PIN, LIFT_FAN_PIN);
    IR_sensor_initialize(&h->ir_sensor, IR_PIN);
}

// =============================================================================
// ADC
// =============================================================================
void adc_init(void)
{
    ADMUX  = (1 << REFS0);
    ADCSRA = (1 << ADEN)|(1 << ADPS2)|(1 << ADPS1)|(1 << ADPS0);
    DDRC  &= ~(1 << IR_PIN);
}

uint16_t adc_read(void)
{
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

// =============================================================================
// main
// =============================================================================
int main(void)
{
    cli();
    timer2_millis_init();   // Timer 2 → millis (must be first)
    sei();

    uart_init();            // single UART init
    uart_puts("\nHovercraft boot\n");

    adc_init();
    timer0_fans_init();     // Timer 0 → fans
    timer1_servo_init();    // Timer 1 → servo

    hvc_init(&hvc);

    twi_begin();
    imu_init();
    imu_calibrate();

    last_imu_time   = millis_now();
    last_print_time = millis_now();

    float target_yaw = 0.0f;  // hold starting heading

    // Set fan speeds 0-255, adjust for your hardware
    set_lift(255);
    set_thrust(255);

    int perpendicular = 0;
    int counter = 0;

    while (1) {
        // US_Sensor distance reading
        int angles[] = {5, 90, 175};
        int control = 0;
        
        int index = 0;
        imu_update();
        counter++;

        if (counter == 10 && perpendicular == 0 && control_fans(&hvc.us_sensor)){ // leftmost condition 1st in C...
            
            uart_puts("distance: ");
            uart_putfloat(hvc.us_sensor.distance, 1);
            uart_puts("\n");
            uart_puts("initial angle: ");
            uart_putfloat(target_yaw, 1);
            uart_puts("\n");
            set_lift(0); // LOOK HERE
            set_thrust(0);
            // direction change
            _delay_ms(1000);
            control = 0;
            index = 0;
            // look around 5 directions
            for (int i = 0; i < 3; i++){

                uart_puts("Angle: ");
                uart_putfloat(angles[i], 1);
                uart_puts("\n");

                servo_write(angles[i]);
                _delay_ms(750);
                searching(&hvc.us_sensor);
                // print
                uart_putfloat(hvc.us_sensor.distance, 1);
                _delay_us(2);
                uart_puts("\n");
                if(hvc.us_sensor.distance > control){
                    control = hvc.us_sensor.distance;
                    index = i;
                }
            }
            // get the new offset value, move in new direction
            if(index == 0){
                target_yaw = target_yaw - 90;
                perpendicular = 20;
            } else if (index == 2) {
                target_yaw = target_yaw + 90;
                perpendicular = 20;
            } else {
                perpendicular = 10;
            }
            // checkpoint
            
            uart_puts("new angle: ");
            uart_putfloat(target_yaw, 1);
            uart_puts("\n");
            imu_update();
            servo_update_from_yaw(target_yaw);
            _delay_ms(1000);
            searching(&hvc.us_sensor);
            
            // now turning logic for 90 degrees  
        } else if(perpendicular > 0) {
            uart_puts("distance: ");
            uart_putfloat(hvc.us_sensor.distance, 1);
            uart_puts("\n");
            set_lift(hvc.us_sensor.lift_pwm * 0.9);
            set_thrust(hvc.us_sensor.thrust_pwm * 0.8);
            perpendicular--;
        } else {
            uart_puts("distance: ");
            uart_putfloat(hvc.us_sensor.distance, 1);
            uart_puts("\n");
            int difference = abs(target_yaw - yaw); // in case turning be smoother
            set_lift(hvc.us_sensor.lift_pwm);
            set_thrust(hvc.us_sensor.thrust_pwm);
        }

        // IMU heading calibration
        imu_update();
        servo_update_from_yaw(target_yaw);
        _delay_ms(1);

        // IR sensor to detect overhead
        int tmp = 0;
        if (counter == 10){
            counter = 0;
            for(int i = 0; i < 5; i++){
                IR_sensor_update(&hvc.ir_sensor);
                tmp += hvc.ir_sensor.distance;
            }
            tmp = tmp/5;
            if (tmp < 40){
                set_lift(0);
                set_thrust(0);
                servo_update_from_yaw(-yaw);
                _delay_ms(3000);
            }
        }
    }

    return 0;
}
