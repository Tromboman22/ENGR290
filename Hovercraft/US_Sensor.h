// US sensor .h file

#ifndef US_SENSOR_H_
#define US_SENSOR_H_

#include <stdint.h>

// struct data

typedef struct
{
    uint8_t trig_pin;
    uint8_t echo_pin;
    uint8_t thrust_fan;
    uint8_t lift_fan;
    volatile uint32_t distance;
    volatile uint32_t pulse_width;
    const float min_distance;
    const float max_distance;
    const uint32_t min_pulse;
    const uint32_t max_pulse;
    uint8_t thrust_pwm;
    uint8_t lift_pwm;

} US_Sensor;

void US_init(US_Sensor *sensor, uint8_t trig_pin, uint8_t echo_pin, uint8_t thrust_fan, uint8_t lift_fan);

bool control_fans(US_Sensor *sensor);

void trigger_pulse(US_Sensor *sensor);

uint32_t pulse_length(US_Sensor *sensor);

uint32_t getterDistance(uint32_t pulseVal);

void set_fanspeed(uint32_t distance, US_Sensor *sensor);

#endif /* US_SENSOR_H_ */
