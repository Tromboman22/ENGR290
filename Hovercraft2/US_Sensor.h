// US sensor .h file

#ifndef US_SENSOR_H_
#define US_SENSOR_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// struct data

typedef struct
{
    uint8_t trig_pin;
    uint8_t echo_pin;
    uint8_t thrust_fan;
    uint8_t lift_fan;
    volatile uint32_t distance;
    volatile uint32_t pulse_width;
    float min_distance;
    float max_distance;
    uint32_t min_pulse;
    uint32_t max_pulse;
    uint8_t min_fan_speed;
    uint8_t max_fan_speed;
    uint8_t thrust_pwm;
    uint8_t lift_pwm;

} US_Sensor;

void US_init(US_Sensor *sensor, uint8_t trig_pin, uint8_t echo_pin, uint8_t thrust_fan, uint8_t lift_fan);

bool control_fans(US_Sensor *sensor);

void trigger_pulse(US_Sensor *sensor);

uint32_t pulse_length(US_Sensor *sensor);

uint32_t getterDistance(uint32_t pulseVal);

bool set_fanspeed(US_Sensor *sensor);

void searching(US_Sensor *sensor);

#ifdef __cplusplus
}
#endif

#endif /* US_SENSOR_H_ */
