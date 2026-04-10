#ifndef IR_SENSOR_H_
#define IR_SENSOR_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_SAMPLE_MAX 4

typedef struct {
    uint8_t IRPin;     // ADC pin for the IR sensor (0-7)
    float reading;     // averaged raw ADC reading
    float distance;    // calculated distance in cm
} IRSensor;

// Initialize the ADC and set up the sensor pin
void IR_sensor_initialize(IRSensor *sensor, uint8_t irpin);

// Read a single ADC value from the sensor
uint16_t IR_ADC_read(IRSensor *sensor);

// Update the sensor reading and calculate distance
void IR_sensor_update(IRSensor *sensor);

#ifdef __cplusplus
}
#endif

#endif /* IR_SENSOR_H_ */