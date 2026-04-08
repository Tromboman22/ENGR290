#include "IR_Sensor.h"

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>


// void IRSensor_init(IRSensor *sensor, uint8_t IRpin, uint8_t samples) {
//     sensor->pin = IRpin;
//     sensor->samples = samples;
//     sensor->raw = 0;
//     sensor->distance = 0;
// }