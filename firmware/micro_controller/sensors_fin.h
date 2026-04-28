#pragma once

#include <stdbool.h>
#include "driver/gpio.h"

// ============================
// APDS9960 Ambient Light
// ============================

void sensors_init(void);          // initialize I2C + APDS9960
int  sensors_get_ambient(void);   // raw ambient light value
bool sensors_is_dark(void);       // threshold comparison

// ============================
// PIR Motion Sensors
// ============================

#define PIR_OUTSIDE_PIN GPIO_NUM_14 
#define PIR_INSIDE_PIN  GPIO_NUM_21 

void init_pir(void);

bool read_pir_outside(void);
bool read_pir_inside(void);