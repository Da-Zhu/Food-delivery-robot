#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H
#include "sys.h"

#define ultrasonic_trig HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);

void ultrasonic_init(void);

#endif
