/*
 * init.h
 *
 *  Created on: Aug 1, 2025
 *      Author: jeffr
 */

#ifndef INC_INIT_H_
#define INC_INIT_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "mainpp.h"
#include "Mx106v2.h"
#include "Mx106v2_CRC.h"
#include "stm32f4xx_it.h"
#include "motor_control.h"
#include "ROS2STM.h"
#include "ps4_parser.h"
#include "inv_kine.h"
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>

void init();
#ifdef __cplusplus
}
#endif

#endif /* INC_INIT_H_ */
