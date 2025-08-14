/*
 * pressure_sensor.h
 *
 *  Created on: Aug 14, 2025
 *      Author: jeffr
 */

#ifndef INC_PRESSURE_SENSOR_H_
#define INC_PRESSURE_SENSOR_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t Read_Pressure_Right(void);
uint16_t Read_Pressure_Left(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_PRESSURE_SENSOR_H_ */
