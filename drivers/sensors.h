/*
 * @file sensors.h
 *
 * @brief   Created on: Jul 10, 2018
 *          Author: Artur Balanuta <Artur [dot] Balanuta [at] Gmail [dot] com>
 */

#include "bme680.h" // for struct bme680_field_data and BME680_GASM_VALID_MSK

#ifndef __SENSORS_H__
#define __SENSORS_H__

extern void BoardInitSensors(void);
extern uint16_t getMIC(void);
extern uint16_t getLUX(void);
extern uint8_t getPIR(void);
extern struct bme680_field_data getBME(void);
extern uint8_t getBMXInts(void);
extern void initBMI(void);
extern void initBMM(void);

#endif // __SENSORS_H__
