/*
 * @file sensors.h
 *
 * @brief   Created on: Jul 10, 2018
 *          Author: Artur Balanuta <Artur [dot] Balanuta [at] Gmail [dot] com>
 */

#include "bme680.h" // for struct bme680_field_data and BME680_GASM_VALID_MSK

#ifndef __SENSORS_H__
#define __SENSORS_H__

void BoardInitSensors(bool motion_en);
uint16_t getMIC(void);
uint16_t getLUX(void);
uint32_t getPIR(void);
struct bme680_field_data getBME(void);
uint32_t getBMXInts(void);
void setupBMI(bool enabled);
void initBMM(void);

#endif // __SENSORS_H__
