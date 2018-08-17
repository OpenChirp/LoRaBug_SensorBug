/*
 * @file sensors.h
 *
 * @brief   Created on: Jul 10, 2018
 *          Author: Artur Balanuta <Artur [dot] Balanuta [at] Gmail [dot] com>
 */

#include <PERIPHERALS.h>
#include "bme680.h"

extern void BoardInitSensors(void);
extern uint16_t getMIC(void);
extern uint16_t getLUX(void);
extern uint8_t getPIR(void);
extern struct bme680_field_data getBME(void);
extern uint8_t getBMXInts(void);
