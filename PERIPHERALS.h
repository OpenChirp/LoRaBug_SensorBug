/** @file PERIPHERALS.h
 *
 * @brief Place to describe accessory board specific configuration
 *
 * @date Aug 17, 2018
 * @author Craig Hesling
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include "LORABUG.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SPI Board */
#define Board_SPI1_MISO             PIN_UNASSIGNED
#define Board_SPI1_MOSI             PIN_UNASSIGNED
#define Board_SPI1_CLK              PIN_UNASSIGNED
#define Board_SPI1_CSN              PIN_UNASSIGNED

/* I2C */
#define Board_I2C0_SCL0             PIN_UNASSIGNED
#define Board_I2C0_SDA0             PIN_UNASSIGNED

/* SPI Flash */
#define Board_SPI_FLASH_CS          PIN_UNASSIGNED
#define Board_FLASH_CS_ON           0
#define Board_FLASH_CS_OFF          1

/* PWM outputs */
#define Board_PWMPIN0               Board_RLED
#define Board_PWMPIN1               Board_GLED
#define Board_PWMPIN2               PIN_UNASSIGNED
#define Board_PWMPIN3               PIN_UNASSIGNED
#define Board_PWMPIN4               PIN_UNASSIGNED
#define Board_PWMPIN5               PIN_UNASSIGNED
#define Board_PWMPIN6               PIN_UNASSIGNED
#define Board_PWMPIN7               PIN_UNASSIGNED


#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_H_ */
