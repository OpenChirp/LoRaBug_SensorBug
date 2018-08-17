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
#define Board_I2C0_SCL0             Board_HDR_HDIO2
#define Board_I2C0_SDA0             Board_HDR_HDIO0

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

/* Header pins */

#define DOMAIN1_EN      Board_HDR_HDIO1 // Active High - Also CS# on Flash
#define DOMAIN2_ENN     Board_HDR_ADIO7 // Active Low
#define LIGHT_OUT       Board_HDR_ADIO0 // Analog Input
#define MIC_OUT         Board_HDR_ADIO2 // Analog Input
#define BMX_INT1        Board_HDR_ADIO4 // Digital Input
#define PIR_OUT         Board_HDR_ADIO6 // Digital Input

#define FLASH_MOSI      Board_HDR_ADIO1
#define FLASH_MISO      Board_HDR_ADIO5
#define FLASH_CLK       Board_HDR_ADIO3
#define FLASH_CSN       DOMAIN1_EN

#define ADC_INDEX_LUX   LORABUG_ADC0
#define ADC_INDEX_MIC   LORABUG_ADC2

#define MIC_SAMPLES             5000                            // MAXVAL < uint16_t
#define LUX_SAMPLES             100                             // MAXVAL < uint16_t
#define LUX_AVG_MIN_VALUE       8.0                             // The average minimum value the sensor reports in complete darkness
#define LUX_AVG_MAX_VALUE       1200.0                          // Average value when the sensor is exposed to very bright light
#define LUX_SCALE_MULTIPLIER    (0xffff/LUX_AVG_MAX_VALUE)      // Maximizes the read value to the uint16 scale

/* Macros to define power domain signal levels,
 * since they have different polarity. */
#define DOMAIN1_OFF     PIN_GPIO_LOW
#define DOMAIN1_ON      PIN_GPIO_HIGH
#define DOMAIN2_OFF     PIN_GPIO_HIGH
#define DOMAIN2_ON      PIN_GPIO_LOW

/* Add items to pin init table in LORABUG.c */
#define PERIPHERALS_PIN_INIT \
        DOMAIN1_EN     | PIN_GPIO_OUTPUT_EN | DOMAIN1_OFF | PIN_PUSHPULL | PIN_DRVSTR_MAX, \
        DOMAIN2_ENN    | PIN_GPIO_OUTPUT_EN | DOMAIN2_OFF | PIN_PUSHPULL,

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_H_ */
