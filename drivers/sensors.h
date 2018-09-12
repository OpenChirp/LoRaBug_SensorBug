/*
 * @file sensors.h
 *
 * @brief   Created on: Jul 10, 2018
 *          Author: Artur Balanuta <Artur [dot] Balanuta [at] Gmail [dot] com>
 */

#include "bme680.h" // for struct bme680_field_data and BME680_GASM_VALID_MSK

#ifndef __SENSORS_H__
#define __SENSORS_H__

#define MIC_SAMPLES             5000                            // MAXVAL < uint16_t
#define LUX_SAMPLES             100                             // MAXVAL < uint16_t
#define LUX_AVG_MIN_VALUE       8.0                             // The average minimum value the sensor reports in complete darkness
#define LUX_AVG_MAX_VALUE       2400.0                          // Average value when the sensor is exposed to very bright light
#define LUX_SCALE_MULTIPLIER    (0xffff/LUX_AVG_MAX_VALUE)      // Maximizes the read value to the uint16 scale

/**@def DEFAULT_MIC_BIAS
 * This is the 12-bit ADC value that we use as first avg offset for stddev calc.
 * The ADC internal reference is 4.3V and the MIC regulated DC bias is 1.8V/2=0.9V.
 * Therefore, the DC bias point should be approximately 0.9V/4.3V * 2^12 = 857.3
 */
#define DEFAULT_MIC_BIAS        857

void BoardInitSensors(bool motion_en);
uint16_t getMIC(void);
uint32_t sampleNoise(void);
uint16_t getLUX(void);
uint32_t getPIR(void);
struct bme680_field_data getBME(void);
uint32_t getBMXInts(void);
void setupBMI(bool enabled);
void initBMM(void);

#endif // __SENSORS_H__
