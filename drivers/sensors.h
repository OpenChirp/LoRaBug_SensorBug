/*
 * @file sensors.h
 *
 * @brief   Created on: Jul 10, 2018
 *          Author: Artur Balanuta <Artur [dot] Balanuta [at] Gmail [dot] com>
 */

#include "bme680.h" // for struct bme680_field_data and BME680_GASM_VALID_MSK

#ifndef __SENSORS_H__
#define __SENSORS_H__

#define LUX_SAMPLES             100                             // MAXVAL < uint16_t
#define LUX_AVG_MIN_VALUE       8.0                             // The average minimum value the sensor reports in complete darkness
#define LUX_AVG_MAX_VALUE       2400.0                          // Average value when the sensor is exposed to very bright light
#define LUX_SCALE_MULTIPLIER    (0xffff/LUX_AVG_MAX_VALUE)      // Maximizes the read value to the uint16 scale

#define ADCBUFFERSIZE       100
//#define MIC_SAMPLING_RUNTIME_MS 50                            // 50ms
//#define MIC_SAMPLING_RUNTIME_MS 100                           // 100ms
#define MIC_SAMPLING_RUNTIME_MS 500                           // 500ms
#define MIC_SAMPLING_FREQ       (2*20*1000)                   // 20kHz * 2
#define MIC_SAMPLING_COUNT      ((MIC_SAMPLING_FREQ/1000) * MIC_SAMPLING_RUNTIME_MS) // Number of total samples to span SAMPLING_RUNTIME_MS
//#define MIC_ADC_BUFFERS_COUNT   ((SAMPLING_COUNT + ADCBUFFERSIZE - 1)/ ADCBUFFERSIZE) // Number of ADC buffers we need in order to fetch a min of SAMPLING_COUNT

#define MIC_STABILIZE_TIME_MS   200                             // The time required for the MIC output to stabilize
#define LIGHT_STABILIZE_TIME_MS 1                               //TODO The time required for the light sensor to stabilize

/**@def DEFAULT_MIC_BIAS
 * This is the 12-bit ADC value that we use as first avg offset for stddev calc.
 * The MIC regulated DC bias is 1.8V/2=0.9V.
 * The corrected and converted ADC values are in microvolts.
 * Therefore, 0.9V = 900000uV
 */
#define DEFAULT_MIC_BIAS        900000

void BoardInitSensors(bool motion_en);
uint32_t sampleNoise(void);
uint16_t getLUX(void);
uint32_t getPIR(void);
struct bme680_field_data getBME(void);
uint32_t getBMXInts(void);
void setupBMI(bool enabled);
void initBMM(void);

#endif // __SENSORS_H__
