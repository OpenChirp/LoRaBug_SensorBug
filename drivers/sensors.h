/*
 * @file sensors.h
 *
 * @brief   Created on: Jul 10, 2018
 *          Author: Artur Balanuta <Artur [dot] Balanuta [at] Gmail [dot] com>
 */

#include "bme680.h" // for struct bme680_field_data and BME680_GASM_VALID_MSK

#ifndef __SENSORS_H__
#define __SENSORS_H__

/**
 * MIC sampling is currently configured to sample at 40kHz for 500ms.
 * Since the ADCBUFFERSIZE if 100 elements, the reducing callback has 2.5ms
 * to complete.
 * Currently, the reduction callback take about 518us to complete, which is
 * within the 2.5ms possible.
 *
 * Light sampling is currently configured to sample at 5kHz for 10ms.
 * Since this requires only 50 samples, we configure the adcbuf collection
 * to use 50 of the 100 element buffer in a one shot configuration.
 */
#define ADCBUFFERSIZE           100
#define MIC_SAMPLING_RUNTIME_MS 500                           // 500ms
#define MIC_SAMPLING_FREQ       (2*20*1000)                   // 20kHz * 2
#define MIC_SAMPLING_COUNT      ((MIC_SAMPLING_FREQ/1000) * MIC_SAMPLING_RUNTIME_MS) // Number of total samples to span SAMPLING_RUNTIME_MS
#define LIGHT_SAMPLING_FREQ     5000                          // 5kHz
#define LIGHT_SAMPLING_COUNT    50                            // 50

#define MIC_STABILIZE_TIME_US   200000                        // The time required for the MIC output to stabilize
#define LIGHT_STABILIZE_TIME_US 90                            // The time required for the light sensor to stabilize

/**@def DEFAULT_MIC_BIAS
 * This is the 12-bit ADC value that we use as first avg offset for stddev calc.
 * The MIC regulated DC bias is 1.8V/2=0.9V.
 * The corrected and converted ADC values are in microvolts.
 * Therefore, 0.9V = 900000uV
 */
#define DEFAULT_MIC_BIAS        900000

void BoardInitSensors(bool motion_en);
void sampleNoiseStart(void);
uint32_t sampleNoiseWaitResult(void);
void sampleLightStart(void);
uint32_t sampleLightWaitResult(void);
uint32_t getPIR(void);
struct bme680_field_data *getBME(void);
uint32_t getBMXInts(void);
void setupBMI(bool enabled);
void initBMM(void);

#endif // __SENSORS_H__
