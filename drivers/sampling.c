/**@file sample_noise.c
 *
 * @date Sep 10, 2018
 * @author Craig Hesling
 */

#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h> // BIOS_WAIT_FOREVER
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS drivers */
#include <ti/drivers/ADC.h>

#include <PERIPHERALS.h>
#include <io.h>
#include <error.h>
#include "sensors.h"

#include <math.h>

/*--------------------------------------------------------*
 *                    Configuration                       *
 *--------------------------------------------------------*/

/**@def ENABLE_TIMING_MEASUREMENTS
 * When enabled, the the two uint64_t variables
 * durationNs and periodNs are written to with the duration
 * of the adcbuf handler function and the period between calls
 * to the adcbuf handler function, respectively,
 */
//#define ENABLE_TIMING_MEASUREMENTS


/*--------------------------------------------------------*
 *                    Math Functions                      *
 *--------------------------------------------------------*/

static inline
int64_t int_square(int64_t x) {
    return x*x;
}

// From https://gist.github.com/foobaz/3287f153d125277eefea
static inline
uint32_t int_sqrt64(uint64_t x) // 780 µs
{
    uint32_t res = 0;
    uint32_t add = 0x80000000;
    uint8_t i;
    for (i = 0; i < 32; i++)
    {
        uint32_t temp = res | add;
        uint64_t g2 = temp;
        g2 *= temp;
        if (x >= g2)
        {
            res = temp;
        }
        add >>= 1;
    }
    return res;
}

/*--------------------------------------------------------*
 *                    Setup Variable                      *
 *--------------------------------------------------------*/

static ADC_Handle adc;

//#define EVENT_SAMPLING_FINISHED Event_Id_00
//static Event_Struct adcEventsStruct;
//static Event_Handle adcEvents;

#define MIN(x,y) ( ((x) < (y)) ? (x) : (y) )

#ifdef ENABLE_TIMING_MEASUREMENTS
Types_Timestamp64 start, end;
Types_Timestamp64 lastStart = {0, 0};

/// The duration in nanoseconds that it took to finish
/// the adcBufCallback function.
volatile uint64_t durationNs; // inspect with debugger
/// The duration between calls to the adcBufCallback
/// function.
volatile uint64_t periodNs;   // inspect with debugger
#endif

/*--------------------------------------------------------*
 *                    Local Functions                     *
 *--------------------------------------------------------*/

static uint64_t sampleSum;
static uint64_t sampleSquaredSum;

static uint32_t lastMICAvg = DEFAULT_MIC_BIAS;

static void micReduce(uint32_t microVoltReading) {
    uint64_t val = microVoltReading;
    sampleSum += val;
    int64_t square = int_square((int64_t)val - (int64_t)lastMICAvg);
    // Check for overflow
    if (sampleSquaredSum>(sampleSquaredSum+square) || sampleSquaredSum==((uint64_t)-1)) {
        sampleSquaredSum = (uint64_t)-1;
//        System_abort("Overflow detected in sample noise");
        return;
    }
    sampleSquaredSum += square;
}

static void sumReduce(uint32_t microVoltReading) {
    sampleSum += (uint64_t)microVoltReading;
}


/**
 * This value is equal to the uV for 1lux.
 * This corresponds to 0.48uA through 27kOhm resistor.
 */
static const uint32_t uVIn1Lux = 48*27*10;

/**
 *
 * @param uV
 * @return
 */
static inline
uint32_t microVoltsToMilliLux(uint32_t uV) {
    // 1lux   = .48uA*27kOhm
    // 10lux  = 4.8uA*27kOhm
    // 100lux = 48uA*27kOhm

    return uV*1000 / uVIn1Lux;
}

/*--------------------------------------------------------*
 *                  Interface Functions                   *
 *--------------------------------------------------------*/

void sampleInit() {
//    Event_construct(&adcEventsStruct, NULL);
//    adcEvents = Event_handle(&adcEventsStruct);
}


/**
 * This starts the microphone sampling routine process.
 * This cannot be run concurrently with the light samping routine.
 */
void sampleNoiseStart() {
    debugprintf("Sampling Noise\r\n");

    ADC_Params adcParams;
    uint16_t adcValue;
    uint32_t adcValueMicoVolt;
    int count;
    const uint32_t sleepticks =  ((1000*1000) / MIC_SAMPLING_FREQ) / Clock_tickPeriod;

    sampleSum        = 0;
    sampleSquaredSum = 0;

    ADC_Params_init(&adcParams);


    adc = ADC_open(ADC_INDEX_MIC, &adcParams);
    if (adc == NULL) {
        System_abort("Failed to open ADC for MIC\n");
    }

    for (count = 0; count < MIC_SAMPLING_COUNT; count++) {
        if (ADC_convert(adc, &adcValue) != ADC_STATUS_SUCCESS) {
            System_abort("Failed to convert ADC value for MIC\n");
        }
        adcValueMicoVolt = ADC_convertToMicroVolts(adc, adcValue);
#ifdef ENABLE_TIMING_MEASUREMENTS
        TimestampNow(&start);
#endif
        micReduce(adcValueMicoVolt);
#ifdef ENABLE_TIMING_MEASUREMENTS
        TimestampNow(&end);
        TimestampDiffNs(&durationNs, &start, &end);
        TimestampDiffNs(&periodNs, &lastStart, &start);
        lastStart = start;
#endif
        Task_sleep(sleepticks);
    }

    ADC_close(adc);
}

/**
 *
 * @return The RMS in millivolts
 */
uint32_t sampleNoiseWaitResult() {
//    Event_pend(adcEvents, Event_Id_NONE, EVENT_SAMPLING_FINISHED, BIOS_WAIT_FOREVER);
//    ADCBuf_close(adcBuf);

    if (sampleSquaredSum == ((uint64_t)-1)) {
        return 0;
    }

    uint32_t avg =  sampleSum / MIC_SAMPLING_COUNT;
    uint32_t newavg = (lastMICAvg + avg) / 2;

    debugprintf("MIC Old AVG = %d\n", lastMICAvg);
    debugprintf("MIC AVG = %d\n", avg);
    debugprintf("MIC New AVG = %d\n", newavg);

    lastMICAvg = newavg;

    uint64_t variance = sampleSquaredSum / MIC_SAMPLING_COUNT;
    uint32_t stddev = int_sqrt64(variance);
    debugprintf("MIC STDDEV = %d\n", stddev);

    return stddev / 1000;
}


/**
 * Should take 1/5kHz * 50 samples = 10ms duration
 */
void sampleLightStart() {
    debugprintf("Sampling Light\r\n");

    ADC_Params adcParams;
    uint16_t adcValue;
    uint32_t adcValueMicoVolt;
    int count;
    const uint32_t sleepticks =  ((1000*1000) / LIGHT_SAMPLING_FREQ) / Clock_tickPeriod;

    sampleSum = 0;

    ADC_Params_init(&adcParams);


    adc = ADC_open(ADC_INDEX_LUX, &adcParams);
    if (adc == NULL) {
        System_abort("Failed to open ADC for light\n");
    }

    for (count = 0; count < LIGHT_SAMPLING_COUNT; count++) {
        if (ADC_convert(adc, &adcValue) != ADC_STATUS_SUCCESS) {
            System_abort("Failed to convert ADC value for light\n");
        }
        adcValueMicoVolt = ADC_convertToMicroVolts(adc, adcValue);
        sumReduce(adcValueMicoVolt);
        Task_sleep(sleepticks);
    }

    ADC_close(adc);
}

/**
 * @return The average in millilux
 */
uint32_t sampleLightWaitResult() {
//    Event_pend(adcEvents, Event_Id_NONE, EVENT_SAMPLING_FINISHED, BIOS_WAIT_FOREVER);
//    ADCBuf_close(adcBuf);

    uint64_t avg =  sampleSum / LIGHT_SAMPLING_COUNT;

    debugprintf("Light AVG = %llu = %f V\n", avg, ((double)avg)/1000.0/1000.0);

    return microVoltsToMilliLux(avg);
}
