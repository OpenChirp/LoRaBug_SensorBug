/**@file sample_noise.c
 *
 * @date Sep 10, 2018
 * @author Craig Hesling
 */

#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h> // BIOS_WAIT_FOREVER
#include <ti/sysbios/knl/Event.h>

/* TI-RTOS drivers */
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/adcbuf/ADCBufCC26XX.h>

#include <PERIPHERALS.h>
#include <io.h>
#include "sensors.h"

#include <math.h>

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


#define ADCBUFFERSIZE       100
//#define SAMPLING_RUNTIME_MS 50                            // 50ms
//#define SAMPLING_RUNTIME_MS 100                           // 100ms
#define SAMPLING_RUNTIME_MS 500                           // 500ms
#define SAMPLING_FREQ       (2*20*1000)                   // 20kHz * 2
#define SAMPLING_COUNT      ((SAMPLING_FREQ/1000) * SAMPLING_RUNTIME_MS) // Number of total samples to span SAMPLING_RUNTIME_MS
//#define ADC_BUFFERS_COUNT   ((SAMPLING_COUNT + ADCBUFFERSIZE - 1)/ ADCBUFFERSIZE) // Number of ADC buffers we need in order to fetch a min of SAMPLING_COUNT

static uint16_t sampleBufferOne[ADCBUFFERSIZE];
static uint16_t sampleBufferTwo[ADCBUFFERSIZE];
static uint32_t microVoltBuffer[ADCBUFFERSIZE];
static uint32_t samplesCountdown = 0;

static uint32_t lastMICAvg = DEFAULT_MIC_BIAS;

#define EVENT_SAMPLING_FINISHED Event_Id_00
static Event_Struct adcEventsStruct;
static Event_Handle adcEvents;

#define MIN(x,y) ( (x < y) ? x : y )

/*
 * This function is called whenever a buffer is full.
 * The content of the buffer is then converted into human-readable format and
 * sent to the PC via UART.
 *
 */
static void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion, void *completedADCBuffer, uint32_t completedChannel) {

    uint_fast16_t i, sampleCount;
    if (samplesCountdown <= ADCBUFFERSIZE) {
        ADCBuf_convertCancel(handle);
        sampleCount = samplesCountdown;
        samplesCountdown = 0;
    } else {
        sampleCount = ADCBUFFERSIZE;
        samplesCountdown -= ADCBUFFERSIZE;
    }

    /* Adjust raw adc values and convert them to microvolts */
    ADCBuf_adjustRawValues(handle, completedADCBuffer, sampleCount, completedChannel);
    ADCBuf_convertAdjustedToMicroVolts(handle, completedChannel, completedADCBuffer, microVoltBuffer, sampleCount);

    void (*reduce)(uint32_t microVoltReadin) = (void (*)(uint32_t))conversion->arg;
    for (i = 0; i < sampleCount; i++) {
        reduce(microVoltBuffer[i]);
    }

    if (samplesCountdown == 0) {
        Event_post(adcEvents, EVENT_SAMPLING_FINISHED);
    }
}

static uint64_t sampleSum;
static uint64_t sampleSquaredSum;

static void micReduce(uint32_t microVoltReading) {
    uint64_t val = microVoltReading;
    sampleSum += val;
    int64_t square = int_square((int64_t)val - (int64_t)lastMICAvg);
    // Check for overflow
    if (sampleSquaredSum > (sampleSquaredSum+square)) {
        sampleSquaredSum = (uint64_t)-1;
        System_abort("Overflow detected in sample noise");
    }
    sampleSquaredSum += square;
}

/**
 *
 * @return The RMS in millivolts
 */
uint32_t sampleNoise() {
    ADCBuf_Handle adcBuf;
    ADCBuf_Params adcBufParams;
    ADCBuf_Conversion continuousConversion;

    debugprintf("Sampling Noise\r\n");

    Event_construct(&adcEventsStruct, NULL);
    adcEvents = Event_handle(&adcEventsStruct);

    /* Set up an ADCBuf peripheral in ADCBuf_RECURRENCE_MODE_CONTINUOUS */
    ADCBufCC26XX_ParamsExtension adcCustomParams = {
        .inputScalingEnabled = true,
        .refSource = ADCBufCC26XX_FIXED_REFERENCE,
        .samplingMode = ADCBufCC26XX_SAMPING_MODE_ASYNCHRONOUS,
//        .samplingMode = ADCBufCC26XX_SAMPING_MODE_SYNCHRONOUS,
        .samplingDuration = ADCBufCC26XX_SAMPLING_DURATION_21P3_US,
    };

    ADCBuf_Params_init(&adcBufParams);
    adcBufParams.callbackFxn = adcBufCallback;
    adcBufParams.recurrenceMode = ADCBuf_RECURRENCE_MODE_CONTINUOUS;
    adcBufParams.returnMode = ADCBuf_RETURN_MODE_CALLBACK;
    adcBufParams.samplingFrequency = SAMPLING_FREQ;
    adcBufParams.custom = &adcCustomParams;
    adcBuf = ADCBuf_open(Board_ADCBuf0, &adcBufParams);


    /* Configure the conversion struct */
    continuousConversion.arg = (void *)micReduce;
    continuousConversion.adcChannel = ADC_INDEX_MIC;
    continuousConversion.sampleBuffer = sampleBufferOne;
    continuousConversion.sampleBufferTwo = sampleBufferTwo;
    continuousConversion.samplesRequestedCount = ADCBUFFERSIZE;

    if (!adcBuf){
        System_abort("adcBuf did not open correctly\n");
    }

    samplesCountdown = SAMPLING_COUNT;
    sampleSum        = 0;
    sampleSquaredSum = 0;


    /* Start converting. */
    if (ADCBuf_convert(adcBuf, &continuousConversion, 1) != ADCBuf_STATUS_SUCCESS) {
        System_abort("Did not start conversion process correctly\n");
    }
    Event_pend(adcEvents, Event_Id_NONE, EVENT_SAMPLING_FINISHED, BIOS_WAIT_FOREVER);
    uint32_t avg =  sampleSum / SAMPLING_COUNT;
    uint32_t newavg = (lastMICAvg + avg) / 2;

    debugprintf("Old AVG = %d\n", lastMICAvg);
    debugprintf("AVG = %d\n", avg);
    debugprintf("New AVG = %d\n", newavg);

    lastMICAvg = newavg;

    uint64_t variance = sampleSquaredSum / SAMPLING_COUNT;
    uint32_t stddev = int_sqrt64(variance);
    debugprintf("STDDEV = %d\n", stddev);

    ADCBuf_close(adcBuf);
    Event_destruct(&adcEventsStruct);

    return stddev / 1000;
}
