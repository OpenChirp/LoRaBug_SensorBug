
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>

/* TI-RTOS Header files */
// #include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/ADC.h>
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Watchdog.h>

/* Board Header files */
#include "LORABUG.h"
#include "PERIPHERALS.h"

#include <string.h> // strlen in uartputs and LoRaWan code
#include <math.h>
#include <stdbool.h>
#include <sensors.h>
#include <ble_rf.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <sensorbug.pb.h>
#include "io.h"
#include "error.h"

/* LoRa Radio Header files */
#include "board.h" // The LoRaMac-node/src/boads/LoRaBug/board.h file
#include "radio.h"
#include "LoRaMac.h"
#include "Commissioning.h"


/*------------------------------------------------------------------------*/
/*                       Basic Setup                                      */
/*------------------------------------------------------------------------*/

#define TASKSTACKSIZE   2048

static Task_Struct mainTaskStruct;
static Char mainTaskStack[TASKSTACKSIZE];

/* Runtime Events */
#define EVENT_STATECHANGE   Event_Id_00
#define EVENT_BUTTONPRESSED Event_Id_01

static Event_Struct runtimeEventsStruct;
static Event_Handle runtimeEvents;

// Software Version 2.1
const uint32_t software_ver_major = 2;
const uint32_t software_ver_minor = 1;

/*------------------------------------------------------------------------*/
/*                     Configuration                                      */
/*------------------------------------------------------------------------*/

#define LED_ONTIME_MS                               100

/**@def USE_BOARD_UNIQUE_ID_DEV_EUI
 * When defined, the DevEUI is set to part of the unique IEEE 802.15.4
 * given to the MCU at manufacturing time.
 */
#define USE_BOARD_UNIQUE_ID_DEV_EUI

/**@def DISABLE_LEDS
 * When defined, the LEDs will be disabled after successfully
 * joining the LoRaWAN network.
 */
#define DISABLE_LEDS

/**@def ENABLE_BLE_ADVERTISEMENT
 * When defined, we will use BLE to advertise our DevEUI and AppKey
 * upon a button press.
 */
#define ENABLE_BLE_ADVERTISEMENT


/**@def DEFAULT_REPORT_PERIOD
 * The default number of seconds between reporting.
 */
#define DEFAULT_REPORT_PERIOD  10

#define DEFAULT_MOTION_ENABLED false
#define DEFAULT_LIGHT_ENABLED  true
#define DEFAULT_MIC_ENABLED    true

#define BUTTON_AS_RESET

/**@def WATCHDOG_ENABLED
 * When defined, the watchdog will be setup and used.
 * Please check the default duration(in ms) in LORABUG.c .
 */
//#define WATCHDOG_ENABLED

/**@def CALIBRATION_MODE_LIGHT
 * When set, the main task will continuously call the sampleLight
 * function and print it's results using \a debugprintf.
 */
//#define CALIBRATION_MODE_LIGHT

/**@def CALIBRATION_MODE_NOISE
 * When set, the main task will continuously call the sampleNoise
 * function and print it's results using \a debugprintf.
 */
//#define CALIBRATION_MODE_NOISE

/*------------------------------------------------------------------------*/
/*                          Convenience                                   */
/*------------------------------------------------------------------------*/

#define MYMAX(x,y) ( (x>y) ? x : y )


/*------------------------------------------------------------------------*/
/*                            Globals                                     */
/*------------------------------------------------------------------------*/

#ifdef WATCHDOG_ENABLED
static Watchdog_Handle watchdog;
#endif

/*------------------------------------------------------------------------*/
/*                      Start of LoRaWan Demo Code                        */
/*------------------------------------------------------------------------*/

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
//#define APP_TX_DUTYCYCLE                            3000         // 3sec - Basically the fastest possible interval
//#define APP_TX_DUTYCYCLE                            4000         // 4sec
//#define APP_TX_DUTYCYCLE                            (1000*60*15) // 15min
#define APP_TX_DUTYCYCLE                            Settings.report_period

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
//#define LORAWAN_DEFAULT_DATARATE                    DR_0
#define LORAWAN_DEFAULT_DATARATE                    DR_4

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    true
//#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
//#define LORAWAN_ADR_ON                              1
#define LORAWAN_ADR_ON                              0


/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_SIZE                       11

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

/*!
 * User application data buffer size
 */
//#define LORAWAN_APP_DATA_MAX_SIZE                           64
#define LORAWAN_APP_DATA_MAX_SIZE                           (SensorBugUplinkMsg_size) // max(SensorBugUplinkMsg_size, SensorBugDownlinkMsg_size)

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Timer to handle the state of LED1
 */
static TimerEvent_t Led1Timer;

/*!
 * Timer to handle the state of LED2
 */
static TimerEvent_t Led2Timer;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * Allow the user to schedule a packet to send immediately when we join
 */
static bool SendOnJoin = true;

/*!
 * Device states
 */
static enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP,
    DEVICE_STATE_BUTTON
}DeviceState;


/*------------------------------------------------------------------------*/
/*                          Settings                                      */
/*------------------------------------------------------------------------*/

typedef struct {
    uint32_t report_period; // Reporting period in ms
    bool     motion_enabled;
    bool     light_enabled;
    bool     mic_enabled;
} settings_t;

static settings_t Settings = {
  .report_period  = DEFAULT_REPORT_PERIOD*1000, // ms
  .motion_enabled = DEFAULT_MOTION_ENABLED,
  .light_enabled  = DEFAULT_LIGHT_ENABLED,
  .mic_enabled    = DEFAULT_MIC_ENABLED,
};

static void UpdateReportPeriod(uint32_t seconds) {
    debugprintf("# UpdatedReportPeriod: %d\r\n", seconds);
    // Valid interval 1s to 48h
    if( seconds >= 1 && seconds <= (48*60*60) ) {
        uint32_t ms = seconds*1000;
        if (ms != Settings.report_period) {
            Settings.report_period = ms;
            TimerStop(&TxNextPacketTimer);
            DeviceState = DEVICE_STATE_CYCLE;
            Event_post(runtimeEvents, EVENT_STATECHANGE);
        }
    }
}

static void UpdateMotionEnabled(bool motion_enabled) {
    debugprintf("# UpdatedMotionEnabled: %s\r\n", motion_enabled?"true":"false");
    if (motion_enabled != Settings.motion_enabled) {
        setupBMI(motion_enabled);
    }
    Settings.motion_enabled = motion_enabled;
}


/*------------------------------------------------------------------------*/
/*                          Handlers                                      */
/*------------------------------------------------------------------------*/


/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    static uint32_t counter = 1;
    static SensorBugUplinkMsg msg; // reduce stack allocation

    memset(&msg, 0, sizeof(msg)); // msg=SensorBugUplinkMsg_init_zero
    uint32_t noiseLevel = 0;
    uint32_t luxLevel = 0;
    struct bme680_field_data *bmeData;

    pb_ostream_t stream;

    debugprintf("# PrepareTxFrame\n");

    switch( port )
    {
    case 2:
    {
        stream = pb_ostream_from_buffer(AppData, sizeof(AppData));
        // We would prefer to concurrently do MIC and BME polling
        int bmeSched = 0;
        if (Settings.light_enabled)
            bmeSched = 1;
        if (Settings.mic_enabled)
            bmeSched = 2;



        if (Settings.light_enabled || Settings.mic_enabled) {
            // Enable Light/MIC Power
            setPin(DOMAIN1_EN, DOMAIN1_ON);
            // Wait for MIC and Light sensor to stabilize
            Task_sleep(MYMAX(MIC_STABILIZE_TIME_US, LIGHT_STABILIZE_TIME_US) / Clock_tickPeriod);
            if (Settings.light_enabled) {
                sampleLightStart();
                if (bmeSched == 1) {
                    bmeData = getBME();
                }
                luxLevel = sampleLightWaitResult();
            }
            // MIC takes a while to stabilize, whereas the light sensor is ready within 90us
            if (Settings.mic_enabled) {
                sampleNoiseStart();
                if (bmeSched == 2) {
                    bmeData = getBME();
                }
                noiseLevel = sampleNoiseWaitResult();
            }
            // Disable Light/MIC Power
            setPin(DOMAIN1_EN, DOMAIN1_OFF);
        }

        if (bmeSched == 0) {
            bmeData = getBME();
        }


        msg.counter        = counter++;
        msg.battery        = BoardGetBatteryVoltage();
        msg.light          = luxLevel;
        msg.pir_count      = getPIR();
        msg.motion_count   = getBMXInts();
        msg.temperature    = bmeData->temperature;
        msg.humidity       = bmeData->humidity;
        msg.pressure       = bmeData->pressure;
        msg.gas_resistance = bmeData->gas_resistance;
        msg.ambient_noise  = noiseLevel;
        msg.period         = Settings.report_period/1000;
        msg.motion_en      = Settings.motion_enabled;
        msg.light_en       = Settings.light_enabled;
        msg.mic_en         = Settings.mic_enabled;
        // Version info should probably only be sent once on join
        msg.sw_version_major = software_ver_major;
        msg.sw_version_minor = software_ver_minor;

        msg.has_counter = true;
        msg.has_battery = true;
        msg.has_light = Settings.light_enabled;
        msg.has_pir_count = true;
        msg.has_motion_count = Settings.motion_enabled;
        msg.has_temperature = true;
        msg.has_humidity = true;
        msg.has_pressure = true;
        msg.has_gas_resistance = true;
        msg.has_ambient_noise = Settings.mic_enabled;
        msg.has_period = true;
        msg.has_motion_en = true;
        msg.has_light_en = true;
        msg.has_mic_en = true;
        msg.has_sw_version_major = true;
        msg.has_sw_version_minor = true;

        if (!pb_encode(&stream, SensorBugUplinkMsg_fields, &msg)) {
            debugprintf("Failed to encode data for nanopb");
            // send anyways
        }

        AppDataSize = (uint8_t)stream.bytes_written;

        debugprintf("Counter: %d\r\n", msg.counter);
        debugprintf("Battery Voltage: %d mV\r\n", msg.battery);
        debugprintf("Light Level: %d\r\n", msg.light);
        debugprintf("PIR Level: %d\r\n", msg.pir_count);
        debugprintf("Motion Count: %d\r\n", msg.motion_count);
        debugprintf("Mic Level: %d\r\n", msg.ambient_noise);
        debugprintf("Temperature: %f degC\r\n", msg.temperature);
        debugprintf("Humidity: %f %%rH\r\n", msg.humidity);
        debugprintf("Pressure: %f hPa\r\n", msg.pressure);
        debugprintf("GasResistance: %f ohms\r\n", msg.gas_resistance);

        // TODO
        /* Avoid using measurements from an unstable heating setup */
        if(!(bmeData->status & BME680_GASM_VALID_MSK)){
            debugprintf("Gas Resistance is not Valid!\r\n");
        }

        debugprintf("Total Packet Size: %d bytes\r\n", AppDataSize);
    }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    debugprintf("# SendFrame\n");

    // AppDataSize must be set before calling TxPossible
    // So, PrepareFrame needs to have already been called.
    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    debugprintf("# OnTxNextPacketTimerEvent\n");
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            DeviceState = DEVICE_STATE_JOIN;
            SendOnJoin = true;
        }
    }
    setLed(Board_RLED, 1);
    TimerStart( &Led2Timer );

    setLed(Board_GLED, 1); // denote busy - turned off on send confirm
    Event_post(runtimeEvents, EVENT_STATECHANGE);
}

/*!
 * \brief Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent( void )
{
    TimerStop( &Led1Timer );
    // Switch LED 1 OFF
    setLed(Board_GLED, 0);
}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void )
{
    TimerStop( &Led2Timer );
    // Switch LED 2 OFF
    setLed(Board_RLED, 0);
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    debugprintf("# McpsConfirm\n");
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                debugprintf("# Got McpsConfirm: MCPS_UNCONFIRMED\n");
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                debugprintf("# Got McpsConfirm: MCPS_CONFIRMED\n");
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }

        setLed(Board_GLED, 0);
    }
    NextTx = true;
    Event_post(runtimeEvents, EVENT_STATECHANGE);
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    debugprintf("# McpsIndication\n");
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            debugprintf("# Got McpsIndication: MCPS_UNCONFIRMED\n");
            break;
        }
        case MCPS_CONFIRMED:
        {
            debugprintf("# Got McpsIndication: MCPS_CONFIRMED\n");
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            {
                static SensorBugDownlinkMsg msg;
                debugprintf("# Got Downlink Message\r\n");
                pb_istream_t stream = pb_istream_from_buffer(mcpsIndication->Buffer, mcpsIndication->BufferSize);
                memset(&msg, 0, sizeof(msg)); // msg=SensorBugDownlinkMsg_init_zero;
                pb_decode(&stream, SensorBugDownlinkMsg_fields, &msg);

                if (msg.has_cmd_reset && msg.cmd_reset) {
                    // Will never return from this
                    debugprintf("# Hard Resetting\r\n");
                    hardreset();
                }
                if (msg.has_period) {
                    UpdateReportPeriod(msg.period);
                }
                if (msg.has_motion_en) {
                    UpdateMotionEnabled(msg.motion_en);
                }
                if (msg.has_light_en) {
                    Settings.light_enabled = msg.light_en;
                }
                if (msg.has_mic_en) {
                    Settings.mic_enabled = msg.mic_en;
                }
            }
            break;
        default:
            break;
        }
    }

    // Switch LED 2 ON for each received downlink
    setLed(Board_RLED, 1);
    TimerStart( &Led2Timer );
    Event_post(runtimeEvents, EVENT_STATECHANGE);
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    debugprintf("# MlmeConfirm\n");
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
                debugprintf("# MlmeConfirm: Join Ok\n");
                DeviceState = SendOnJoin ? DEVICE_STATE_SEND:DEVICE_STATE_SLEEP;

                setLed(Board_GLED, 0);

#               ifdef DISABLE_LEDS
                disableLeds();
#               endif
            }
            else
            {
                // Join was not successful. Try to join again
                debugprintf("# MlmeConfirm: Join Failed\n");
                DeviceState = DEVICE_STATE_JOIN;

                setLed(Board_RLED, 1);
                TimerStart( &Led2Timer );
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            debugprintf("# MlmeConfirm: Link Check\n");
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
            }
            break;
        }
        default:
            break;
    }
    NextTx = true;
    Event_post(runtimeEvents, EVENT_STATECHANGE);
}


static void ButtonCallback(void) {
    Event_post(runtimeEvents, EVENT_BUTTONPRESSED);
}

static void printLorawanCred() {
    // Space after 0x, so that it is easy to copy-paste
    allprintf("# Software Version: %u.%u\n", software_ver_major, software_ver_minor);
    allprintf("# DevEUI: 0x %2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x\n", DevEui[0], DevEui[1], DevEui[2], DevEui[3], DevEui[4], DevEui[5], DevEui[6], DevEui[7]);
    allprintf("# AppKey: 0x %2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x%2.2x\n", AppKey[0], AppKey[1], AppKey[2], AppKey[3], AppKey[4], AppKey[5], AppKey[6], AppKey[7], AppKey[8], AppKey[9], AppKey[10], AppKey[11], AppKey[12], AppKey[13], AppKey[14], AppKey[15]);
}


/* Define Calibration Functions */
#if defined(CALIBRATION_MODE_LIGHT) || defined(CALIBRATION_MODE_NOISE)
static void lightCalibration() {
    debugprintf("Sampling Light\r\n");
    sampleLightStart();
    uint32_t mlux = sampleLightWaitResult();
    debugprintf("# Light = %d mlux = %f lux\r\n", mlux, (float)mlux / 1000.0);
}
static void noiseCalibration() {
    debugprintf("Sampling Noise\r\n");
    sampleNoiseStart();
    uint32_t mv = sampleNoiseWaitResult();
    debugprintf("# Noise = %d mV = %f V\r\n", mv, (double)mv / 1000.0);
}

static void calibrationMode() {
    // Setting callback to NULL will re-enable the push-to-bootload functionality
    // This allows for one press programming using the bootloader.
    setBtnCallback(NULL);
    // Turn on the red LED to indicate we are in calibration mode
    setLed(Board_RLED, 1);
    // Turn on power to the MIC and Light sensor
    setPin(DOMAIN1_EN, DOMAIN1_ON);
    // Wait for MIC and Light sensor to stabilize
    Task_sleep(MYMAX(MIC_STABILIZE_TIME_US, LIGHT_STABILIZE_TIME_US) / Clock_tickPeriod);

    // Loop forever in calibration loop
    while (true) {

#       ifdef CALIBRATION_MODE_LIGHT
        lightCalibration();
#       endif
#       ifdef CALIBRATION_MODE_NOISE
        noiseCalibration();
#       endif

    }// while(true)
}
#endif

void maintask(UArg arg0, UArg arg1)
{
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;

    Event_construct(&runtimeEventsStruct, NULL);
    runtimeEvents = Event_handle(&runtimeEventsStruct);

    BoardInitMcu( );
    BoardInitPeriph( );
    BoardInitSensors( Settings.motion_enabled );

    #ifdef USE_BOARD_UNIQUE_ID_DEV_EUI
    // Get the 15.4 MAC Addr as DevEuiMAC_15_4_Addr
    BoardGetUniqueId(DevEui);
    #endif

    debugprintf("# Board initialized\n");
    printLorawanCred();

#ifdef ENABLE_BLE_ADVERTISEMENT
    // Set the DevEUI and AppKey in the BLE payload
    ble_set_eui_payload(DevEui, AppKey);
#endif

#if defined(CALIBRATION_MODE_LIGHT) || defined(CALIBRATION_MODE_NOISE)
    calibrationMode();
#endif

    DeviceState = DEVICE_STATE_INIT;

    while( 1 )
    {
        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
                debugprintf("# DeviceState: DEVICE_STATE_INIT\n");
                LoRaMacPrimitives.MacMcpsConfirm    = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm    = MlmeConfirm;
                LoRaMacCallbacks.GetBatteryLevel    = BoardGetBatteryLevel;
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks );

                TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );
                // Listen for button press to trigger next packet instead of timer
                setBtnCallback(ButtonCallback);

                TimerInit( &Led1Timer, OnLed1TimerEvent );
                TimerSetValue( &Led1Timer, LED_ONTIME_MS );

                TimerInit( &Led2Timer, OnLed2TimerEvent );
                TimerSetValue( &Led2Timer, LED_ONTIME_MS );

                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );

                DeviceState = DEVICE_STATE_JOIN;
                break;
            }
            case DEVICE_STATE_JOIN:
            {
                debugprintf("# DeviceState: DEVICE_STATE_JOIN\n");
                setLed(Board_GLED, 1);
#if( OVER_THE_AIR_ACTIVATION != 0 )
                MlmeReq_t mlmeReq;

                // Initialize LoRaMac device unique ID
//                BoardGetUniqueId( DevEui );

                mlmeReq.Type = MLME_JOIN;

                mlmeReq.Req.Join.DevEui   = DevEui;
                mlmeReq.Req.Join.AppEui   = AppEui;
                mlmeReq.Req.Join.AppKey   = AppKey;
                mlmeReq.Req.Join.NbTrials = 3;

                if( NextTx == true )
                {
                    LoRaMacMlmeRequest( &mlmeReq );
                }
                DeviceState = DEVICE_STATE_SLEEP;
#else
                // Choose a random device address if not already defined in Commissioning.h
                if( DevAddr == 0 )
                {
                    // Random seed initialization
                    srand1( BoardGetRandomSeed( ) );

                    // Choose a random device address
                    DevAddr = randr( 0, 0x01FFFFFF );
                }

                mibReq.Type = MIB_NET_ID;
                mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_DEV_ADDR;
                mibReq.Param.DevAddr = DevAddr;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NWK_SKEY;
                mibReq.Param.NwkSKey = NwkSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_APP_SKEY;
                mibReq.Param.AppSKey = AppSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NETWORK_JOINED;
                mibReq.Param.IsNetworkJoined = true;
                LoRaMacMibSetRequestConfirm( &mibReq );

                DeviceState = DEVICE_STATE_SEND;
#endif
                break;
            }
            case DEVICE_STATE_SEND:
            {
                debugprintf("# DeviceState: DEVICE_STATE_SEND\n");
                if( NextTx == true )
                {
                    PrepareTxFrame( AppPort );
                    NextTx = SendFrame( );
                }

                DeviceState = DEVICE_STATE_CYCLE;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                debugprintf("# DeviceState: DEVICE_STATE_CYCLE\n");
                DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                TimerStart( &TxNextPacketTimer );
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                debugprintf("# DeviceState: DEVICE_STATE_SLEEP\n");
#ifdef WATCHDOG_ENABLED
                Watchdog_clear(watchdog);
#endif
                // Wake up through events
                UInt events = Event_pend(runtimeEvents, Event_Id_NONE, EVENT_STATECHANGE|EVENT_BUTTONPRESSED, BIOS_WAIT_FOREVER);
                // If only button press event (priority to STATECHANGE)
                if (events == EVENT_BUTTONPRESSED) {
                    DeviceState = DEVICE_STATE_BUTTON;
                }
                break;
            }
            case DEVICE_STATE_BUTTON:
            {
                debugprintf("# DeviceState: DEVICE_STATE_BUTTON\n");

#ifndef BUTTON_AS_RESET
                DeviceState = DEVICE_STATE_SLEEP;

                printLorawanCred();

#               ifdef ENABLE_BLE_ADVERTISEMENT
                ble_send_advertisement();
#               endif

                MibRequestConfirm_t mibReq;
                LoRaMacStatus_t status;

                mibReq.Type = MIB_NETWORK_JOINED;
                status = LoRaMacMibGetRequestConfirm( &mibReq );

                if( status == LORAMAC_STATUS_OK )
                {
                    if( mibReq.Param.IsNetworkJoined == true )
                    {
                        DeviceState = DEVICE_STATE_SEND;
                        NextTx = true;
                    }
                    else
                    {
                        DeviceState = DEVICE_STATE_JOIN;
                        SendOnJoin = true;
                    }
                }
                setLed(Board_RLED, 1);
                TimerStart( &Led2Timer );

                setLed(Board_GLED, 1); // denote busy - turned off on send confirm

                // If still depressed 1 sec later - reset into bootloader
                Task_sleep(TIME_MS * 1000);
                if (!getButtonState()) {
                    hardreset();
                }
#else
                // Will reset in 1 sec -- hold button to trigger bootload on boot
                Task_sleep(TIME_MS * 1000);
                hardreset();
#endif
                break;
            }
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }
    }

}

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams;

    /* Call board init functions */
    Board_initGeneral();
    Board_initI2C();
    Board_initSPI();
    Board_initUART();
    Board_initADC();
    Board_initADCBuf();
    Board_initWatchdog();

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000000 / Clock_tickPeriod;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &mainTaskStack;
    Task_construct(&mainTaskStruct, (Task_FuncPtr) maintask, &taskParams, NULL);

    /* Setup watchdog */
#ifdef WATCHDOG_ENABLED
    // Default Functionality:
    // * HardReset when watchdog is upset
    // * Allow stalling the watchdog when debugger is at breakpoint
    // * No callback
    watchdog = Watchdog_open(LORABUG_WATCHDOG0, NULL);
    if (watchdog == NULL) {
        System_abort("Failed to open watchdog");
    }
    Watchdog_clear(watchdog);
#endif

    /* Open and setup pins */
    setuppins();

    /* Open UART */
    setupuart();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
