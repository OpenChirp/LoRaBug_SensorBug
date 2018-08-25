
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
#include <ti/drivers/I2C.h>
// #include <ti/drivers/Watchdog.h>

/* Board Header files */
#include "LORABUG.h"
#include "PERIPHERALS.h"

#include <string.h> // strlen in uartputs and LoRaWan code
#include <math.h>
#include <sensors.h>
#include "io.h"

/* LoRa Radio Header files */
#include "board.h" // The LoRaMac-node/src/boads/LoRaBug/board.h file
#include "radio.h"
#include "LoRaMac.h"
#include "Commissioning.h"

#define TASKSTACKSIZE   2048

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

#define LED_ONTIME_MS                               100

/* Runtime Events */
#define EVENT_STATECHANGE   Event_Id_00
#define EVENT_BUTTONPRESSED Event_Id_01

static Event_Struct runtimeEventsStruct;
static Event_Handle runtimeEvents;

/*------------------------------------------------------------------------*/
/*                     Configuration                                      */
/*------------------------------------------------------------------------*/

#define USE_BOARD_UNIQUE_ID_DEV_EUI

/**@def DISABLE_LEDS
 * When defined, the LEDs will be disabled after successfully
 * joining the LoRaWAN network.
 */
#define DISABLE_LEDS

/*------------------------------------------------------------------------*/
/*                      Start of LoRaWan Demo Code                        */
/*------------------------------------------------------------------------*/

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
//#define APP_TX_DUTYCYCLE                            5000
//#define APP_TX_DUTYCYCLE                            1000
#define APP_TX_DUTYCYCLE                            (1000*60*10)
/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
//#define APP_TX_DUTYCYCLE_RND                        1000
#define APP_TX_DUTYCYCLE_RND                        100

/*!
 * Default datarate
 */
//#define LORAWAN_DEFAULT_DATARATE                    DR_0
#define LORAWAN_DEFAULT_DATARATE                    DR_4

/*!
 * LoRaWAN confirmed messages
 */
//#define LORAWAN_CONFIRMED_MSG_ON                    true
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1
//#define LORAWAN_ADR_ON                              0


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
#define LORAWAN_APP_DATA_MAX_SIZE                           64

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
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;

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
static bool SendOnJoin = false;

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

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    static uint32_t counter = 1;
    uint16_t batteryVoltage = 0;
//    uint8_t batteryLevel = 0;
    //uint16_t micLevel = 0;
    uint16_t luxLevel = 0;
    uint8_t pirLevel = 0;
    uint8_t bmxInts = 0;
    struct bme680_field_data bmeData;

    debugprintf("# PrepareTxFrame\n");

    switch( port )
    {
    case 2:
    {
        // Layout Of Payload: [(counter-uint32), (batteryvoltage-uint16), ...]
        // Fields: counter, battery, lux, pir, accel, temp, humidity, pressure, gas
        // Types:  uint32, uint16, uint16, uint8, uint8, float32, float32, float32, float32

        batteryVoltage = BoardGetBatteryVoltage();
//        batteryLevel = BoardGetBatteryLevel();

        // Enable Sensors Power
        setPin(DOMAIN1_EN, DOMAIN1_ON);

        //micLevel = getMIC();
        luxLevel = getLUX();
        pirLevel = getPIR();
        bmeData = getBME();
        bmxInts = getBMXInts();

        // Disable Sensors Power
        setPin(DOMAIN1_EN, DOMAIN1_OFF);

        // Clean Buffer
        memset(AppData, '\0', sizeof(AppData));
        AppDataSize = 0;

        // Copy Counter
        memcpy(AppData, &counter, sizeof(counter));
        AppDataSize += sizeof(counter);
        debugprintf("Counter: %d\r\n", counter);
        counter++;

        // Copy Battery Voltage
        memcpy(AppData + AppDataSize, &batteryVoltage, sizeof(batteryVoltage));
        AppDataSize += sizeof(batteryVoltage);
        debugprintf("Battery Voltage: %d mV\r\n", batteryVoltage);

        // Copy Battery Level
        //memcpy(AppData + AppDataSize, &batteryLevel, sizeof(batteryLevel));
        //AppDataSize += sizeof(batteryLevel);
        //debugprintf("Battery Level: %d/254\r\n", batteryLevel);

        // Copy Mic Value
        //memcpy(AppData + AppDataSize, &micLevel, sizeof(micLevel));
        //AppDataSize += sizeof(micLevel);
        //debugprintf("Mic Level: %d\r\n", micLevel);

        // Copy Lux Value
        memcpy(AppData + AppDataSize, &luxLevel, sizeof(luxLevel));
        AppDataSize += sizeof(luxLevel);
        debugprintf("Lux Level: %d\r\n", luxLevel);

        // Copy PIR Value
        memcpy(AppData + AppDataSize, &pirLevel, sizeof(pirLevel));
        AppDataSize += sizeof(pirLevel);
        debugprintf("PIR Level: %d\r\n", pirLevel);

        // Copy Accelerometer Value
        memcpy(AppData + AppDataSize, &bmxInts, sizeof(bmxInts));
        AppDataSize += sizeof(bmxInts);
        debugprintf("Acc Interrupts: %d/255\r\n", bmxInts);

        // Copy Sensors Values, Temperature Humidity, Pressure and Gas Resistance
        memcpy(AppData + AppDataSize, &bmeData.temperature, sizeof(bmeData.temperature));
        AppDataSize += sizeof(bmeData.temperature);
        debugprintf("Temperature: %f degC\r\n", bmeData.temperature);

        memcpy(AppData + AppDataSize, &bmeData.humidity, sizeof(bmeData.humidity));
        AppDataSize += sizeof(bmeData.humidity);
        debugprintf("Humidity: %f %%rH\r\n", bmeData.humidity);

        memcpy(AppData + AppDataSize, &bmeData.pressure, sizeof(bmeData.pressure));
        AppDataSize += sizeof(bmeData.pressure);
        debugprintf("Pressure: %f hPa\r\n", bmeData.pressure);

        memcpy(AppData + AppDataSize, &bmeData.gas_resistance, sizeof(bmeData.gas_resistance));
        AppDataSize += sizeof(bmeData.gas_resistance);
        debugprintf("GasResistance: %f ohms\r\n", bmeData.gas_resistance);

        // TODO
        /* Avoid using measurements from an unstable heating setup */
        if(!(bmeData.status & BME680_GASM_VALID_MSK)){
            debugprintf("Gas Resistance is not Valid!\r\n");
        }

        debugprintf("Total Packet Size: %d bytes\r\n", AppDataSize);
    }
        break;
    case 224:
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
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

    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if( mcpsIndication->BufferSize == 1 )
            {
                AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
                setLed(Board_RLED, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 1 : 0);
            }
            break;
        case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( mcpsIndication->BufferSize == 4 ) &&
                    ( mcpsIndication->Buffer[0] == 0x01 ) &&
                    ( mcpsIndication->Buffer[1] == 0x01 ) &&
                    ( mcpsIndication->Buffer[2] == 0x01 ) &&
                    ( mcpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                }
            }
            else
            {
                ComplianceTest.State = mcpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = LORAWAN_APP_DATA_SIZE;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = mcpsIndication->BufferSize;

                    AppData[0] = 4;
                    for( uint8_t i = 1; i < AppDataSize; i++ )
                    {
                        AppData[i] = mcpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    break;
                case 6: // (ix)
                    {
                        MlmeReq_t mlmeReq;

                        // Disable TestMode and revert back to normal operation
                        IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                        AppPort = LORAWAN_APP_PORT;
                        AppDataSize = LORAWAN_APP_DATA_SIZE;
                        ComplianceTest.DownLinkCounter = 0;
                        ComplianceTest.Running = false;

                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                        LoRaMacMibSetRequestConfirm( &mibReq );

                        mlmeReq.Type = MLME_JOIN;

                        mlmeReq.Req.Join.DevEui = DevEui;
                        mlmeReq.Req.Join.AppEui = AppEui;
                        mlmeReq.Req.Join.AppKey = AppKey;
                        mlmeReq.Req.Join.NbTrials = 3;

                        LoRaMacMlmeRequest( &mlmeReq );
                        DeviceState = DEVICE_STATE_SLEEP;
                    }
                    break;
                case 7: // (x)
                    {
                        if( mcpsIndication->BufferSize == 3 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        ComplianceTest.State = 1;
                    }
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        }
    }

    // Switch LED 2 ON for each received downlink
//    GpioWrite( &Led2, 0 );
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
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
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

void printLorawanCred() {
    // Space after 0x, so that it is easy to copy-paste
    debugprintf("# DevEUI: 0x %2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X\n", DevEui[0], DevEui[1], DevEui[2], DevEui[3], DevEui[4], DevEui[5], DevEui[6], DevEui[7]);
    debugprintf("# AppKey: 0x %2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X\n", AppKey[0], AppKey[1], AppKey[2], AppKey[3], AppKey[4], AppKey[5], AppKey[6], AppKey[7], AppKey[8], AppKey[9], AppKey[10], AppKey[11], AppKey[12], AppKey[13], AppKey[14], AppKey[15]);
}

void maintask(UArg arg0, UArg arg1)
{
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;

    Event_construct(&runtimeEventsStruct, NULL);
    runtimeEvents = Event_handle(&runtimeEventsStruct);

    BoardInitMcu( );
    BoardInitPeriph( );
    BoardInitSensors( );

    #ifdef USE_BOARD_UNIQUE_ID_DEV_EUI
    // Get the 15.4 MAC Addr as DevEuiMAC_15_4_Addr
    BoardGetUniqueId(DevEui);
    #endif

    debugprintf("# Board initialized\n");
    printLorawanCred();

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
                DeviceState = DEVICE_STATE_SEND;
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
//                    PrepareTxFrame( AppPort );
                    NextTx = SendFrame( );
                }
                if( ComplianceTest.Running == true )
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = 5000; // 5000 ms
                }
                else
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                }
                DeviceState = DEVICE_STATE_CYCLE;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                debugprintf("# DeviceState: DEVICE_STATE_CYCLE\n");
                DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                TimerStart( &TxNextPacketTimer );
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                debugprintf("# DeviceState: DEVICE_STATE_SLEEP\n");
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

                DeviceState = DEVICE_STATE_SLEEP;

                printLorawanCred();
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
    // Board_initWatchdog();

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000000 / Clock_tickPeriod;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr) maintask, &taskParams,
                   NULL);

    /* Open and setup pins */
    setuppins();

    /* Open UART */
    setupuart();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
