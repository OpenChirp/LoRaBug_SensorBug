/*
 * ble_rf.c
 *
 *	@brief  Created on: Jul 18, 2018
 *          Author: Artur Balanuta <Artur [dot] Balanuta [at] Gmail [dot] com>
 */

#include <ti/drivers/rf/RF.h>

#include "ble_rf.h"

#include "driverlib/rf_ble_cmd.h"
#include "driverlib/rf_common_cmd.h"

#include <rf_patches/rf_patch_cpe_ble.h>
#include <rf_patches/rf_patch_rfe_ble.h>

#define ADV_CHANNEL_START 37
#define ADV_CHANNEL_END 39

#define EUI_AD_LEN 8+1
#define EUI_AD_TYPE 0xFA // Invalid EIR on purpose
#define EUI_PAYLOAD 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // to be set later
#define EUI_AD EUI_AD_LEN, EUI_AD_TYPE, EUI_PAYLOAD

#define AKEY_AD_LEN 16+1
#define AKEY_AD_TYPE 0xFB // Invalid EIR on purpose
#define AKEY_PAYLOAD 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // to be set later
#define AKEY_AD AKEY_AD_LEN, AKEY_AD_TYPE, AKEY_PAYLOAD

#define NAME_AD_LEN	1+1
#define NAME_AD_TYPE 0x09 // Complete Local Name AD Type
#define NAME_PAYLOAD 0x4c// 'L' in HEX, LoRaBUG does not fit because of the AppKey
#define NAME_AD NAME_AD_LEN, NAME_AD_TYPE, NAME_PAYLOAD

#define AD_PAYLOAD_LEN (EUI_AD_LEN + AKEY_AD_LEN + NAME_AD_LEN + 3)

static uint8_t adv_payload_len = AD_PAYLOAD_LEN;
static uint8_t adv_payload[] = {EUI_AD, AKEY_AD, NAME_AD};
static uint8_t advAddress[] = {0x55, 0x44, 0x33, 0x22, 0x11, 0x00};

// Overrides for CMD_RADIO_SETUP
static uint32_t pOverrides[] =
{
    // override_use_patch_ble_1mbps.xml
    // PHY: Use MCE ROM, RFE RAM patch
    MCE_RFE_OVERRIDE(0,0,0,1,0,0),
    // override_synth_ble_1mbps.xml
    // Synth: Set recommended RTRIM to 5
    HW_REG_OVERRIDE(0x4038,0x0035),
    // Synth: Set Fref to 3.43 MHz
    (uint32_t)0x000784A3,
    // Synth: Set loop bandwidth after lock to 80 kHz
    (uint32_t)0xA47E0583,
    // Synth: Set loop bandwidth after lock to 80 kHz
    (uint32_t)0xEAE00603,
    // Synth: Set loop bandwidth after lock to 80 kHz
    (uint32_t)0x00010623,
    // Synth: Configure PLL bias
    HW32_ARRAY_OVERRIDE(0x405C,1),
    // Synth: Configure PLL bias
    (uint32_t)0x1801F800,
    // Synth: Configure PLL latency
    HW32_ARRAY_OVERRIDE(0x402C,1),
    // Synth: Configure PLL latency
    (uint32_t)0x00608402,
    // Synth: Use 24 MHz XOSC as synth clock, enable extra PLL filtering
    (uint32_t)0x02010403,
    // Synth: Configure extra PLL filtering
    HW32_ARRAY_OVERRIDE(0x4034,1),
    // Synth: Configure extra PLL filtering
    (uint32_t)0x177F0408,
    // Synth: Configure extra PLL filtering
    (uint32_t)0x38000463,
    // override_phy_ble_1mbps.xml
    // Tx: Configure symbol shape for BLE frequency deviation requirements
    (uint32_t)0x013800C3,
    // Rx: Configure AGC reference level
    HW_REG_OVERRIDE(0x6088, 0x0045),
    // Tx: Configure pilot tone length to ensure stable frequency before start of packet
    HW_REG_OVERRIDE(0x52AC, 0x0360),
    // Tx: Compensate timing offset to match new pilot tone setting
    (uint32_t)0x01AD02A3,
    // Tx: Compensate timing offset to match new pilot tone setting
    (uint32_t)0x01680263,
    // override_frontend_id.xml
    (uint32_t)0xFFFFFFFF,
};


// Overrwrite the devEui into the  BLE Payload
void ble_set_eui_payload(uint8_t devEui[8], uint8_t appKey[16]){
	// skip first two bytes are the len and AD Type for the devEUI, previously set
	memcpy(adv_payload+2, devEui, 8);
	// skip devEui fields, len and AD Type, previously
	memcpy(adv_payload+12, appKey, 16);
}
void ble_send_advertisement() {
    RF_Object ble_rfObj;
    RF_Handle ble_handle;
    RF_Params ble_params;

    RF_Mode RF_ble =
    {
        .rfMode      = RF_MODE_BLE,
        .cpePatchFxn = &rf_patch_cpe_ble,
        .mcePatchFxn = 0,
        .rfePatchFxn = &rf_patch_rfe_ble,
    };

    rfc_CMD_RADIO_SETUP_t RF_cmdRadioSetup =
    {
        .commandNo = 0x0802,
        .status = 0x0000,
        .pNextOp = 0,
        .startTime = 0x00000000,
        .startTrigger.triggerType = 0x0,
        .startTrigger.bEnaCmd = 0x0,
        .startTrigger.triggerNo = 0x0,
        .startTrigger.pastTrig = 0x0,
        .condition.rule = 0x1,
        .condition.nSkip = 0x0,
        .mode = 0x00,
        .__dummy0 = 0x00,
        .config.frontEndMode = 0x0,
        .config.biasMode = 0x0,
        .config.analogCfgMode = 0x0,
        .config.bNoFsPowerUp = 0x0,
        .txPower = 0x9330,
        .pRegOverride = pOverrides,
    };

    RF_Params_init(&ble_params);
    ble_params.nInactivityTimeout = 200;

    ble_handle = RF_open(&ble_rfObj, &RF_ble, (RF_RadioSetup*)&RF_cmdRadioSetup, &ble_params);

    // CMD_FS
    // Frequency Synthesizer Programming Command
    rfc_CMD_FS_t RF_cmdFs =
    {
        .commandNo = 0x0803,
        .status = 0x0000,
        .pNextOp = 0,
        .startTime = 0x00000000,
        .startTrigger.triggerType = 0x0,
        .startTrigger.bEnaCmd = 0x0,
        .startTrigger.triggerNo = 0x0,
        .startTrigger.pastTrig = 0x0,
        .condition.rule = 0x1,
        .condition.nSkip = 0x0,
        .frequency = 0x097A,
        .fractFreq = 0x0000,
        .synthConf.bTxMode = 0x0,
        .synthConf.refFreq = 0x0,
        .__dummy0 = 0x00,
        .__dummy1 = 0x00,
        .__dummy2 = 0x00,
        .__dummy3 = 0x0000,
    };

    RF_runCmd(ble_handle, (RF_Op *) &RF_cmdFs, RF_PriorityNormal, NULL, 0);

    // Send message on each advertisement channel
    for (int i = ADV_CHANNEL_START; i <= ADV_CHANNEL_END; i++) {
        // Structure for CMD_BLE_ADV_NC.pParams
        rfc_bleAdvPar_t bleAdvPar =
        {
            .pRxQ = 0,
            .rxConfig.bAutoFlushIgnored = 0x0,
            .rxConfig.bAutoFlushCrcErr = 0x0,
            .rxConfig.bAutoFlushEmpty = 0x0,
            .rxConfig.bIncludeLenByte = 0x0,
            .rxConfig.bIncludeCrc = 0x0,
            .rxConfig.bAppendRssi = 0x0,
            .rxConfig.bAppendStatus = 0x0,
            .rxConfig.bAppendTimestamp = 0x0,
            .advConfig.advFilterPolicy = 0x0,
            .advConfig.deviceAddrType = 0x0,
            .advConfig.peerAddrType = 0x0,
            .advConfig.bStrictLenFilter = 0x0,
            .advConfig.rpaMode = 0x0,
            .advLen = adv_payload_len,
            .scanRspLen = 0x00,
            .pAdvData = adv_payload,
            .pScanRspData = 0,
            .pDeviceAddress = (uint16_t*)advAddress,
            .pWhiteList = 0,
            .__dummy0 = 0x0000,
            .__dummy1 = 0x00,
            .endTrigger.triggerType = 0x1,
            .endTrigger.bEnaCmd = 0x0,
            .endTrigger.triggerNo = 0x0,
            .endTrigger.pastTrig = 0x0,
            .endTime = 0x00000000,
        };


        // CMD_BLE_ADV_NC
        // BLE Non-Connectable Advertiser Command
        rfc_CMD_BLE_ADV_NC_t RF_cmdBleAdvNc =
        {
            .commandNo = 0x1805,
            .status = 0x0000,
            .pNextOp = 0,
            .startTime = 0x00000000,
            .startTrigger.triggerType = 0x0,
            .startTrigger.bEnaCmd = 0x0,
            .startTrigger.triggerNo = 0x0,
            .startTrigger.pastTrig = 0x0,
            .condition.rule = 0x1,
            .condition.nSkip = 0x0,
            .channel = i,
            .whitening.init = 0x66,
            .whitening.bOverride = 0x1,
            .pParams = &bleAdvPar,
            .pOutput = 0,
        };

        RF_runCmd(ble_handle, (RF_Op *) &RF_cmdBleAdvNc, RF_PriorityNormal, NULL, 0);
    }

    RF_close(ble_handle);
}
