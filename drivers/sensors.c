/*
 * @file sensors.c
 *
 * @brief   Created on: Jul 10, 2018
 *          Author: Artur Balanuta <Artur [dot] Balanuta [at] Gmail [dot] com>
 */

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/drivers/ADC.h>

#include <PERIPHERALS.h>
#include <io.h>

#include "sensors.h"
#include "i2c.h"
#include "bme680.h"
#include "bmi160.h"


volatile static uint8_t pirCount = 0;
volatile static uint8_t bmxCount = 0;

static PIN_Handle PinHandle;
static PIN_State PinState;

PIN_Config pinTable[] = {
//	PIR_OUT | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_POSEDGE,
	PIR_OUT | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_POSEDGE,
//	BMX_INT1 | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_POSEDGE,
	PIN_TERMINATE
};

static void PIRCallback(PIN_Handle handle, PIN_Id pinId) {

	if (pinId == PIR_OUT && pirCount < 0xFF)
		pirCount++;

	if (pinId == BMX_INT1 && bmxCount< 0xFF)
		bmxCount++;
}

void BoardInitSensors(void){

    // Adds PIR Interrupt Callback
    PinHandle = PIN_open(&PinState, pinTable);
    if (!PinHandle) {
        System_abort("Error Opening Pins\n");
    }
    if (PIN_registerIntCb(PinHandle, PIRCallback) != PIN_SUCCESS) {
        System_printf("Failed to register pir int callback\n");
    }

//    struct bmi160_dev sensor;
//
//	sensor.id = BMI160_I2C_ADDR;
//	sensor.interface = BMI160_I2C_INTF;
//	sensor.read = user_i2c_read;
//	sensor.write = user_i2c_write;
//	sensor.delay_ms = user_delay_ms;
//
//	int8_t rslt = BMI160_OK;
//	rslt = bmi160_init(&sensor);


}

uint16_t getMIC(void){

    ADC_Params   params;
    ADC_Handle adc;
    int_fast16_t res;
    uint16_t adcValue, minV = 0xFFFF, maxV = 0, count = 0;
    uint32_t sampleSum = 0;

    ADC_Params_init(&params);
    adc = ADC_open(ADC_INDEX_MIC, &params);

    if (adc == NULL) {
        uartprintf("ADC err\r\n");
        System_abort("ADC err\n");
    }

    while(count < MIC_SAMPLES) {
        res = ADC_convert(adc, &adcValue);
        if (res == ADC_STATUS_SUCCESS) {
            if(maxV < adcValue) maxV = adcValue;
            if(minV > adcValue) minV = adcValue;
            sampleSum += adcValue;
            count++;
        }
        else {
            uartprintf("ADConverr\r\n");
            System_abort("ADC err\n");
        }
    }
    ADC_close(adc);
    uartprintf("MIC min: %d\r\n" , minV);
    uartprintf("MIC max: %d\r\n", maxV);
    uartprintf("MIC max-min: %d\r\n", maxV-minV);
    uartprintf("MIC avg: %d in %d samples\r\n", (sampleSum/MIC_SAMPLES), count);

    //return (uint16_t)(maxV-minV);
    return (uint16_t)(sampleSum/MIC_SAMPLES);
}

uint16_t getLUX(void){
    ADC_Params   params;
    ADC_Handle adc;

    int_fast16_t res;
    uint16_t adcValue, count = 0;
    int32_t sampleSum = 0;

    ADC_Params_init(&params);
    adc = ADC_open(ADC_INDEX_LUX, &params);

    if (adc == NULL) {
        uartprintf("ADC err\r\n");
        System_abort("ADC err\n");
    }

    while(count < LUX_SAMPLES) {
        res = ADC_convert(adc, &adcValue);
        if (res == ADC_STATUS_SUCCESS) {
            sampleSum += adcValue;
            count++;
        }
        else {
            uartprintf("ADConverr\r\n");
            System_abort("ADC err\n");
        }

    }
    ADC_close(adc);

    sampleSum -= (uint32_t)(LUX_AVG_MIN_VALUE * LUX_SAMPLES);

    // Test for edge cases
    if (sampleSum <= 0){
    	return 0x00;
    }
    else if (sampleSum >= (int32_t)(LUX_SAMPLES * LUX_AVG_MAX_VALUE) ){
    	return 0xff;
    }

    return (uint16_t)(sampleSum/(LUX_SAMPLES/LUX_SCALE_MULTIPLIER)); // Returns a value that maximizes the uint16 range

}

uint8_t getPIR(void){
    uint8_t tmp = pirCount;
    pirCount = 0;
    return tmp;
}

uint8_t getBMXInts(void){
    uint8_t tmp = bmxCount;
    bmxCount = 0;
    return tmp;
}

struct bme680_field_data getBME(void) {

	// TODO Refactor

	struct bme680_dev gas_sensor;
	struct bme680_field_data data = {0};
	int8_t rslt = BME680_OK;
	uint16_t meas_period;
	uint8_t set_required_settings;

	//Configuring I2C communication interface
	gas_sensor.dev_id = BME680_I2C_ADDR_SECONDARY;
	gas_sensor.intf = BME680_I2C_INTF;
	gas_sensor.read = user_i2c_read;
	gas_sensor.write = user_i2c_write;
	gas_sensor.delay_ms = user_delay_ms;


	//Initializing object to default values
	rslt = bme680_init(&gas_sensor);
	if(rslt != BME680_OK){
		uartprintf("BME680 Init fail\r\n");
		return data;
	}

	/* Set the temperature, pressure and humidity settings */
	gas_sensor.tph_sett.os_hum = BME680_OS_2X;
	gas_sensor.tph_sett.os_pres = BME680_OS_4X;
	gas_sensor.tph_sett.os_temp = BME680_OS_8X;
	gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

	/* Set the remaining gas sensor settings and link the heating profile */
	gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
	gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
	gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */
	gas_sensor.power_mode = BME680_FORCED_MODE;

	/* Set the required sensor settings needed */
	set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL
			| BME680_GAS_SENSOR_SEL;

	/* Set the desired sensor configuration */
	if(bme680_set_sensor_settings(set_required_settings,&gas_sensor) != 0) {
		uartprintf("BME680 Sensor settings set fail\r\n");
		return data;
	}

	/* Set the power mode */
	if(bme680_set_sensor_mode(&gas_sensor) != 0) {
		uartprintf("BME680 Power mode fail \r\n");
	}

	bme680_get_profile_dur(&meas_period, &gas_sensor);
	user_delay_ms(meas_period); /* Delay till the measurement is ready */

	bme680_get_sensor_data(&data, &gas_sensor);
	data.pressure = data.pressure / 100.0; // result is x100

	return data;
}

void getBMX(void){

	/* After the above function call, accel and gyro parameters in the device structure
	are set with default values, found in the datasheet of the sensor */
}



