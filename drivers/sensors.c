/*
 * @file sensors.c
 *
 * @brief   Created on: Jul 10, 2018
 *          Author: Artur Balanuta <Artur [dot] Balanuta [at] Gmail [dot] com>
 */

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/drivers/ADC.h>
#include <ti/sysbios/hal/Hwi.h>
#include <stdint.h>

#include <PERIPHERALS.h>
#include <io.h>

#include "i2c.h"
#include "bme680.h"
#include "bmi160.h"

#include "sensors.h"

volatile static uint32_t pirCount = 0;
volatile static uint32_t bmxCount = 0;

static PIN_Handle PinHandle;
static PIN_State PinState;

static const PIN_Config pinTable[] = {
    PIR_OUT  | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_POSEDGE, // PIR seems to push up
    BMX_INT1 | PIN_INPUT_EN | PIN_NOPULL   | PIN_IRQ_NEGEDGE, // BMX is configured to push-pull
    PIN_TERMINATE
};

static void SensorIntHandler(PIN_Handle handle, PIN_Id pinId) {

    if (pinId == PIR_OUT && pirCount < UINT32_MAX)
        pirCount++;

    if (pinId == BMX_INT1 && bmxCount < UINT32_MAX)
        bmxCount++;
}

void BoardInitSensors(bool motion_en) {

    // Adds PIR Interrupt Callback
    PinHandle = PIN_open(&PinState, pinTable);
    if (!PinHandle) {
        System_abort("Error Opening Pins\n");
    }
    if (PIN_registerIntCb(PinHandle, SensorIntHandler) != PIN_SUCCESS) {
        System_abort("Failed to register pir int callback\n");
    }

    if (motion_en) {
        setupBMI(true);  // Initialize the Accelerometer in Low-Power mode with Interrupts
    }
    initBMM();           // Initialize the Magnetometer in Suspend-Modestruct bmi160_int_settg int_config;

}

uint16_t getLUX(void){
    ADC_Params params;
    ADC_Handle adc;

    int_fast16_t res;
    uint16_t adcValue, count = 0;
    int32_t sampleSum = 0;

    ADC_Params_init(&params);
    adc = ADC_open(ADC_INDEX_LUX, &params);

    if (adc == NULL) {
        uartprintf("ADC err\r\n");
        //System_abort("ADC err\n");
    }

    while(count < LUX_SAMPLES) {
        res = ADC_convert(adc, &adcValue);
        if (res == ADC_STATUS_SUCCESS) {
            sampleSum += adcValue;
            count++;
        }
        else {
            uartprintf("ADConverr\r\n");
            //System_abort("ADC err\n");
        }

    }
    ADC_close(adc);

    sampleSum -= (uint32_t)(LUX_AVG_MIN_VALUE * LUX_SAMPLES);

    // Test for edge cases
    if (sampleSum <= 0){
        return 0x0000;
    }
    else if (sampleSum >= (int32_t)(LUX_SAMPLES * LUX_AVG_MAX_VALUE) ){
        return 0xffff;
    }

    return (uint16_t)(sampleSum/(LUX_SAMPLES/LUX_SCALE_MULTIPLIER)); // Returns a value that maximizes the uint16 range

}

uint32_t getPIR(void) {
    UInt key = Hwi_disable();
    uint32_t tmp = pirCount;
    pirCount = 0;
    Hwi_restore(key);
    return tmp;
}

uint32_t getBMXInts(void) {
    UInt key = Hwi_disable();
    uint32_t tmp = bmxCount;
    bmxCount = 0;
    Hwi_restore(key);
    return tmp;
}

struct bme680_field_data getBME(void) {

    struct bme680_dev gas_sensor;
    struct bme680_field_data data;
    int8_t rslt = BME680_OK;
    uint16_t meas_period;
    uint8_t set_required_settings;

    data.status = (uint8_t)BME680_E_NULL_PTR;

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

void setupBMI(bool enabled) {

    struct bmi160_dev sensor;
    struct bmi160_int_settg int_config;
    int8_t rslt = BMI160_OK;

    sensor.id = BMI160_I2C_ADDR;
    sensor.interface = BMI160_I2C_INTF;
    sensor.read = user_i2c_read;
    sensor.write = user_i2c_write;
    sensor.delay_ms = user_delay_ms;

    rslt = bmi160_init(&sensor);
    if(rslt != BMI160_OK){
        uartprintf("BMI Init fail\r\n");
        return;
    }

    /* Select the power mode */
    sensor.accel_cfg.power = enabled?BMI160_ACCEL_LOWPOWER_MODE:BMI160_ACCEL_SUSPEND_MODE;
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_0_78HZ;
    sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    sensor.accel_cfg.bw = BMI160_ACCEL_BW_OSR4_AVG1;

    // Gyro is suspended by Default
    //sensor.gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;

    /*  Set the Power mode  */
    rslt = bmi160_set_power_mode(&sensor);
    if(rslt != BMI160_OK){
        uartprintf("BMI fail setting power\r\n");
        return;
    }

    /* Select the Interrupt channel/pin */
    int_config.int_channel = BMI160_INT_CHANNEL_1;// Interrupt channel/pin 1

    /* Select the Interrupt type */
    int_config.int_type = BMI160_ACC_ANY_MOTION_INT;// Choosing Any motion interrupt
    /* Select the interrupt channel/pin settings */
    int_config.int_pin_settg.output_en = BMI160_ENABLE;// Enabling interrupt pins to act as output pin
    int_config.int_pin_settg.output_mode = BMI160_DISABLE;// Choosing push-pull mode for interrupt pin
    int_config.int_pin_settg.output_type = BMI160_DISABLE;// Choosing active low output
    int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE;// Choosing edge triggered output
    int_config.int_pin_settg.input_en = BMI160_DISABLE;// Disabling interrupt pin to act as input
    int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE;// non-latched output

    /* Select the Any-motion interrupt parameters */
    int_config.int_type_cfg.acc_any_motion_int.anymotion_en = BMI160_ENABLE;// 1- Enable the any-motion, 0- disable any-motion
    int_config.int_type_cfg.acc_any_motion_int.anymotion_x = BMI160_ENABLE;// Enabling x-axis for any motion interrupt
    int_config.int_type_cfg.acc_any_motion_int.anymotion_y = BMI160_ENABLE;// Enabling y-axis for any motion interrupt
    int_config.int_type_cfg.acc_any_motion_int.anymotion_z = BMI160_ENABLE;// Enabling z-axis for any motion interrupt
    int_config.int_type_cfg.acc_any_motion_int.anymotion_dur = 0;// any-motion duration
    int_config.int_type_cfg.acc_any_motion_int.anymotion_thr = 20;// (2-g range) -> (slope_thr) * 3.91 mg, (4-g range) -> (slope_thr) * 7.81 mg, (8-g range) ->(slope_thr) * 15.63 mg, (16-g range) -> (slope_thr) * 31.25 mg

    /* Set the Any-motion interrupt */
    //rslt = bmi160_set_int_config(&int_config, &sensor); /* sensor is an instance of the structure bmi160_dev  */
    rslt = set_accel_any_motion_int(&int_config, &sensor);

    if(rslt != BMI160_OK){
        uartprintf("BMI fail setting Any-motion interrupt\r\n");
        return;
    }

}

void initBMM(void){

//  struct bmi160_dev sensor;
//
//  sensor.id = BMI160_I2C_ADDR;
//  sensor.interface = BMI160_I2C_INTF;
//  sensor.read = user_i2c_read;
//  sensor.write = user_i2c_write;
//  sensor.delay_ms = user_delay_ms;
//
//  int8_t rslt = BMI160_OK;
//  rslt = bmi160_init(&sensor);
//  /* After the above function call, accel and gyro parameters in the device structure
//  are set with default values, found in the datasheet of the sensor */
}



