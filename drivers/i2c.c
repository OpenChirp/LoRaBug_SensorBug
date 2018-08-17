/*
 * @file i2c.c
 *
 * @brief   Created on: Jul 13, 2018
 *          Author: Artur Balanuta <Artur [dot] Balanuta [at] Gmail [dot] com>
 */

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/I2C.h>
#include <stdlib.h>

#include <PERIPHERALS.h>
#include <io.h>

#include "i2c.h"

static I2C_Handle i2cHandle;
static I2C_Params i2cParams;
static I2C_Transaction i2cTransaction;

static int8_t rslt = 0;
static uint8_t* txBuf;


int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

	rslt = 0;

	i2cParams.bitRate = I2C_400kHz;
	i2cParams.transferMode = I2C_MODE_BLOCKING;

	I2C_Params_init(&i2cParams);
	i2cHandle = I2C_open(Board_I2C, &i2cParams);

	if(!i2cHandle) {
        rslt = -1;
    } else {
    	i2cTransaction.slaveAddress = dev_id;
    	i2cTransaction.writeBuf = &reg_addr;
    	i2cTransaction.writeCount = 1;
    	i2cTransaction.readBuf = reg_data;
    	i2cTransaction.readCount = len;

    	if(!I2C_transfer(i2cHandle, &i2cTransaction)) {
    		rslt = -1;
    	}
    }

	I2C_close(i2cHandle);
    return rslt;
}



int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

	rslt = 0;
	txBuf = malloc((len + 1) * sizeof(uint8_t));

	i2cParams.bitRate = I2C_400kHz;
	i2cParams.transferMode = I2C_MODE_BLOCKING;

	I2C_Params_init(&i2cParams);
	i2cHandle = I2C_open(Board_I2C, &i2cParams);

	if (i2cHandle == NULL) {
		rslt = -1;
	} else {
		i2cTransaction.slaveAddress = dev_id;
		i2cTransaction.writeBuf = txBuf;
		i2cTransaction.writeCount = len + 1;
		i2cTransaction.readBuf = NULL;
		i2cTransaction.readCount = 0;
		txBuf[0] = reg_addr;

		for(int j = 0; j < len; j++) {
			txBuf[1 + j] = reg_data[j];
		}

		if(!I2C_transfer(i2cHandle, &i2cTransaction)) {
			rslt = -1;
		}
	}

	I2C_close(i2cHandle);
	free(txBuf);
	return rslt;
}


void user_delay_ms(uint32_t period)
{
	Task_sleep(((period) * 1000) / Clock_tickPeriod);
}
