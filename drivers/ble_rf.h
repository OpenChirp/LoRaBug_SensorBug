/*
 * ble_rf.h
 *
 *	@brief  Created on: Jul 18, 2018
 *          Author: Artur Balanuta <Artur [dot] Balanuta [at] Gmail [dot] com>
 */

#ifndef SERVICES_BLE_RF_H_
#define SERVICES_BLE_RF_H_

#include <stdint.h>

void ble_set_eui_payload(uint8_t devEui[8], uint8_t appKey[16]);
void ble_send_advertisement();

#endif /* SERVICES_BLE_RF_H_ */
