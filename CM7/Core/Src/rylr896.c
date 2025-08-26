/*
 * rylr896.c
 *
 *  Created on: Aug 19, 2025
 *      Author: firaz
 */

#include "rylr896.h"
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"

/* @brief Initialize the RYLR896 module
 * @return Rylr896_Status_t: Status of the operation
 */
Rylr896_Status_t Rylr896Test(void){
	Rylr896_Status_t ret = Rylr896_ERROR;
	const uint8_t packetSize = strlen(TEST_COMMAND) + strlen(TERMINATOR);
	uint8_t uartTxBuffer[packetSize] = {};

	memcpy(uartTxBuffer, TEST_COMMAND, 2);
	strcat((char *)uartTxBuffer, TERMINATOR);
	printf("Sending command: %s\r\n", uartTxBuffer);

	ret = HAL_UART_Transmit(&huart8, uartTxBuffer, packetSize, HAL_MAX_DELAY);

	return ret;
}

/* @brief set the address for the RYLR896 module
 * @param address: The address to set (0~65535)
 * @return Rylr896_Status_t: Status of the operation
 */
Rylr896_Status_t Rylr896SetAddress(char *address){
	Rylr896_Status_t ret = Rylr896_ERROR;
	const uint8_t packetSize = strlen(AT) + strlen(ADDRESS) +
			strlen(SET_VALUE) + strlen(address) + strlen(TERMINATOR);
	uint8_t uartTxBuffer[packetSize] = {};

	memcpy(uartTxBuffer, AT, AT_PRIFEX_SIZE);
	strcat((char *)uartTxBuffer, ADDRESS);
	strcat((char *)uartTxBuffer, SET_VALUE);
	strcat((char*) uartTxBuffer, address);
	strcat((char *)uartTxBuffer, TERMINATOR);

	ret = HAL_UART_Transmit(&huart8, (uint8_t*) uartTxBuffer, packetSize, HAL_MAX_DELAY);

	return ret;
}

/* @brief Set the Network ID for the RYLR896 module
 * @param networkID: The network ID to set (0~16)
 * @return Rylr896_Status_t: Status of the operation
 */
Rylr896_Status_t Rylr896setNetworkID(char *networkID){
	Rylr896_Status_t ret = Rylr896_ERROR;
	const uint8_t packetSize = strlen(AT) + strlen(NETWORKID) +
			strlen(SET_VALUE) + strlen(networkID) + strlen(TERMINATOR);
	uint8_t uartTxBuffer[packetSize] = {};

	memcpy(uartTxBuffer, AT, AT_PRIFEX_SIZE);
	strcat((char *)uartTxBuffer, NETWORKID);
	strcat((char *)uartTxBuffer, SET_VALUE);
	strcat((char *)uartTxBuffer, networkID);
	strcat((char *)uartTxBuffer, TERMINATOR);

	ret = HAL_UART_Transmit(&huart8, (uint8_t*) uartTxBuffer, packetSize, HAL_MAX_DELAY);

	return ret;
}

/* @brief Set a parameter on the RYLR896 module
 * @param parameter: The parameter to set <Spreading Factor>,
 * <Bandwidth>,<Coding Rate>,<Programmed Preamble>
 *  <Spreading Factor>7~12, (default 12)
 *	<Bandwidth>0~9 list as below
 *	0 : 7.8KHz (not recommended, over spec.)
 *	1 : 10.4KHz (not recommended, over spec.)
 *	2 : 15.6KHz
 *	3 : 20.8 KHz
 *	4 : 31.25 KHz
 *	5 : 41.7 KHz
 *	6 : 62.5 KHz
 *	7 : 125 KHz (default).
 *	8 : 250 KHz
 *	9 : 500 KHz
 *	<Coding Rat>1~4, (default 1)
 *  <Programmed Preamble> 4~7(default 4)
 * @return Rylr896_Status_t: Status of the operation
 * Communication within 3 km: Recommend to set “AT + PARAMETER = 10,7,1,7”
 * More than 3 km: Recommend to set “ AT + PARAMETER = 12,4,1,7”
 */
Rylr896_Status_t Rylr896SetParameter(char *parameter){
	Rylr896_Status_t ret = Rylr896_ERROR;
	const uint8_t packetSize = strlen(AT) + strlen(PARAMETER) +
			strlen(SET_VALUE) + strlen(parameter) + strlen(TERMINATOR);
	uint8_t uartTxBuffer[packetSize] = {};
	memcpy(uartTxBuffer, AT, AT_PRIFEX_SIZE);
	strcat((char *)uartTxBuffer, PARAMETER);
	strcat((char *)uartTxBuffer, SET_VALUE);
	strcat((char *)uartTxBuffer, parameter);
	strcat((char *)uartTxBuffer, TERMINATOR);

	ret = HAL_UART_Transmit(&huart8, (uint8_t*) uartTxBuffer, packetSize, HAL_MAX_DELAY);

	return ret;
}


/* @brief Send a packet to the RYLR896 module
 * @param address: The address to send the packet to (0~65535)
 * @param payload_len: The length of the payload
 * @param payload: The actual payload data (Maximum 240 bytes)
 * @return Rylr896_Status_t: Status of the operation
 */
Rylr896_Status_t Rylr896Send(char *address, char *payload_len, char *payload){
	Rylr896_Status_t ret = Rylr896_ERROR;
	const uint8_t packetSize = strlen(AT) + strlen(SEND) + strlen(SET_VALUE) +
			strlen(address) + strlen(SEGMENT_SEPARATOR) + strlen(payload_len)
			+ strlen(SEGMENT_SEPARATOR) + strlen(payload) + strlen(TERMINATOR);
	uint8_t uartTxBuffer[packetSize] = {};

	memcpy(uartTxBuffer, AT, AT_PRIFEX_SIZE);
	strcat((char *)uartTxBuffer, SEND);
	strcat((char *)uartTxBuffer, SET_VALUE);
	strcat((char *)uartTxBuffer, address);
	strcat((char *)uartTxBuffer, SEGMENT_SEPARATOR);
	strcat((char *)uartTxBuffer, payload_len);
	strcat((char *)uartTxBuffer, SEGMENT_SEPARATOR);
	strcat((char *)uartTxBuffer, payload);
	strcat((char *)uartTxBuffer, TERMINATOR);

	ret = HAL_UART_Transmit(&huart8, (uint8_t*) uartTxBuffer, packetSize, HAL_MAX_DELAY);

	return ret;
}

/* @brief Receive a packet from the RYLR896 module
 * @param rxBuffer: The buffer to store the received data
 * @param bufferSize: The size of the buffer
 * @return Rylr896_Status_t: Status of the operation
 */
Rylr896_Status_t Rylr896Receive(uint8_t *rxBuffer, uint8_t bufferSize){
	Rylr896_Status_t ret = Rylr896_ERROR;
	const uint8_t packetSize = strlen(RX_PACKET_START) + strlen(RCV) + strlen(TERMINATOR);
	uint8_t uartTxBuffer[packetSize] = {};

	memcpy(uartTxBuffer, RX_PACKET_START, strlen(RX_PACKET_START));
	strcat((char *)uartTxBuffer, RCV);
	strcat((char *)uartTxBuffer, TERMINATOR);

	ret = HAL_UART_Transmit(&huart8, (uint8_t*) uartTxBuffer, packetSize, HAL_MAX_DELAY);
	if(ret != Rylr896_OK) {
		return ret;
	}

	ret = HAL_UART_Receive(&huart8, rxBuffer, bufferSize, HAL_MAX_DELAY);

	return ret;
}


