/*
 * rylr896.h
 *
 *  Created on: Aug 19, 2025
 *      Author: firaz
 */

#ifndef RYLR896_H_
#define RYLR896_H_

#ifdef __cplusplus
extern "C" {
#endif

//AT Command Set
#define TEST_COMMAND			"AT"
#define AT						"AT+"

#define RESETART				"RESET"		//Software RESET
#define MODE					"MODE"		//Set the wireless work mode.
#define IPR						"IPR"		//Set the UART baud rate
#define PARAMETER				"PARAMETER"//Set the RF parameters
#define BAND					"BAND"		//Set RF Frequency
#define ADDRESS					"ADDRESS"	//Set the ADDRESS ID of module LoRa.
#define NETWORKID				"NETWORKID"	//Set the network ID.
#define CRFOP					"CRFOP"		//Set the RF output power.
#define SEND					"SEND"		//Send data to the appointed address by Command Mode.
#define RCV						"RCV"		//Show the received data actively
#define UID						"UID"		//To inquire module ID. 12BYTES
#define VER						"VER"
#define OK						"OK"
#define READY					"READY"

#define TERMINATOR				"\r\n"
#define CHECK					"?"
#define SET_VALUE				"="
#define RX_PACKET_START			"+"
#define SEGMENT_SEPARATOR		","

#define RYLR998_ADDRESS			0x02

#define AT_PRIFEX_SIZE			0x03
#define AT_TERMINATOR_SIZE		0x02
#define AT_SET_VALUE_SIZE		0x01
#define AT_OVERHEAD_SIZE		(AT_PRIFEX_SIZE + AT_TERMINATOR_SIZE + AT_SET_VALUE_SIZE)
#define AT_ADDRESS_SIZE			0x02

#define RESPONSE_OFFSET			0x01		//offset by 1 to ignore + character after checking it
#define ADDRESS_OFFSET			0x09		//offset to where the data is located in the address get command response

typedef enum
{
	Rylr896_OK = 0x00U,
	Rylr896_ERROR,
	Rylr896_BUSY,
	Rylr896_TIMEOUT
} Rylr896_Status_t;

Rylr896_Status_t Rylr896Test(void);
Rylr896_Status_t Rylr896SetAddress(char *address);
Rylr896_Status_t Rylr896setNetworkID(char *networkID);
Rylr896_Status_t Rylr896SetParameter(char *parameter);
Rylr896_Status_t Rylr896Send(char *address, char *payload_len, char *payload);

#ifdef __cplusplus
}
#endif

#endif /* RYLR896_H_ */



