/**
  ******************************************************************************
  * @file           : uart_protocol.h
  * @brief          : UART Protocol for PC Communication
  ******************************************************************************
  * 
  * Protocol Format:
  * [START][CMD][LEN][DATA...][CHECKSUM]
  * 
  * START:    0xAA
  * CMD:      1 byte command
  * LEN:      1 byte data length (0-255)
  * DATA:     LEN bytes of payload
  * CHECKSUM: XOR of CMD, LEN and all DATA bytes
  * 
  * Commands:
  * 0x01 - UART_CMD_CONNECT:     Enable UART communication
  * 0x02 - UART_CMD_DISCONNECT:  Disable UART communication
  * 0x03 - UART_CMD_SEND:        Send data via LoRa
  * 0x04 - UART_CMD_RECEIVE:     Received data notification (device->PC)
  * 0x05 - UART_CMD_SET_PARAM:   Set LoRa parameter
  * 0x06 - UART_CMD_GET_PARAM:   Get LoRa parameters
  * 0x07 - UART_CMD_GET_HISTORY: Export all received messages
  * 0x08 - UART_CMD_ACK:         Acknowledgment response
  * 0x09 - UART_CMD_NACK:        Negative acknowledgment
  * 
  * Parameter IDs (for SET_PARAM/GET_PARAM):
  * 0x01 - Frequency (4 bytes, Hz)
  * 0x02 - TX Power (1 byte, dBm)
  * 0x03 - Spreading Factor (1 byte, 5-12)
  * 0x04 - Bandwidth (1 byte, index 0-9)
  * 0x05 - Coding Rate (1 byte, 1-4)
  * 0x06 - Sync Word (2 bytes)
  * 
  ******************************************************************************
  */

#ifndef __UART_PROTOCOL_H
#define __UART_PROTOCOL_H

#include "main.h"

/* ======================== Protocol Constants ======================== */

#define UART_START_BYTE         0xAA
#define UART_MAX_DATA_LEN       255
#define UART_RX_BUFFER_SIZE     300

/* ======================== Commands ======================== */

#define UART_CMD_CONNECT        0x01
#define UART_CMD_DISCONNECT     0x02
#define UART_CMD_SEND           0x03
#define UART_CMD_RECEIVE        0x04
#define UART_CMD_SET_PARAM      0x05
#define UART_CMD_GET_PARAM      0x06
#define UART_CMD_GET_HISTORY    0x07
#define UART_CMD_ACK            0x08
#define UART_CMD_NACK           0x09

/* ======================== Parameter IDs ======================== */

#define PARAM_FREQUENCY         0x01
#define PARAM_TX_POWER          0x02
#define PARAM_SF                0x03
#define PARAM_BW                0x04
#define PARAM_CR                0x05
#define PARAM_SYNC_WORD         0x06

/* ======================== State ======================== */

extern volatile bool UART_Connected;

/* ======================== Functions ======================== */

void UART_Protocol_Init(void);
void UART_Protocol_Process(void);

// Send functions
void UART_SendAck(uint8_t cmd);
void UART_SendNack(uint8_t cmd, uint8_t errorCode);
void UART_SendReceived(uint8_t* data, uint8_t len, int8_t rssi, int8_t snr);
void UART_SendParams(void);
void UART_SendHistoryMessage(uint8_t index, uint8_t* data, uint8_t len, int8_t rssi);

// For interrupt handler
void UART_RxCallback(uint8_t data);

#endif /* __UART_PROTOCOL_H */
