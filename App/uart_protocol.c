/**
  ******************************************************************************
  * @file           : uart_protocol.c
  * @brief          : UART Protocol Implementation
  ******************************************************************************
  */

#include "uart_protocol.h"
#include "e22.h"
#include "app.h"

/* ======================== State ======================== */

volatile bool UART_Connected = false;

/* ======================== Parser State Machine ======================== */

typedef enum {
    PARSER_WAIT_START,
    PARSER_WAIT_CMD,
    PARSER_WAIT_LEN,
    PARSER_WAIT_DATA,
    PARSER_WAIT_CHECKSUM
} ParserState_t;

static ParserState_t parserState = PARSER_WAIT_START;
static uint8_t rxCmd;
static uint8_t rxLen;
static uint8_t rxDataIndex;
static uint8_t rxData[UART_MAX_DATA_LEN];
static uint8_t rxChecksum;

/* ======================== TX Buffer ======================== */

static uint8_t txBuffer[UART_RX_BUFFER_SIZE];

/* ======================== Helper Functions ======================== */

static void UART_SendByte(uint8_t data)
{
    while(!LL_USART_IsActiveFlag_TXE(USART1));
    LL_USART_TransmitData8(USART1, data);
}

static void UART_SendPacket(uint8_t cmd, uint8_t* data, uint8_t len)
{
    uint8_t checksum = cmd ^ len;
    
    UART_SendByte(UART_START_BYTE);
    UART_SendByte(cmd);
    UART_SendByte(len);
    
    for(uint8_t i = 0; i < len; i++) {
        UART_SendByte(data[i]);
        checksum ^= data[i];
    }
    
    UART_SendByte(checksum);
}

/* ======================== Command Handlers ======================== */

static void HandleConnect(void)
{
    UART_Connected = true;
    UART_SendAck(UART_CMD_CONNECT);
}

static void HandleDisconnect(void)
{
    UART_Connected = false;
    UART_SendAck(UART_CMD_DISCONNECT);
}

static void HandleSend(uint8_t* data, uint8_t len)
{
    if(len == 0 || len > E22_MAX_PACKET_SIZE) {
        UART_SendNack(UART_CMD_SEND, 0x01);  // Invalid length
        return;
    }
    
    E22_Send(data, len);
    UART_SendAck(UART_CMD_SEND);
}

static void HandleSetParam(uint8_t* data, uint8_t len)
{
    if(len < 2) {
        UART_SendNack(UART_CMD_SET_PARAM, 0x01);
        return;
    }
    
    uint8_t paramId = data[0];
    
    switch(paramId) {
        case PARAM_FREQUENCY:
            if(len >= 5) {
                uint32_t freq = ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) |
                               ((uint32_t)data[3] << 8) | data[4];
                E22_SetFrequency(freq);
            }
            break;
            
        case PARAM_TX_POWER:
            if(len >= 2) {
                E22_SetTxPower((int8_t)data[1]);
            }
            break;
            
        case PARAM_SF:
            if(len >= 2) {
                E22_SetSpreadingFactor(data[1]);
            }
            break;
            
        case PARAM_BW:
            if(len >= 2) {
                E22_SetBandwidth(data[1]);
            }
            break;
            
        case PARAM_CR:
            if(len >= 2) {
                E22_SetCodingRate(data[1]);
            }
            break;
            
        case PARAM_SYNC_WORD:
            if(len >= 3) {
                uint16_t sw = ((uint16_t)data[1] << 8) | data[2];
                E22_SetSyncWord(sw);
            }
            break;
            
        default:
            UART_SendNack(UART_CMD_SET_PARAM, 0x02);  // Unknown param
            return;
    }
    
    E22_ApplyParams();
    UART_SendAck(UART_CMD_SET_PARAM);
}

static void HandleGetParam(void)
{
    UART_SendParams();
}

static void HandleGetHistory(void)
{
    // This will be handled by the app layer
    App_ExportHistory();
    UART_SendAck(UART_CMD_GET_HISTORY);
}

/* ======================== Process Received Packet ======================== */

static void ProcessPacket(void)
{
    switch(rxCmd) {
        case UART_CMD_CONNECT:
            HandleConnect();
            break;
            
        case UART_CMD_DISCONNECT:
            HandleDisconnect();
            break;
            
        case UART_CMD_SEND:
            if(UART_Connected) {
                HandleSend(rxData, rxLen);
            } else {
                UART_SendNack(UART_CMD_SEND, 0xFF);  // Not connected
            }
            break;
            
        case UART_CMD_SET_PARAM:
            if(UART_Connected) {
                HandleSetParam(rxData, rxLen);
            } else {
                UART_SendNack(UART_CMD_SET_PARAM, 0xFF);
            }
            break;
            
        case UART_CMD_GET_PARAM:
            if(UART_Connected) {
                HandleGetParam();
            } else {
                UART_SendNack(UART_CMD_GET_PARAM, 0xFF);
            }
            break;
            
        case UART_CMD_GET_HISTORY:
            if(UART_Connected) {
                HandleGetHistory();
            } else {
                UART_SendNack(UART_CMD_GET_HISTORY, 0xFF);
            }
            break;
            
        default:
            UART_SendNack(rxCmd, 0x00);  // Unknown command
            break;
    }
}

/* ======================== Public Functions ======================== */

void UART_Protocol_Init(void)
{
    UART_Connected = false;
    parserState = PARSER_WAIT_START;
    
    // Enable USART RX interrupt
    LL_USART_EnableIT_RXNE(USART1);
}

void UART_RxCallback(uint8_t data)
{
    switch(parserState) {
        case PARSER_WAIT_START:
            if(data == UART_START_BYTE) {
                parserState = PARSER_WAIT_CMD;
                rxChecksum = 0;
            }
            break;
            
        case PARSER_WAIT_CMD:
            rxCmd = data;
            rxChecksum ^= data;
            parserState = PARSER_WAIT_LEN;
            break;
            
        case PARSER_WAIT_LEN:
            rxLen = data;
            rxChecksum ^= data;
            rxDataIndex = 0;
            if(rxLen == 0) {
                parserState = PARSER_WAIT_CHECKSUM;
            } else {
                parserState = PARSER_WAIT_DATA;
            }
            break;
            
        case PARSER_WAIT_DATA:
            rxData[rxDataIndex++] = data;
            rxChecksum ^= data;
            if(rxDataIndex >= rxLen) {
                parserState = PARSER_WAIT_CHECKSUM;
            }
            break;
            
        case PARSER_WAIT_CHECKSUM:
            if(data == rxChecksum) {
                ProcessPacket();
            }
            // Reset parser regardless of checksum result
            parserState = PARSER_WAIT_START;
            break;
    }
}

void UART_Protocol_Process(void)
{
    // Polling mode backup (if interrupt disabled)
    if(LL_USART_IsActiveFlag_RXNE(USART1)) {
        uint8_t data = LL_USART_ReceiveData8(USART1);
        UART_RxCallback(data);
    }
}

void UART_SendAck(uint8_t cmd)
{
    txBuffer[0] = cmd;
    UART_SendPacket(UART_CMD_ACK, txBuffer, 1);
}

void UART_SendNack(uint8_t cmd, uint8_t errorCode)
{
    txBuffer[0] = cmd;
    txBuffer[1] = errorCode;
    UART_SendPacket(UART_CMD_NACK, txBuffer, 2);
}

void UART_SendReceived(uint8_t* data, uint8_t len, int8_t rssi, int8_t snr)
{
    if(!UART_Connected) return;
    
    // Format: [RSSI][SNR][DATA...]
    txBuffer[0] = (uint8_t)rssi;
    txBuffer[1] = (uint8_t)snr;
    memcpy(&txBuffer[2], data, len);
    
    UART_SendPacket(UART_CMD_RECEIVE, txBuffer, len + 2);
}

void UART_SendParams(void)
{
    E22_Params_t params;
    E22_GetParams(&params);
    
    uint8_t idx = 0;
    
    // Frequency (4 bytes)
    txBuffer[idx++] = PARAM_FREQUENCY;
    txBuffer[idx++] = (params.frequency >> 24) & 0xFF;
    txBuffer[idx++] = (params.frequency >> 16) & 0xFF;
    txBuffer[idx++] = (params.frequency >> 8) & 0xFF;
    txBuffer[idx++] = params.frequency & 0xFF;
    
    // TX Power (1 byte)
    txBuffer[idx++] = PARAM_TX_POWER;
    txBuffer[idx++] = (uint8_t)params.txPower;
    
    // SF (1 byte)
    txBuffer[idx++] = PARAM_SF;
    txBuffer[idx++] = params.sf;
    
    // BW (1 byte)
    txBuffer[idx++] = PARAM_BW;
    txBuffer[idx++] = params.bw;
    
    // CR (1 byte)
    txBuffer[idx++] = PARAM_CR;
    txBuffer[idx++] = params.cr;
    
    // Sync Word (2 bytes)
    txBuffer[idx++] = PARAM_SYNC_WORD;
    txBuffer[idx++] = (params.syncWord >> 8) & 0xFF;
    txBuffer[idx++] = params.syncWord & 0xFF;
    
    UART_SendPacket(UART_CMD_GET_PARAM, txBuffer, idx);
}

void UART_SendHistoryMessage(uint8_t index, uint8_t* data, uint8_t len, int8_t rssi)
{
    if(!UART_Connected) return;
    
    // Format: [INDEX][RSSI][DATA...]
    txBuffer[0] = index;
    txBuffer[1] = (uint8_t)rssi;
    memcpy(&txBuffer[2], data, len);
    
    UART_SendPacket(UART_CMD_GET_HISTORY, txBuffer, len + 2);
}
