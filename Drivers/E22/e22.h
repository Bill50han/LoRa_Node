/**
  ******************************************************************************
  * @file           : e22.h
  * @brief          : E22-400M22S LoRa Module Driver (SX1268)
  ******************************************************************************
  */

#ifndef __E22_H
#define __E22_H

#include "main.h"

/* ======================== Module Parameters ======================== */

#define E22_MAX_PACKET_SIZE     255
#define E22_FIFO_SIZE           256

/* ======================== Default RF Parameters ======================== */

#define E22_DEFAULT_FREQ        433125000   // 433.125 MHz
#define E22_DEFAULT_TX_POWER    22          // dBm
#define E22_DEFAULT_SF          7          // Spreading Factor
#define E22_DEFAULT_BW          7           // 500kHz (index)
#define E22_DEFAULT_CR          1           // 4/5

/* ======================== Bandwidth Values (Index) ======================== */
// 0: 7.81kHz, 1: 10.42kHz, 2: 15.63kHz, 3: 20.83kHz
// 4: 31.25kHz, 5: 41.67kHz, 6: 62.5kHz, 7: 125kHz
// 8: 250kHz, 9: 500kHz

/* ======================== LoRa Parameters Structure ======================== */

typedef struct {
    uint32_t frequency;     // Center frequency in Hz
    int8_t   txPower;       // TX power in dBm (-9 to +22)
    uint8_t  sf;            // Spreading factor (5-12)
    uint8_t  bw;            // Bandwidth index (0-9)
    uint8_t  cr;            // Coding rate (1=4/5, 2=4/6, 3=4/7, 4=4/8)
    uint16_t syncWord;      // Sync word
    uint16_t preambleLen;   // Preamble length
} E22_Params_t;

/* ======================== Packet Status ======================== */

typedef struct {
    int8_t  rssi;
    int8_t  snr;
    uint8_t status;
} E22_PacketStatus_t;

/* ======================== IRQ Flags ======================== */

#define E22_IRQ_TX_DONE         (1 << 0)
#define E22_IRQ_RX_DONE         (1 << 1)
#define E22_IRQ_PREAMBLE_DETECT (1 << 2)
#define E22_IRQ_SYNC_VALID      (1 << 3)
#define E22_IRQ_HEADER_VALID    (1 << 4)
#define E22_IRQ_HEADER_ERROR    (1 << 5)
#define E22_IRQ_CRC_ERROR       (1 << 6)
#define E22_IRQ_CAD_DONE        (1 << 7)
#define E22_IRQ_CAD_DETECT      (1 << 8)
#define E22_IRQ_TIMEOUT         (1 << 9)

/* ======================== GPIO Control Macros ======================== */

// NSS (Chip Select)
#define LR_NSS_LOW()     LL_GPIO_ResetOutputPin(LR_SPI_NSS_PORT, LR_SPI_NSS_PIN)
#define LR_NSS_HIGH()    LL_GPIO_SetOutputPin(LR_SPI_NSS_PORT, LR_SPI_NSS_PIN)

// NRST (Reset)
#define LR_NRST_LOW()    LL_GPIO_ResetOutputPin(LR_SPI_NRST_PORT, LR_SPI_NRST_PIN)
#define LR_NRST_HIGH()   LL_GPIO_SetOutputPin(LR_SPI_NRST_PORT, LR_SPI_NRST_PIN)

// BUSY (Input)
#define LR_BUSY_READ()   LL_GPIO_IsInputPinSet(LR_SPI_BUSY_PORT, LR_SPI_BUSY_PIN)

// TXEN/RXEN (RF Switch Control)
#define LR_TXEN_HIGH()   LL_GPIO_SetOutputPin(LR_TXEN_PORT, LR_TXEN_PIN)
#define LR_TXEN_LOW()    LL_GPIO_ResetOutputPin(LR_TXEN_PORT, LR_TXEN_PIN)
#define LR_RXEN_HIGH()   LL_GPIO_SetOutputPin(LR_RXEN_PORT, LR_RXEN_PIN)
#define LR_RXEN_LOW()    LL_GPIO_ResetOutputPin(LR_RXEN_PORT, LR_RXEN_PIN)

/* ======================== Callback Types ======================== */

typedef void (*E22_TxDoneCallback_t)(void);
typedef void (*E22_RxDoneCallback_t)(uint8_t* data, uint8_t size, int8_t rssi, int8_t snr);
typedef void (*E22_ErrorCallback_t)(uint16_t irqStatus);

/* ======================== Public Variables ======================== */

extern E22_Params_t E22_Params;
extern volatile bool E22_TxBusy;

/* ======================== Functions ======================== */

// Initialization
void E22_Init(void);
void E22_Reset(void);

// Configuration
void E22_SetFrequency(uint32_t freq);
void E22_SetTxPower(int8_t power);
void E22_SetSpreadingFactor(uint8_t sf);
void E22_SetBandwidth(uint8_t bw);
void E22_SetCodingRate(uint8_t cr);
void E22_SetSyncWord(uint16_t syncWord);
void E22_ApplyParams(void);

// Get current parameters
void E22_GetParams(E22_Params_t* params);

// Transmit/Receive
void E22_Send(uint8_t* data, uint8_t size);
void E22_StartReceive(void);
void E22_SetSleep(void);
void E22_SetStandby(void);

// Polling task (call periodically)
void E22_PollTask(void);

// Callbacks
void E22_SetTxDoneCallback(E22_TxDoneCallback_t cb);
void E22_SetRxDoneCallback(E22_RxDoneCallback_t cb);
void E22_SetErrorCallback(E22_ErrorCallback_t cb);

// Status
bool E22_IsBusy(void);
bool E22_IsTxBusy(void);
void E22_GetPacketStatus(E22_PacketStatus_t* status);

// Utility
const char* E22_GetBandwidthString(uint8_t bw);
uint32_t E22_GetBandwidthHz(uint8_t bw);

#endif /* __E22_H */