/**
  ******************************************************************************
  * @file           : app.h
  * @brief          : Application Layer - Menu, Input, Message Handling
  ******************************************************************************
  */

#ifndef __APP_H
#define __APP_H

#include "main.h"

/* ======================== Message History ======================== */

#define MSG_HISTORY_SIZE        20      // Number of messages to store
#define MSG_MAX_LENGTH          200     // Max length per message

typedef struct {
    uint8_t data[MSG_MAX_LENGTH];
    uint8_t length;
    int8_t  rssi;
    int8_t  snr;
    uint32_t timestamp;
} Message_t;

/* ======================== UI States ======================== */

typedef enum {
    UI_STATE_MENU,
    UI_STATE_SEND,
    UI_STATE_HISTORY,
    UI_STATE_SETTINGS
} UI_State_t;

typedef enum {
    MENU_SEND = 0,
    MENU_HISTORY,
    MENU_SETTINGS,
    MENU_COUNT
} MenuItem_t;

typedef enum {
    SETTING_FREQ = 0,
    SETTING_POWER,
    SETTING_SF,
    SETTING_BW,
    SETTING_CR,
    SETTING_COUNT
} SettingItem_t;

/* ======================== Input Buffer ======================== */

#define INPUT_BUFFER_SIZE       200

/* ======================== Functions ======================== */

void App_Init(void);
void App_Task(void);

// UI
void App_UpdateDisplay(void);

// Message handling
void App_OnMessageReceived(uint8_t* data, uint8_t len, int8_t rssi, int8_t snr);
void App_SendMessage(void);

// History
void App_ExportHistory(void);
uint8_t App_GetHistoryCount(void);
Message_t* App_GetHistoryMessage(uint8_t index);

// LoRa callbacks
void App_LoRaTxDone(void);
void App_LoRaRxDone(uint8_t* data, uint8_t size, int8_t rssi, int8_t snr);
void App_LoRaError(uint16_t irqStatus);

#endif /* __APP_H */
