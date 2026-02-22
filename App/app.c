/**
  ******************************************************************************
  * @file           : app.c
  * @brief          : Application Layer Implementation
  ******************************************************************************
  */

#include "app.h"
#include "oled.h"
#include "keyboard.h"
#include "e22.h"
#include "uart_protocol.h"

/* ======================== Message History ======================== */

static Message_t msgHistory[MSG_HISTORY_SIZE];
static uint8_t msgHistoryHead = 0;
static uint8_t msgHistoryCount = 0;

/* ======================== UI State ======================== */

static UI_State_t uiState = UI_STATE_MENU;
static MenuItem_t menuSelection = MENU_SEND;
static SettingItem_t settingSelection = SETTING_FREQ;
static uint8_t historyViewIndex = 0;

/* ======================== Input State ======================== */

static char inputBuffer[INPUT_BUFFER_SIZE];
static uint8_t inputLength = 0;
static uint8_t inputCursor = 0;
static bool inputT9Active = false;
static char inputT9Char = '\0';
static uint32_t inputT9LastTime = 0;

/* ======================== Settings Edit State ======================== */

static bool settingEditMode = false;
static char settingEditBuffer[16];
static uint8_t settingEditLen = 0;

/* ======================== Display Flags ======================== */

static bool displayNeedsUpdate = true;
static uint32_t lastKeyTime = 0;

/* ======================== Menu Strings ======================== */

static const char* MenuStrings[] = {
    "Send Message",
    "View History",
    "Settings"
};

static const char* SettingNames[] = {
    "Freq(kHz)",
    "Power(dBm)",
    "SF",
    "BW",
    "CR"
};

/* ======================== Helper Functions ======================== */

static void ClearInputBuffer(void)
{
    memset(inputBuffer, 0, INPUT_BUFFER_SIZE);
    inputLength = 0;
    inputCursor = 0;
    inputT9Active = false;
}

static void ClearSettingEdit(void)
{
    memset(settingEditBuffer, 0, sizeof(settingEditBuffer));
    settingEditLen = 0;
    settingEditMode = false;
}

static void CommitT9Char(void)
{
    if(inputT9Active && inputT9Char != '\0') {
        if(inputLength < INPUT_BUFFER_SIZE - 1) {
            // Insert at cursor position
            memmove(&inputBuffer[inputCursor + 1], &inputBuffer[inputCursor], 
                    inputLength - inputCursor);
            inputBuffer[inputCursor] = inputT9Char;
            inputLength++;
            inputCursor++;
        }
        inputT9Active = false;
        inputT9Char = '\0';
    }
}

static void GetSettingValueString(SettingItem_t setting, char* buf)
{
    E22_Params_t params;
    E22_GetParams(&params);
    
    switch(setting) {
        case SETTING_FREQ:
            sprintf(buf, "%lu", params.frequency / 1000);
            break;
        case SETTING_POWER:
            sprintf(buf, "%d", params.txPower);
            break;
        case SETTING_SF:
            sprintf(buf, "%d", params.sf);
            break;
        case SETTING_BW:
            sprintf(buf, "%s", E22_GetBandwidthString(params.bw));
            break;
        case SETTING_CR:
            sprintf(buf, "4/%d", params.cr + 4);
            break;
        default:
            buf[0] = '\0';
    }
}

static void ApplySettingValue(SettingItem_t setting)
{
    if(settingEditLen == 0) return;
    
    int32_t value = 0;
    for(uint8_t i = 0; i < settingEditLen; i++) {
        if(settingEditBuffer[i] >= '0' && settingEditBuffer[i] <= '9') {
            value = value * 10 + (settingEditBuffer[i] - '0');
        }
    }
    
    switch(setting) {
        case SETTING_FREQ:
            E22_SetFrequency((uint32_t)value * 1000);
            break;
        case SETTING_POWER:
            E22_SetTxPower((int8_t)value);
            break;
        case SETTING_SF:
            E22_SetSpreadingFactor((uint8_t)value);
            break;
        case SETTING_BW:
            E22_SetBandwidth((uint8_t)value);
            break;
        case SETTING_CR:
            E22_SetCodingRate((uint8_t)value);
            break;
        default:
            break;
    }
    
    E22_ApplyParams();
}

/* ======================== Display Functions ======================== */

static void DrawMenu(void)
{
    OLED_Clear();
    
    // Title at top
    OLED_DrawString(0, 0, "= LoRa Node =", 1);
    
    // Current selection (large, centered)
    const char* item = MenuStrings[menuSelection];
    uint8_t len = strlen(item);
    uint8_t x = (OLED_WIDTH - len * 6) / 2;
    OLED_DrawStringInverse(x, 12, item, 1);
    
    // Navigation hint at bottom
    OLED_DrawString(0, 24, "UP/DN:Nav ENT:Select", 1);
    
    OLED_Update();
}

static void DrawSendScreen(void)
{
    OLED_Clear();
    
    // Show Caps indicator at top right
    if(Keyboard.capsLock) {
        OLED_DrawStringInverse(110, 0, "ABC", 1);
    } else {
        OLED_DrawString(110, 0, "abc", 1);
    }
    
    // Show input text (up to 2 lines)
    uint8_t maxChars = 18;  // Leave space for caps indicator
    
    // Line 1 (characters 0-17)
    char line[22];
    memset(line, 0, sizeof(line));
    uint8_t startIdx = 0;
    if(inputCursor >= maxChars) {
        startIdx = inputCursor - maxChars + 1;
    }
    
    uint8_t copyLen = inputLength - startIdx;
    if(copyLen > maxChars) copyLen = maxChars;
    memcpy(line, &inputBuffer[startIdx], copyLen);
    
    // Add T9 preview char if active
    if(inputT9Active && inputT9Char != '\0') {
        if(copyLen < maxChars) {
            line[copyLen] = inputT9Char;
        }
    }
    
    OLED_DrawString(0, 0, line, 1);
    
    // Draw cursor
    uint8_t cursorX = (inputCursor - startIdx) * 6;
    if(inputT9Active) cursorX += 6;
    if(cursorX < 108) {  // Don't overlap with caps indicator
        OLED_DrawString(cursorX, 8, "_", 1);
    }
    
    // T9 hint line (bottom)
    if(Keyboard.key != KEY_NONE && Keyboard.key <= KEY_0) {
        const char* t9str = Keyboard_GetT9String(Keyboard.key);
        if(t9str) {
            char hint[24];
            snprintf(hint, sizeof(hint), "%s:[%s]", 
                    Keyboard_GetKeyName(Keyboard.key), t9str);
            OLED_DrawString(0, 24, hint, 1);
        }
    } else {
        // Show hint with CTRL toggle info
        OLED_DrawString(0, 24, "ENT:Send CTRL:Aa", 1);
    }
    
    OLED_Update();
}

static void DrawHistoryScreen(void)
{
    OLED_Clear();
    
    if(msgHistoryCount == 0) {
        OLED_DrawString(0, 8, "No messages", 1);
        OLED_DrawString(0, 24, "Press any key", 1);
    } else {
        // Show message info
        uint8_t idx = (msgHistoryHead + MSG_HISTORY_SIZE - msgHistoryCount + historyViewIndex) 
                     % MSG_HISTORY_SIZE;
        Message_t* msg = &msgHistory[idx];
        
        // Header: index and RSSI
        char header[24];
        snprintf(header, sizeof(header), "[%d/%d] RSSI:%d", 
                historyViewIndex + 1, msgHistoryCount, msg->rssi);
        OLED_DrawString(0, 0, header, 1);
        
        // Message preview (2 lines)
        char preview[43];  // 21*2 + 1
        memset(preview, 0, sizeof(preview));
        uint8_t previewLen = msg->length > 42 ? 42 : msg->length;
        memcpy(preview, msg->data, previewLen);
        
        // First line
        if(previewLen > 0) {
            char line1[22];
            memset(line1, 0, sizeof(line1));
            memcpy(line1, preview, previewLen > 21 ? 21 : previewLen);
            OLED_DrawString(0, 10, line1, 1);
        }
        
        // Second line
        if(previewLen > 21) {
            char line2[22];
            memset(line2, 0, sizeof(line2));
            memcpy(line2, &preview[21], previewLen - 21);
            OLED_DrawString(0, 18, line2, 1);
        }
        
        // Navigation hint
        OLED_DrawString(0, 24, "U/D:Nav BS:Back", 1);
    }
    
    OLED_Update();
}

static void DrawSettingsScreen(void)
{
    OLED_Clear();
    
    // Title
    OLED_DrawString(0, 0, "= Settings =", 1);
    
    // Current setting
    char valueBuf[16];
    
    if(settingEditMode) {
        // Edit mode: show name and edit buffer
        char line[24];
        snprintf(line, sizeof(line), "%s:", SettingNames[settingSelection]);
        OLED_DrawString(0, 10, line, 1);
        
        // Edit buffer with cursor
        OLED_DrawStringInverse(64, 10, settingEditBuffer, 1);
        OLED_DrawString(64 + settingEditLen * 6, 18, "_", 1);
        
        OLED_DrawString(0, 24, "ENT:OK BS:Del", 1);
    } else {
        // Browse mode: show current setting
        GetSettingValueString(settingSelection, valueBuf);
        
        char line[24];
        snprintf(line, sizeof(line), "%s", SettingNames[settingSelection]);
        OLED_DrawStringInverse(0, 10, line, 1);
        
        OLED_DrawString(70, 10, valueBuf, 1);
        
        OLED_DrawString(0, 24, "U/D:Nav 0-9:Edit", 1);
    }
    
    OLED_Update();
}

/* ======================== Input Handlers ======================== */

static void HandleMenuInput(void)
{
    if(Keyboard.released) {
        switch(Keyboard.releasedKey) {
            case KEY_UP:
                if(menuSelection > 0) menuSelection--;
                else menuSelection = MENU_COUNT - 1;
                displayNeedsUpdate = true;
                break;
                
            case KEY_DOWN:
                if(menuSelection < MENU_COUNT - 1) menuSelection++;
                else menuSelection = 0;
                displayNeedsUpdate = true;
                break;
                
            case KEY_RIGHT:  // Enter
                switch(menuSelection) {
                    case MENU_SEND:
                        uiState = UI_STATE_SEND;
                        ClearInputBuffer();
                        break;
                    case MENU_HISTORY:
                        uiState = UI_STATE_HISTORY;
                        historyViewIndex = 0;
                        break;
                    case MENU_SETTINGS:
                        uiState = UI_STATE_SETTINGS;
                        settingSelection = SETTING_FREQ;
                        ClearSettingEdit();
                        break;
                }
                displayNeedsUpdate = true;
                break;
        }
    }
}

static void HandleSendInput(void)
{
    // Check for T9 timeout
    if(inputT9Active && (GetTick() - inputT9LastTime > 1500)) {
        CommitT9Char();
        displayNeedsUpdate = true;
    }
    
    if(Keyboard.released) {
        uint8_t key = Keyboard.releasedKey;
        
        // Check for Ctrl+Enter (newline)
        if(key == KEY_RIGHT && Keyboard.ctrlHeld) {
            CommitT9Char();
            if(inputLength < INPUT_BUFFER_SIZE - 1) {
                memmove(&inputBuffer[inputCursor + 1], &inputBuffer[inputCursor], 
                        inputLength - inputCursor);
                inputBuffer[inputCursor] = '\n';
                inputLength++;
                inputCursor++;
            }
            displayNeedsUpdate = true;
            return;
        }
        
        switch(key) {
            case KEY_RIGHT:  // Enter - send message
                CommitT9Char();
                if(inputLength > 0) {
                    OLED_ClearArea(0, 24, 128, 8);
                    OLED_DrawString(0, 24, "Sending...", 1);
                    OLED_Update();
                    App_SendMessage();
                }
                uiState = UI_STATE_MENU;
                displayNeedsUpdate = true;
                break;
                
            case KEY_SHIFT:  // Backspace
                CommitT9Char();
                if(Keyboard.longPress) {
                    // Long press: clear all
                    ClearInputBuffer();
                } else if(inputCursor > 0) {
                    // Delete char before cursor
                    memmove(&inputBuffer[inputCursor - 1], &inputBuffer[inputCursor], 
                            inputLength - inputCursor);
                    inputLength--;
                    inputCursor--;
                    inputBuffer[inputLength] = '\0';
                }
                displayNeedsUpdate = true;
                break;
                
            case KEY_LEFT:
                CommitT9Char();
                if(inputCursor > 0) inputCursor--;
                displayNeedsUpdate = true;
                break;
                
            case KEY_UP:
            case KEY_DOWN:
                // Exit to menu
                CommitT9Char();
                uiState = UI_STATE_MENU;
                displayNeedsUpdate = true;
                break;
                
            case KEY_CTRL:
                // Ctrl alone does nothing, wait for combo
                break;
                
            default:
                // T9 input keys (0-9, 1)
                if(key <= KEY_0) {
                    const char* t9str = Keyboard_GetT9String(key);
                    if(t9str && strlen(t9str) > 0) {
                        if(inputT9Active && Keyboard.lastT9Key == key) {
                            // Same key pressed again, cycle character
                            uint8_t idx = Keyboard.t9Index;
                            inputT9Char = t9str[idx];
                        } else {
                            // Different key or new input
                            CommitT9Char();
                            inputT9Char = t9str[0];
                            inputT9Active = true;
                        }
                        inputT9LastTime = GetTick();
                        displayNeedsUpdate = true;
                    }
                }
                break;
        }
    }
}

static void HandleHistoryInput(void)
{
    if(Keyboard.released) {
        switch(Keyboard.releasedKey) {
            case KEY_UP:
                if(historyViewIndex > 0) {
                    historyViewIndex--;
                    displayNeedsUpdate = true;
                }
                break;
                
            case KEY_DOWN:
                if(historyViewIndex < msgHistoryCount - 1) {
                    historyViewIndex++;
                    displayNeedsUpdate = true;
                }
                break;
                
            case KEY_SHIFT:  // Back
            case KEY_LEFT:
            case KEY_RIGHT:
                uiState = UI_STATE_MENU;
                displayNeedsUpdate = true;
                break;
        }
    }
}

static void HandleSettingsInput(void)
{
    if(Keyboard.released) {
        uint8_t key = Keyboard.releasedKey;
        
        if(settingEditMode) {
            // Edit mode
            switch(key) {
                case KEY_RIGHT:  // Enter - apply value
                    ApplySettingValue(settingSelection);
                    ClearSettingEdit();
                    displayNeedsUpdate = true;
                    break;
                    
                case KEY_SHIFT:  // Backspace
                    if(settingEditLen > 0) {
                        settingEditLen--;
                        settingEditBuffer[settingEditLen] = '\0';
                    } else {
                        ClearSettingEdit();  // Exit edit mode
                    }
                    displayNeedsUpdate = true;
                    break;
                    
                default:
                    // Number input
                    if(key >= KEY_1 && key <= KEY_9) {
                        if(settingEditLen < sizeof(settingEditBuffer) - 1) {
                            settingEditBuffer[settingEditLen++] = '0' + (key - KEY_1 + 1);
                            if(key == KEY_1) settingEditBuffer[settingEditLen - 1] = '1';
                            else if(key == KEY_2) settingEditBuffer[settingEditLen - 1] = '2';
                            else if(key == KEY_3) settingEditBuffer[settingEditLen - 1] = '3';
                            else if(key == KEY_4) settingEditBuffer[settingEditLen - 1] = '4';
                            else if(key == KEY_5) settingEditBuffer[settingEditLen - 1] = '5';
                            else if(key == KEY_6) settingEditBuffer[settingEditLen - 1] = '6';
                            else if(key == KEY_7) settingEditBuffer[settingEditLen - 1] = '7';
                            else if(key == KEY_8) settingEditBuffer[settingEditLen - 1] = '8';
                            else if(key == KEY_9) settingEditBuffer[settingEditLen - 1] = '9';
                        }
                        displayNeedsUpdate = true;
                    } else if(key == KEY_0) {
                        if(settingEditLen < sizeof(settingEditBuffer) - 1) {
                            settingEditBuffer[settingEditLen++] = '0';
                        }
                        displayNeedsUpdate = true;
                    }
                    break;
            }
        } else {
            // Browse mode
            switch(key) {
                case KEY_UP:
                    if(settingSelection > 0) settingSelection--;
                    else settingSelection = SETTING_COUNT - 1;
                    displayNeedsUpdate = true;
                    break;
                    
                case KEY_DOWN:
                    if(settingSelection < SETTING_COUNT - 1) settingSelection++;
                    else settingSelection = 0;
                    displayNeedsUpdate = true;
                    break;
                    
                case KEY_SHIFT:  // Back
                    uiState = UI_STATE_MENU;
                    displayNeedsUpdate = true;
                    break;
                    
                default:
                    // Start editing with number key
                    if((key >= KEY_1 && key <= KEY_9) || key == KEY_0) {
                        settingEditMode = true;
                        settingEditLen = 0;
                        memset(settingEditBuffer, 0, sizeof(settingEditBuffer));
                        // Add first digit
                        if(key == KEY_0) {
                            settingEditBuffer[settingEditLen++] = '0';
                        } else {
                            settingEditBuffer[settingEditLen++] = '1' + (key - KEY_1);
                        }
                        displayNeedsUpdate = true;
                    }
                    break;
            }
        }
    }
}

/* ======================== Public Functions ======================== */

void App_Init(void)
{
    uiState = UI_STATE_MENU;
    menuSelection = MENU_SEND;
    
    ClearInputBuffer();
    
    msgHistoryHead = 0;
    msgHistoryCount = 0;
    
    // Register LoRa callbacks
    E22_SetTxDoneCallback(App_LoRaTxDone);
    E22_SetRxDoneCallback(App_LoRaRxDone);
    E22_SetErrorCallback(App_LoRaError);
    
    displayNeedsUpdate = true;
}

void App_Task(void)
{
    // Scan keyboard
    Keyboard_Scan();
    
    // Handle input based on current state
    switch(uiState) {
        case UI_STATE_MENU:
            HandleMenuInput();
            break;
        case UI_STATE_SEND:
            HandleSendInput();
            break;
        case UI_STATE_HISTORY:
            HandleHistoryInput();
            break;
        case UI_STATE_SETTINGS:
            HandleSettingsInput();
            break;
    }
    
    // Update display if needed
    if(displayNeedsUpdate) {
        App_UpdateDisplay();
        displayNeedsUpdate = false;
    }
    
    // Poll LoRa
    E22_PollTask();
}

void App_UpdateDisplay(void)
{
    switch(uiState) {
        case UI_STATE_MENU:
            DrawMenu();
            break;
        case UI_STATE_SEND:
            DrawSendScreen();
            break;
        case UI_STATE_HISTORY:
            DrawHistoryScreen();
            break;
        case UI_STATE_SETTINGS:
            DrawSettingsScreen();
            break;
    }
}

void App_OnMessageReceived(uint8_t* data, uint8_t len, int8_t rssi, int8_t snr)
{
    // Store in history
    Message_t* msg = &msgHistory[msgHistoryHead];
    
    uint8_t copyLen = len > MSG_MAX_LENGTH ? MSG_MAX_LENGTH : len;
    memcpy(msg->data, data, copyLen);
    msg->length = copyLen;
    msg->rssi = rssi;
    msg->snr = snr;
    msg->timestamp = GetTick();
    
    msgHistoryHead = (msgHistoryHead + 1) % MSG_HISTORY_SIZE;
    if(msgHistoryCount < MSG_HISTORY_SIZE) {
        msgHistoryCount++;
    }
    
    // Forward to UART if connected
    UART_SendReceived(data, len, rssi, snr);
    
    // Update display if in history view
    if(uiState == UI_STATE_HISTORY) {
        displayNeedsUpdate = true;
    }
}

void App_SendMessage(void)
{
    if(inputLength > 0) {
        E22_Send((uint8_t*)inputBuffer, inputLength);
        ClearInputBuffer();
    }
}

void App_ExportHistory(void)
{
    for(uint8_t i = 0; i < msgHistoryCount; i++) {
        uint8_t idx = (msgHistoryHead + MSG_HISTORY_SIZE - msgHistoryCount + i) % MSG_HISTORY_SIZE;
        Message_t* msg = &msgHistory[idx];
        UART_SendHistoryMessage(i, msg->data, msg->length, msg->rssi);
        Delay_Ms(10);  // Small delay between messages
    }
}

uint8_t App_GetHistoryCount(void)
{
    return msgHistoryCount;
}

Message_t* App_GetHistoryMessage(uint8_t index)
{
    if(index >= msgHistoryCount) return NULL;
    uint8_t idx = (msgHistoryHead + MSG_HISTORY_SIZE - msgHistoryCount + index) % MSG_HISTORY_SIZE;
    return &msgHistory[idx];
}

/* ======================== LoRa Callbacks ======================== */

void App_LoRaTxDone(void)
{
    // TX complete notification
}

void App_LoRaRxDone(uint8_t* data, uint8_t size, int8_t rssi, int8_t snr)
{
    App_OnMessageReceived(data, size, rssi, snr);
}

void App_LoRaError(uint16_t irqStatus)
{
    // Handle errors if needed
}
