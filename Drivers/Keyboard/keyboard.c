/**
  ******************************************************************************
  * @file           : keyboard.c
  * @brief          : 4x4 Matrix Keyboard Driver Implementation
  ******************************************************************************
  */

#include "keyboard.h"

/* ======================== T9 Character Tables ======================== */

// 1键: 标点符号
static const char T9_Key1[] = ".,!?'\";:-+*/=@#$%&()[]<>";
// 2-9键: 字母(小写) + 数字
static const char T9_Key2[] = "abc2";
static const char T9_Key3[] = "def3";
static const char T9_Key4[] = "ghi4";
static const char T9_Key5[] = "jkl5";
static const char T9_Key6[] = "mno6";
static const char T9_Key7[] = "pqrs7";
static const char T9_Key8[] = "tuv8";
static const char T9_Key9[] = "wxyz9";
// 0键: 空格和数字0
static const char T9_Key0[] = " _0";

// T9 lookup table
static const char* T9_Tables[] = {
    T9_Key1,  // KEY_1
    T9_Key2,  // KEY_2
    T9_Key3,  // KEY_3
    NULL,     // KEY_UP
    T9_Key4,  // KEY_4
    T9_Key5,  // KEY_5
    T9_Key6,  // KEY_6
    NULL,     // KEY_DOWN
    T9_Key7,  // KEY_7
    T9_Key8,  // KEY_8
    T9_Key9,  // KEY_9
    NULL,     // KEY_LEFT
    NULL,     // KEY_CTRL
    T9_Key0,  // KEY_0
    NULL,     // KEY_SHIFT
    NULL      // KEY_RIGHT
};

/* ======================== Row/Column GPIO Tables ======================== */

static GPIO_TypeDef* const RowPorts[] = {K_R0_PORT, K_R1_PORT, K_R2_PORT, K_R3_PORT};
static const uint32_t RowPins[] = {K_R0_PIN, K_R1_PIN, K_R2_PIN, K_R3_PIN};

static GPIO_TypeDef* const ColPorts[] = {K_C0_PORT, K_C1_PORT, K_C2_PORT, K_C3_PORT};
static const uint32_t ColPins[] = {K_C0_PIN, K_C1_PIN, K_C2_PIN, K_C3_PIN};

/* ======================== State Variables ======================== */

KeyboardState_t Keyboard;

static uint8_t lastKey = KEY_NONE;
static uint8_t debounceCounter = 0;
static const uint8_t DEBOUNCE_COUNT = 3;
static const uint32_t LONG_PRESS_TIME = 800;  // ms
static const uint32_t T9_TIMEOUT = 1500;      // ms for T9 input

/* ======================== Functions ======================== */

void Keyboard_Init(void)
{
    memset(&Keyboard, 0, sizeof(Keyboard));
    Keyboard.key = KEY_NONE;
    Keyboard.releasedKey = KEY_NONE;
    Keyboard.lastT9Key = KEY_NONE;
    Keyboard.capsLock = false;  // 默认小写
    
    // Set all rows high (inactive)
    for(int i = 0; i < 4; i++) {
        LL_GPIO_SetOutputPin(RowPorts[i], RowPins[i]);
    }
}

static uint8_t ScanMatrix(void)
{
    uint8_t key = KEY_NONE;
    
    for(uint8_t row = 0; row < 4; row++) {
        // Set current row low
        LL_GPIO_ResetOutputPin(RowPorts[row], RowPins[row]);
        
        // Delay for signal stabilization
        for(volatile int i = 0; i < 50; i++);
        
        // Read columns
        for(uint8_t col = 0; col < 4; col++) {
            if(!LL_GPIO_IsInputPinSet(ColPorts[col], ColPins[col])) {
                key = row * 4 + col;
            }
        }
        
        // Set row back to high
        LL_GPIO_SetOutputPin(RowPorts[row], RowPins[row]);
        
        // Small delay before next row
        for(volatile int i = 0; i < 10; i++);
    }
    
    return key;
}

void Keyboard_Scan(void)
{
    uint8_t currentKey = ScanMatrix();
    
    Keyboard.released = false;
    Keyboard.releasedKey = KEY_NONE;
    
    // Debouncing
    if(currentKey != lastKey) {
        debounceCounter++;
        if(debounceCounter >= DEBOUNCE_COUNT) {
            // Key state changed
            if(currentKey == KEY_NONE && Keyboard.key != KEY_NONE) {
                // Key released - save which key was released
                Keyboard.released = true;
                Keyboard.releasedKey = Keyboard.key;
                
                // Check if it was a long press
                uint32_t pressDuration = GetTick() - Keyboard.pressTime;
                Keyboard.longPress = (pressDuration >= LONG_PRESS_TIME);
            }
            else if(currentKey != KEY_NONE) {
                // New key pressed
                Keyboard.pressTime = GetTick();
                Keyboard.longPress = false;
                
                // Handle T9 input timing
                if(T9_Tables[currentKey] != NULL) {
                    uint32_t timeSinceLastT9 = GetTick() - Keyboard.lastT9Time;
                    if(currentKey == Keyboard.lastT9Key && timeSinceLastT9 < T9_TIMEOUT) {
                        // Same key pressed within timeout, cycle to next char
                        Keyboard.t9Index++;
                        if(Keyboard.t9Index >= strlen(T9_Tables[currentKey])) {
                            Keyboard.t9Index = 0;
                        }
                    } else {
                        // Different key or timeout, reset
                        Keyboard.t9Index = 0;
                    }
                    Keyboard.lastT9Key = currentKey;
                    Keyboard.lastT9Time = GetTick();
                }
            }
            
            // Update CTRL state and toggle CapsLock
            if(currentKey == KEY_CTRL) {
                Keyboard.ctrlHeld = true;
            } else if(Keyboard.key == KEY_CTRL && currentKey == KEY_NONE) {
                Keyboard.ctrlHeld = false;
                // Toggle CapsLock on CTRL release (short press only)
                if(!Keyboard.longPress) {
                    Keyboard.capsLock = !Keyboard.capsLock;
                }
            }
            
            Keyboard.key = currentKey;
            Keyboard.pressed = (currentKey != KEY_NONE);
            lastKey = currentKey;
            debounceCounter = 0;
        }
    } else {
        debounceCounter = 0;
        
        // Check for long press while key is held
        if(Keyboard.key != KEY_NONE && !Keyboard.longPress) {
            uint32_t pressDuration = GetTick() - Keyboard.pressTime;
            if(pressDuration >= LONG_PRESS_TIME) {
                Keyboard.longPress = true;
            }
        }
    }
}

uint8_t Keyboard_GetKey(void)
{
    return Keyboard.key;
}

bool Keyboard_IsPressed(uint8_t key)
{
    return (Keyboard.key == key && Keyboard.pressed);
}

bool Keyboard_IsReleased(uint8_t key)
{
    return (Keyboard.key == key && Keyboard.released);
}

bool Keyboard_AnyKeyPressed(void)
{
    return Keyboard.pressed;
}

char Keyboard_GetT9Char(uint8_t key, uint8_t index)
{
    if(key > KEY_RIGHT || T9_Tables[key] == NULL) return '\0';
    
    uint8_t len = strlen(T9_Tables[key]);
    if(index >= len) index = 0;
    
    char ch = T9_Tables[key][index];
    
    // Apply CapsLock: convert lowercase to uppercase
    if(Keyboard.capsLock && ch >= 'a' && ch <= 'z') {
        ch = ch - 'a' + 'A';
    }
    
    return ch;
}

uint8_t Keyboard_GetT9CharCount(uint8_t key)
{
    if(key > KEY_RIGHT || T9_Tables[key] == NULL) return 0;
    return strlen(T9_Tables[key]);
}

const char* Keyboard_GetT9String(uint8_t key)
{
    if(key > KEY_RIGHT) return NULL;
    return T9_Tables[key];
}

bool Keyboard_IsCapsLock(void)
{
    return Keyboard.capsLock;
}

void Keyboard_ToggleCapsLock(void)
{
    Keyboard.capsLock = !Keyboard.capsLock;
}

const char* Keyboard_GetKeyName(uint8_t key)
{
    switch(key) {
        case KEY_1:     return "1";
        case KEY_2:     return "2";
        case KEY_3:     return "3";
        case KEY_4:     return "4";
        case KEY_5:     return "5";
        case KEY_6:     return "6";
        case KEY_7:     return "7";
        case KEY_8:     return "8";
        case KEY_9:     return "9";
        case KEY_0:     return "0";
        case KEY_UP:    return "UP";
        case KEY_DOWN:  return "DOWN";
        case KEY_LEFT:  return "LEFT";
        case KEY_RIGHT: return "ENT";
        case KEY_CTRL:  return "CTRL";
        case KEY_SHIFT: return "BS";
        default:        return "?";
    }
}