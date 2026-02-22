/**
  ******************************************************************************
  * @file           : keyboard.h
  * @brief          : 4x4 Matrix Keyboard Driver with T9 Input Method
  ******************************************************************************
  */

#ifndef __KEYBOARD_H
#define __KEYBOARD_H

#include "main.h"

/* ======================== Key Definitions ======================== */

// 4x4 矩阵键盘布局:
//        C0    C1    C2    C3
// R0     1     2     3     UP
// R1     4     5     6     DOWN
// R2     7     8     9     LEFT
// R3     CTRL  0     SHIFT RIGHT
// 
// 额外功能键说明:
// 1键:       标点符号输入
// 2-9键:     九键输入法对应字符
// 0键:       空格/特殊字符
// UP/DOWN:   菜单上下移动
// LEFT/RIGHT:光标左右移动/设置值调整
// CTRL:      配合回车换行
// SHIFT:     退格(Backspace)
// 长按SHIFT: 删除整行

// Key codes (raw scan codes)
#define KEY_1       0x00
#define KEY_2       0x01
#define KEY_3       0x02
#define KEY_UP      0x03
#define KEY_4       0x04
#define KEY_5       0x05
#define KEY_6       0x06
#define KEY_DOWN    0x07
#define KEY_7       0x08
#define KEY_8       0x09
#define KEY_9       0x0A
#define KEY_LEFT    0x0B
#define KEY_CTRL    0x0C
#define KEY_0       0x0D
#define KEY_SHIFT   0x0E  // Also acts as Backspace
#define KEY_RIGHT   0x0F
#define KEY_NONE    0xFF

// Special key combinations
#define KEY_ENTER   KEY_RIGHT   // 回车/确认
#define KEY_BACKSPACE KEY_SHIFT

/* ======================== T9 Input Method ======================== */

// T9键对应的字符
// 1: 标点符号  . , ! ? ' " ; : - + * / = @ # $ % ^ & ( ) [ ] { } < > \ | ~ `
// 2: ABC  3: DEF  4: GHI  5: JKL  6: MNO  7: PQRS  8: TUV  9: WXYZ
// 0: 空格 _ 0

/* ======================== Keyboard State ======================== */

typedef struct {
    uint8_t key;           // Current key code
    uint8_t releasedKey;   // Key that was just released
    bool pressed;          // Key is currently pressed
    bool released;         // Key was just released
    bool longPress;        // Long press detected
    uint32_t pressTime;    // Time when key was pressed
    uint8_t t9Index;       // Current T9 character index
    uint32_t lastT9Time;   // Last T9 key press time
    uint8_t lastT9Key;     // Last T9 key pressed
    bool ctrlHeld;         // CTRL key is being held
    bool capsLock;         // Caps Lock state (toggle with CTRL)
} KeyboardState_t;

extern KeyboardState_t Keyboard;

/* ======================== Functions ======================== */

void Keyboard_Init(void);
void Keyboard_Scan(void);

// 获取当前按下的键
uint8_t Keyboard_GetKey(void);

// 检查某个键是否被按下
bool Keyboard_IsPressed(uint8_t key);

// 检查某个键是否刚被释放
bool Keyboard_IsReleased(uint8_t key);

// 检查是否有任意键被按下
bool Keyboard_AnyKeyPressed(void);

// T9 输入法相关
char Keyboard_GetT9Char(uint8_t key, uint8_t index);
uint8_t Keyboard_GetT9CharCount(uint8_t key);
const char* Keyboard_GetT9String(uint8_t key);

// 大小写切换
bool Keyboard_IsCapsLock(void);
void Keyboard_ToggleCapsLock(void);

// 获取按键对应的显示名称
const char* Keyboard_GetKeyName(uint8_t key);

#endif /* __KEYBOARD_H */