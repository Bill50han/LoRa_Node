/**
  ******************************************************************************
  * @file           : oled.h
  * @brief          : SSD1306 OLED Driver for 128x32 display via I2C
  ******************************************************************************
  */

#ifndef __OLED_H
#define __OLED_H

#include "main.h"

/* ======================== OLED Parameters ======================== */

#define OLED_WIDTH          128
#define OLED_HEIGHT         32
#define OLED_PAGES          (OLED_HEIGHT / 8)
#define OLED_I2C_ADDR       0x78    // 0x3C << 1

/* ======================== Font Sizes ======================== */

#define FONT_6X8_WIDTH      6
#define FONT_6X8_HEIGHT     8

/* ======================== Functions ======================== */

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Fill(uint8_t pattern);
void OLED_Update(void);

void OLED_SetCursor(uint8_t page, uint8_t col);
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void OLED_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t size);
void OLED_DrawString(uint8_t x, uint8_t y, const char* str, uint8_t size);
void OLED_DrawStringInverse(uint8_t x, uint8_t y, const char* str, uint8_t size);

void OLED_ClearLine(uint8_t page);
void OLED_ClearArea(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

void OLED_ScrollUp(void);

// 直接绘制到屏幕（无缓冲）
void OLED_DirectDrawString(uint8_t page, uint8_t col, const char* str);

#endif /* __OLED_H */
