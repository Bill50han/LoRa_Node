/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_iwdg.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* ======================== GPIO Pin Definitions ======================== */

// Keyboard Row Outputs (Active Low)
#define K_R0_PORT       GPIOA
#define K_R0_PIN        LL_GPIO_PIN_3
#define K_R1_PORT       GPIOA
#define K_R1_PIN        LL_GPIO_PIN_2
#define K_R2_PORT       GPIOA
#define K_R2_PIN        LL_GPIO_PIN_1
#define K_R3_PORT       GPIOA
#define K_R3_PIN        LL_GPIO_PIN_0

// Keyboard Column Inputs (Pull-up)
#define K_C0_PORT       GPIOA
#define K_C0_PIN        LL_GPIO_PIN_11
#define K_C1_PORT       GPIOA
#define K_C1_PIN        LL_GPIO_PIN_12
#define K_C2_PORT       GPIOA
#define K_C2_PIN        LL_GPIO_PIN_15
#define K_C3_PORT       GPIOB
#define K_C3_PIN        LL_GPIO_PIN_3

// LoRa Module SPI
#define LR_SPI_NSS_PORT   GPIOA
#define LR_SPI_NSS_PIN    LL_GPIO_PIN_4
#define LR_SPI_SCK_PORT   GPIOA
#define LR_SPI_SCK_PIN    LL_GPIO_PIN_5
#define LR_SPI_MISO_PORT  GPIOA
#define LR_SPI_MISO_PIN   LL_GPIO_PIN_6
#define LR_SPI_MOSI_PORT  GPIOA
#define LR_SPI_MOSI_PIN   LL_GPIO_PIN_7
#define LR_SPI_NRST_PORT  GPIOB
#define LR_SPI_NRST_PIN   LL_GPIO_PIN_0
#define LR_SPI_BUSY_PORT  GPIOB
#define LR_SPI_BUSY_PIN   LL_GPIO_PIN_1

// LoRa RF Switch Control
#define LR_TXEN_PORT      GPIOB
#define LR_TXEN_PIN       LL_GPIO_PIN_10
#define LR_RXEN_PORT      GPIOB
#define LR_RXEN_PIN       LL_GPIO_PIN_11

// I2C for OLED
#define OLED_I2C_SCL_PORT GPIOB
#define OLED_I2C_SCL_PIN  LL_GPIO_PIN_6
#define OLED_I2C_SDA_PORT GPIOB
#define OLED_I2C_SDA_PIN  LL_GPIO_PIN_7

// Debug LED (PC13 on Blue Pill, active low)
#define LED_PORT          GPIOC
#define LED_PIN           LL_GPIO_PIN_13

#define LED_ON()          LL_GPIO_ResetOutputPin(LED_PORT, LED_PIN)
#define LED_OFF()         LL_GPIO_SetOutputPin(LED_PORT, LED_PIN)
#define LED_TOGGLE()      LL_GPIO_TogglePin(LED_PORT, LED_PIN)

/* ======================== Macros ======================== */

#define LR_NSS_HIGH()   LL_GPIO_SetOutputPin(LR_SPI_NSS_PORT, LR_SPI_NSS_PIN)
#define LR_NSS_LOW()    LL_GPIO_ResetOutputPin(LR_SPI_NSS_PORT, LR_SPI_NSS_PIN)
#define LR_NRST_HIGH()  LL_GPIO_SetOutputPin(LR_SPI_NRST_PORT, LR_SPI_NRST_PIN)
#define LR_NRST_LOW()   LL_GPIO_ResetOutputPin(LR_SPI_NRST_PORT, LR_SPI_NRST_PIN)
#define LR_BUSY_READ()  LL_GPIO_IsInputPinSet(LR_SPI_BUSY_PORT, LR_SPI_BUSY_PIN)

/* ======================== System Functions ======================== */

void SystemClock_Config(void);
void Error_Handler(void);
void Delay_Ms(uint32_t ms);
uint32_t GetTick(void);

/* ======================== Watchdog ======================== */

void IWDG_Init(void);
void IWDG_Refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */