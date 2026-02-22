/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * LoRa Node - 基于STM32F103C8T6的LoRa通信节点
  * 
  * 功能:
  * - 4x4矩阵键盘输入 (支持九键输入法)
  * - 0.91寸OLED显示 (128x32, I2C接口)
  * - E22-400M22S LoRa模块通信
  * - UART协议与上位机通信
  * - 独立看门狗保护
  ******************************************************************************
  */

#include "main.h"
#include "oled.h"
#include "keyboard.h"
#include "e22.h"
#include "uart_protocol.h"
#include "app.h"

/* ======================== System Tick ======================== */

static volatile uint32_t sysTick = 0;

uint32_t GetTick(void)
{
    return sysTick;
}

void Delay_Ms(uint32_t ms)
{
    uint32_t start = sysTick;
    while((sysTick - start) < ms) {
        IWDG_Refresh();
    }
}

/* ======================== SysTick Handler ======================== */

void SysTick_Handler(void)
{
    sysTick++;
}

/* ======================== USART1 IRQ Handler ======================== */

void USART1_IRQHandler(void)
{
    if(LL_USART_IsActiveFlag_RXNE(USART1)) {
        uint8_t data = LL_USART_ReceiveData8(USART1);
        UART_RxCallback(data);
    }
    
    if(LL_USART_IsActiveFlag_ORE(USART1)) {
        LL_USART_ClearFlag_ORE(USART1);
    }
}

/* ======================== Watchdog ======================== */

void IWDG_Init(void)
{
    // Enable LSI
    LL_RCC_LSI_Enable();
    while(!LL_RCC_LSI_IsReady());
    
    // Enable write access to IWDG registers
    LL_IWDG_EnableWriteAccess(IWDG);
    
    // Set prescaler to 64 (LSI = 40kHz, 40000/64 = 625Hz)
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_64);
    
    // Set reload value for ~2 second timeout
    // 625Hz * 2s = 1250
    LL_IWDG_SetReloadCounter(IWDG, 125000);
    
    // Wait for registers to be updated
    while(LL_IWDG_IsActiveFlag_PVU(IWDG));
    while(LL_IWDG_IsActiveFlag_RVU(IWDG));
    
    // Reload counter
    LL_IWDG_ReloadCounter(IWDG);
    
    // Enable IWDG
    LL_IWDG_Enable(IWDG);
}

void IWDG_Refresh(void)
{
    LL_IWDG_ReloadCounter(IWDG);
}

/* ======================== GPIO Initialization ======================== */

static void MX_GPIO_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIO clocks
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
    
    // Disable JTAG to free PA15 and PB3
    LL_GPIO_AF_Remap_SWJ_NOJTAG();
    
    // Debug LED (PC13) - Output, active low
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
    LED_OFF();  // LED off initially
    
    // Keyboard Row Outputs (PA3, PA2, PA1, PA0)
    GPIO_InitStruct.Pin = K_R0_PIN | K_R1_PIN | K_R2_PIN | K_R3_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Set rows high initially
    LL_GPIO_SetOutputPin(GPIOA, K_R0_PIN | K_R1_PIN | K_R2_PIN | K_R3_PIN);
    
    // Keyboard Column Inputs with pull-up (PA11, PA12, PA15)
    // F1系列上拉输入: Mode=INPUT, 然后设置ODR=1
    GPIO_InitStruct.Pin = K_C0_PIN | K_C1_PIN | K_C2_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    LL_GPIO_SetOutputPin(GPIOA, K_C0_PIN | K_C1_PIN | K_C2_PIN);  // Enable pull-up
    
    // Keyboard Column Input (PB3) with pull-up
    GPIO_InitStruct.Pin = K_C3_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    LL_GPIO_SetOutputPin(GPIOB, K_C3_PIN);  // Enable pull-up
    
    // LoRa NSS (PA4) - Output, initially high
    GPIO_InitStruct.Pin = LR_SPI_NSS_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(LR_SPI_NSS_PORT, &GPIO_InitStruct);
    LL_GPIO_SetOutputPin(LR_SPI_NSS_PORT, LR_SPI_NSS_PIN);
    
    // LoRa NRST (PB0) - Output, initially high
    GPIO_InitStruct.Pin = LR_SPI_NRST_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(LR_SPI_NRST_PORT, &GPIO_InitStruct);
    LL_GPIO_SetOutputPin(LR_SPI_NRST_PORT, LR_SPI_NRST_PIN);
    
    // LoRa BUSY (PB1) - Input floating
    GPIO_InitStruct.Pin = LR_SPI_BUSY_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
    LL_GPIO_Init(LR_SPI_BUSY_PORT, &GPIO_InitStruct);
}

/* ======================== SPI1 Initialization ======================== */

static void MX_SPI1_Init(void)
{
    LL_SPI_InitTypeDef SPI_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable SPI1 clock
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
    
    // SPI1 GPIO: PA5(SCK), PA6(MISO), PA7(MOSI)
    GPIO_InitStruct.Pin = LR_SPI_SCK_PIN | LR_SPI_MOSI_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = LR_SPI_MISO_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // SPI1 configuration
    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;  // 72MHz/8 = 9MHz
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    LL_SPI_Init(SPI1, &SPI_InitStruct);
    
    // Enable SPI1
    LL_SPI_Enable(SPI1);
}

/* ======================== I2C1 Initialization ======================== */

static void MX_I2C1_Init(void)
{
    LL_I2C_InitTypeDef I2C_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable I2C1 clock
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
    
    // I2C1 GPIO: PB6(SCL), PB7(SDA)
    GPIO_InitStruct.Pin = OLED_I2C_SCL_PIN | OLED_I2C_SDA_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // I2C1 configuration
    LL_I2C_Disable(I2C1);
    
    I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
    I2C_InitStruct.ClockSpeed = 400000;  // 400kHz Fast mode
    I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
    I2C_InitStruct.OwnAddress1 = 0x00;
    I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
    I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
    LL_I2C_Init(I2C1, &I2C_InitStruct);
    
    LL_I2C_Enable(I2C1);
}

/* ======================== USART1 Initialization ======================== */

static void MX_USART1_Init(void)
{
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable USART1 clock
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    
    // USART1 GPIO: PA9(TX), PA10(RX)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // USART1 configuration
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    LL_USART_Init(USART1, &USART_InitStruct);
    
    // Enable USART1
    LL_USART_Enable(USART1);
    
    // Enable RXNE interrupt
    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
    LL_USART_EnableIT_RXNE(USART1);
}

/* ======================== System Clock Configuration ======================== */

void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    
    if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
        Error_Handler();
    }
    
    // Enable HSE
    LL_RCC_HSE_Enable();
    while(!LL_RCC_HSE_IsReady());
    
    // Configure PLL
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
    LL_RCC_PLL_Enable();
    while(!LL_RCC_PLL_IsReady());
    
    // Set prescalers
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    
    // Set PLL as system clock source
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);
    
    // Update SystemCoreClock
    LL_SetSystemCoreClock(72000000);
    
    // Configure SysTick for 1ms interrupt
    SysTick_Config(72000000 / 1000);
}

/* ======================== Error Handler ======================== */

void Error_Handler(void)
{
    __disable_irq();
    while(1) {
        // Flash LED or something if available
    }
}

/* ======================== Main Function ======================== */

// 简单的键盘硬件测试
static void Keyboard_HardwareTest(void)
{
    char buf[24];
    
    OLED_Clear();
    OLED_DrawString(0, 0, "Key Test (3s)", 1);
    OLED_DrawString(0, 8, "Press any key...", 1);
    OLED_Update();
    
    uint32_t start = GetTick();
    while((GetTick() - start) < 3000) {
        IWDG_Refresh();
        
        // 直接读取列状态
        uint8_t c0 = LL_GPIO_IsInputPinSet(K_C0_PORT, K_C0_PIN) ? 1 : 0;
        uint8_t c1 = LL_GPIO_IsInputPinSet(K_C1_PORT, K_C1_PIN) ? 1 : 0;
        uint8_t c2 = LL_GPIO_IsInputPinSet(K_C2_PORT, K_C2_PIN) ? 1 : 0;
        uint8_t c3 = LL_GPIO_IsInputPinSet(K_C3_PORT, K_C3_PIN) ? 1 : 0;
        
        // 扫描键盘
        Keyboard_Scan();
        uint8_t key = Keyboard.releasedKey;
        
        // 显示状态
        snprintf(buf, sizeof(buf), "Col:%d%d%d%d Key:%02X", c0, c1, c2, c3, key);
        OLED_ClearArea(0, 16, 128, 16);
        OLED_DrawString(0, 16, buf, 1);
        OLED_Update();
        
        Delay_Ms(50);
    }
}

int main(void)
{
    // System initialization
    SystemClock_Config();
    
    // Peripheral initialization
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_I2C1_Init();
    MX_USART1_Init();
    
    // Initialize watchdog (2 second timeout)
    IWDG_Init();
    
    // Initialize drivers
    Delay_Ms(100);  // Wait for peripherals to stabilize
    
    OLED_Init();
    Keyboard_Init();
    
    // 键盘硬件测试 (启动时运行3秒)
    //Keyboard_HardwareTest();
    
    // Show startup message
    OLED_Clear();
    OLED_DrawString(20, 8, "LoRa Node", 1);
    OLED_DrawString(20, 18, "Starting...", 1);
    OLED_Update();
    
    // Initialize E22 LoRa module
    E22_Init();
    
    // Initialize UART protocol
    UART_Protocol_Init();
    
    // Initialize application
    App_Init();
    
    // Main loop
    while(1) {
        // Feed watchdog
        IWDG_Refresh();
        
        // Run application task
        App_Task();
        
        // Process UART (polling backup, in case interrupt missed)
        // UART_Protocol_Process();
        
        // Small delay to avoid excessive CPU usage
        // Note: Most delay happens naturally in the app task
    }
}


#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
