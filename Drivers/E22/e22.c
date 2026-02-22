/**
  ******************************************************************************
  * @file           : e22.c
  * @brief          : E22-400M22S LoRa Module Driver Implementation
  ******************************************************************************
  */

#include "e22.h"

/* ======================== SX1268 Commands ======================== */

#define CMD_GET_STATUS              0xC0
#define CMD_WRITE_REGISTER          0x0D
#define CMD_READ_REGISTER           0x1D
#define CMD_WRITE_BUFFER            0x0E
#define CMD_READ_BUFFER             0x1E
#define CMD_SET_SLEEP               0x84
#define CMD_SET_STANDBY             0x80
#define CMD_SET_FS                  0xC1
#define CMD_SET_TX                  0x83
#define CMD_SET_RX                  0x82
#define CMD_SET_CAD                 0xC5
#define CMD_SET_TX_CONTINUOUS_WAVE  0xD1
#define CMD_SET_TX_INFINITE_PREAMBLE 0xD2
#define CMD_SET_PACKET_TYPE         0x8A
#define CMD_GET_PACKET_TYPE         0x11
#define CMD_SET_RF_FREQUENCY        0x86
#define CMD_SET_TX_PARAMS           0x8E
#define CMD_SET_PA_CONFIG           0x95
#define CMD_SET_CAD_PARAMS          0x88
#define CMD_SET_BUFFER_BASE_ADDRESS 0x8F
#define CMD_SET_MODULATION_PARAMS   0x8B
#define CMD_SET_PACKET_PARAMS       0x8C
#define CMD_GET_RX_BUFFER_STATUS    0x13
#define CMD_GET_PACKET_STATUS       0x14
#define CMD_GET_RSSI_INST           0x15
#define CMD_GET_STATS               0x10
#define CMD_RESET_STATS             0x00
#define CMD_CFG_DIO_IRQ             0x08
#define CMD_GET_IRQ_STATUS          0x12
#define CMD_CLR_IRQ_STATUS          0x02
#define CMD_CALIBRATE               0x89
#define CMD_CALIBRATE_IMAGE         0x98
#define CMD_SET_REGULATOR_MODE      0x96
#define CMD_GET_ERROR               0x17
#define CMD_CLR_ERROR               0x07
#define CMD_SET_DIO3_AS_TCXO_CTRL   0x97
#define CMD_SET_DIO2_AS_RF_SWITCH   0x9D
#define CMD_SET_STOP_RX_TIMER_ON_PREAMBLE 0x9F
#define CMD_SET_LORA_SYMB_NUM_TIMEOUT 0xA0

/* ======================== Register Addresses ======================== */

#define REG_LORA_SYNC_WORD_MSB      0x0740
#define REG_LORA_SYNC_WORD_LSB      0x0741
#define REG_XTA_TRIM                0x0911
#define REG_RX_GAIN                 0x08AC

/* ======================== Constants ======================== */

#define XTAL_FREQ                   32000000.0
#define FREQ_DIV                    33554432.0  // 2^25
#define FREQ_STEP                   (XTAL_FREQ / FREQ_DIV)

#define PACKET_TYPE_LORA            0x01
#define STDBY_RC                    0x00
#define STDBY_XOSC                  0x01
#define USE_DCDC                    0x01
#define TCXO_CTRL_3_3V              0x07

#define LORA_PREAMBLE_LENGTH        60

/* ======================== LoRa BW/SF Tables ======================== */

// Bandwidth register values
static const uint8_t BW_RegValues[] = {
    0x00,   // 7.81 kHz
    0x08,   // 10.42 kHz
    0x01,   // 15.63 kHz
    0x09,   // 20.83 kHz
    0x02,   // 31.25 kHz
    0x0A,   // 41.67 kHz
    0x03,   // 62.5 kHz
    0x04,   // 125 kHz
    0x05,   // 250 kHz
    0x06    // 500 kHz
};

static const char* BW_Strings[] = {
    "7.81k", "10.4k", "15.6k", "20.8k", "31.2k",
    "41.7k", "62.5k", "125k", "250k", "500k"
};

static const uint32_t BW_Hz[] = {
    7810, 10420, 15630, 20830, 31250,
    41670, 62500, 125000, 250000, 500000
};

/* ======================== Module State ======================== */

E22_Params_t E22_Params;
volatile bool E22_TxBusy = false;

static E22_TxDoneCallback_t txDoneCallback = NULL;
static E22_RxDoneCallback_t rxDoneCallback = NULL;
static E22_ErrorCallback_t errorCallback = NULL;

static uint8_t rxBuffer[E22_MAX_PACKET_SIZE];
static E22_PacketStatus_t lastPacketStatus;

/* ======================== SPI Functions ======================== */

static void WaitBusy(void)
{
    // 增加超时时间，LoRa操作可能需要较长时间
    uint32_t timeout = 100000;
    while(LR_BUSY_READ() && timeout--) {
        for(volatile int i = 0; i < 10; i++);
    }
}

static uint8_t SpiTransfer(uint8_t data)
{
    while(!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, data);
    while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
    return LL_SPI_ReceiveData8(SPI1);
}

static void WriteCommand(uint8_t cmd, uint8_t* data, uint8_t len)
{
    WaitBusy();
    LR_NSS_LOW();
    
    SpiTransfer(cmd);
    for(uint8_t i = 0; i < len; i++) {
        SpiTransfer(data[i]);
    }
    
    LR_NSS_HIGH();
    
    // 非休眠命令后需要等待BUSY
    if(cmd != CMD_SET_SLEEP) {
        WaitBusy();
    }
}

static void ReadCommand(uint8_t cmd, uint8_t* data, uint8_t len)
{
    WaitBusy();
    LR_NSS_LOW();
    
    SpiTransfer(cmd);
    SpiTransfer(0x00);  // NOP for status
    for(uint8_t i = 0; i < len; i++) {
        data[i] = SpiTransfer(0x00);
    }
    
    LR_NSS_HIGH();
}

static void WriteRegister(uint16_t addr, uint8_t* data, uint8_t len)
{
    WaitBusy();
    LR_NSS_LOW();
    
    SpiTransfer(CMD_WRITE_REGISTER);
    SpiTransfer((addr >> 8) & 0xFF);
    SpiTransfer(addr & 0xFF);
    for(uint8_t i = 0; i < len; i++) {
        SpiTransfer(data[i]);
    }
    
    LR_NSS_HIGH();
}

static void ReadRegister(uint16_t addr, uint8_t* data, uint8_t len)
{
    WaitBusy();
    LR_NSS_LOW();
    
    SpiTransfer(CMD_READ_REGISTER);
    SpiTransfer((addr >> 8) & 0xFF);
    SpiTransfer(addr & 0xFF);
    SpiTransfer(0x00);  // NOP
    for(uint8_t i = 0; i < len; i++) {
        data[i] = SpiTransfer(0x00);
    }
    
    LR_NSS_HIGH();
}

static void WriteBuffer(uint8_t offset, uint8_t* data, uint8_t len)
{
    WaitBusy();
    LR_NSS_LOW();
    
    SpiTransfer(CMD_WRITE_BUFFER);
    SpiTransfer(offset);
    for(uint8_t i = 0; i < len; i++) {
        SpiTransfer(data[i]);
    }
    
    LR_NSS_HIGH();
}

static void ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len)
{
    WaitBusy();
    LR_NSS_LOW();
    
    SpiTransfer(CMD_READ_BUFFER);
    SpiTransfer(offset);
    SpiTransfer(0x00);  // NOP
    for(uint8_t i = 0; i < len; i++) {
        data[i] = SpiTransfer(0x00);
    }
    
    LR_NSS_HIGH();
}

/* ======================== Module Control ======================== */

static void SetStandby(uint8_t mode)
{
    uint8_t data = mode;
    WriteCommand(CMD_SET_STANDBY, &data, 1);
}

static void SetPacketType(uint8_t type)
{
    WriteCommand(CMD_SET_PACKET_TYPE, &type, 1);
}

static void SetDio3AsTcxoCtrl(uint8_t voltage, uint16_t delay)
{
    uint8_t data[4];
    data[0] = voltage;
    data[1] = (delay >> 16) & 0xFF;
    data[2] = (delay >> 8) & 0xFF;
    data[3] = delay & 0xFF;
    WriteCommand(CMD_SET_DIO3_AS_TCXO_CTRL, data, 4);
}

static void SetDio2AsRfSwitchCtrl(uint8_t enable)
{
    WriteCommand(CMD_SET_DIO2_AS_RF_SWITCH, &enable, 1);
}

static void SetRegulatorMode(uint8_t mode)
{
    WriteCommand(CMD_SET_REGULATOR_MODE, &mode, 1);
}

static void SetBufferBaseAddress(uint8_t txBase, uint8_t rxBase)
{
    uint8_t data[2] = {txBase, rxBase};
    WriteCommand(CMD_SET_BUFFER_BASE_ADDRESS, data, 2);
}

static void Calibrate(uint8_t calibParam)
{
    WriteCommand(CMD_CALIBRATE, &calibParam, 1);
    Delay_Ms(10);
}

static void SetRfFrequencyRaw(uint32_t freq)
{
    uint32_t freqReg = (uint32_t)((double)freq / FREQ_STEP);
    uint8_t data[4];
    data[0] = (freqReg >> 24) & 0xFF;
    data[1] = (freqReg >> 16) & 0xFF;
    data[2] = (freqReg >> 8) & 0xFF;
    data[3] = freqReg & 0xFF;
    WriteCommand(CMD_SET_RF_FREQUENCY, data, 4);
}

static void SetPaConfig(int8_t power)
{
    // PA config for SX1268: paDutyCycle, hpMax, deviceSel, paLut
    uint8_t data[4] = {0x04, 0x07, 0x00, 0x01};
    WriteCommand(CMD_SET_PA_CONFIG, data, 4);
}

static void SetTxParams(int8_t power, uint8_t rampTime)
{
    if(power > 22) power = 22;
    if(power < -9) power = -9;
    
    uint8_t data[2] = {(uint8_t)power, rampTime};
    WriteCommand(CMD_SET_TX_PARAMS, data, 2);
}

static void SetModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro)
{
    uint8_t data[4];
    data[0] = sf;
    data[1] = BW_RegValues[bw];
    data[2] = cr;
    data[3] = ldro;
    WriteCommand(CMD_SET_MODULATION_PARAMS, data, 4);
}

static void SetPacketParams(uint16_t preambleLen, uint8_t headerType, uint8_t payloadLen, uint8_t crcType, uint8_t invertIQ)
{
    uint8_t data[6];
    data[0] = (preambleLen >> 8) & 0xFF;
    data[1] = preambleLen & 0xFF;
    data[2] = headerType;
    data[3] = payloadLen;
    data[4] = crcType;
    data[5] = invertIQ;
    WriteCommand(CMD_SET_PACKET_PARAMS, data, 6);
}

static void SetLoraSyncWord(uint16_t syncWord)
{
    uint8_t data[2];
    data[0] = (syncWord >> 8) & 0xFF;
    data[1] = syncWord & 0xFF;
    WriteRegister(REG_LORA_SYNC_WORD_MSB, data, 2);
}

static uint16_t GetIrqStatus(void)
{
    uint8_t data[2];
    ReadCommand(CMD_GET_IRQ_STATUS, data, 2);
    return ((uint16_t)data[0] << 8) | data[1];
}

static void ClearIrqStatus(uint16_t irq)
{
    uint8_t data[2];
    data[0] = (irq >> 8) & 0xFF;
    data[1] = irq & 0xFF;
    WriteCommand(CMD_CLR_IRQ_STATUS, data, 2);
}

// 配置DIO IRQ映射
static void SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t data[8];
    data[0] = (irqMask >> 8) & 0xFF;
    data[1] = irqMask & 0xFF;
    data[2] = (dio1Mask >> 8) & 0xFF;
    data[3] = dio1Mask & 0xFF;
    data[4] = (dio2Mask >> 8) & 0xFF;
    data[5] = dio2Mask & 0xFF;
    data[6] = (dio3Mask >> 8) & 0xFF;
    data[7] = dio3Mask & 0xFF;
    WriteCommand(0x08, data, 8);  // RADIO_CFG_DIOIRQ
}

// 控制TXEN/RXEN引脚进入发送模式
static void RfIoEnterTxMode(void)
{
    LR_RXEN_LOW();   // 关闭RX
    LR_TXEN_HIGH();  // 打开TX
}

// 控制TXEN/RXEN引脚进入接收模式  
static void RfIoEnterRxMode(void)
{
    LR_TXEN_LOW();   // 关闭TX
    LR_RXEN_HIGH();  // 打开RX
}

// 关闭RF开关(休眠时使用)
static void RfIoDisable(void)
{
    LR_TXEN_LOW();
    LR_RXEN_LOW();
}

static void SetTx(uint32_t timeout)
{
    uint8_t data[3];
    data[0] = (timeout >> 16) & 0xFF;
    data[1] = (timeout >> 8) & 0xFF;
    data[2] = timeout & 0xFF;
    WriteCommand(CMD_SET_TX, data, 3);
}

static void SetRx(uint32_t timeout)
{
    uint8_t data[3];
    data[0] = (timeout >> 16) & 0xFF;
    data[1] = (timeout >> 8) & 0xFF;
    data[2] = timeout & 0xFF;
    WriteCommand(CMD_SET_RX, data, 3);
}

static void GetRxBufferStatus(uint8_t* payloadLen, uint8_t* rxStartBufferPointer)
{
    uint8_t data[2];
    ReadCommand(CMD_GET_RX_BUFFER_STATUS, data, 2);
    *payloadLen = data[0];
    *rxStartBufferPointer = data[1];
}

static void GetPacketStatusRaw(int8_t* rssi, int8_t* snr)
{
    uint8_t data[3];
    ReadCommand(CMD_GET_PACKET_STATUS, data, 3);
    *rssi = -(int8_t)(data[0] / 2);
    *snr = (int8_t)data[1] / 4;
}

/* ======================== Public Functions ======================== */

void E22_Init(void)
{
    // Initialize default parameters
    E22_Params.frequency = E22_DEFAULT_FREQ;
    E22_Params.txPower = E22_DEFAULT_TX_POWER;
    E22_Params.sf = E22_DEFAULT_SF;
    E22_Params.bw = E22_DEFAULT_BW;  // 500kHz
    E22_Params.cr = E22_DEFAULT_CR;
    E22_Params.syncWord = 0x3444;  // Public LoRa sync word
    E22_Params.preambleLen = LORA_PREAMBLE_LENGTH;
    
    E22_TxBusy = false;
    
    // Hardware reset
    E22_Reset();
    
    // Wakeup
    LR_NSS_LOW();
    Delay_Ms(1);
    LR_NSS_HIGH();
    Delay_Ms(5);
    
    // Enter standby RC mode
    SetStandby(STDBY_RC);
    
    // Configure TCXO (DIO3 control, 3.3V)
    SetDio3AsTcxoCtrl(TCXO_CTRL_3_3V, 320);
    
    // Write XTA trim value
    uint8_t trim = 0x2F;
    WriteRegister(REG_XTA_TRIM, &trim, 1);
    
    // Calibrate all
    Calibrate(0xFF);
    
    // Enter standby XOSC mode
    SetStandby(STDBY_XOSC);
    
    // 启用DIO2作为RF开关控制(模块内部开关)
    // 同时外部TXEN/RXEN控制PA/LNA
    SetDio2AsRfSwitchCtrl(1);
    
    // Use DC-DC regulator
    SetRegulatorMode(USE_DCDC);
    
    // Set buffer base addresses
    SetBufferBaseAddress(0x00, 0x00);
    
    // Set packet type to LoRa
    SetPacketType(PACKET_TYPE_LORA);
    
    // Apply default parameters
    E22_ApplyParams();
    
    // Enter receive mode
    E22_StartReceive();
}

void E22_Reset(void)
{
    LR_NRST_LOW();
    Delay_Ms(20);
    LR_NRST_HIGH();
    Delay_Ms(50);
}

void E22_SetFrequency(uint32_t freq)
{
    E22_Params.frequency = freq;
}

void E22_SetTxPower(int8_t power)
{
    if(power > 22) power = 22;
    if(power < -9) power = -9;
    E22_Params.txPower = power;
}

void E22_SetSpreadingFactor(uint8_t sf)
{
    if(sf < 5) sf = 5;
    if(sf > 12) sf = 12;
    E22_Params.sf = sf;
}

void E22_SetBandwidth(uint8_t bw)
{
    if(bw > 9) bw = 9;
    E22_Params.bw = bw;
}

void E22_SetCodingRate(uint8_t cr)
{
    if(cr < 1) cr = 1;
    if(cr > 4) cr = 4;
    E22_Params.cr = cr;
}

void E22_SetSyncWord(uint16_t syncWord)
{
    E22_Params.syncWord = syncWord;
}

void E22_ApplyParams(void)
{
    // LDRO (Low Data Rate Optimization) - 始终开启更安全
    // 示例代码中也是固定开启的
    uint8_t ldro = 0x01;
    
    // Set modulation parameters
    SetModulationParams(E22_Params.sf, E22_Params.bw, E22_Params.cr, ldro);
    
    // Set sync word
    SetLoraSyncWord(E22_Params.syncWord);
    
    // Set packet parameters (variable length, CRC on, standard IQ)
    SetPacketParams(E22_Params.preambleLen, 0x00, 0xFF, 0x01, 0x00);
    
    // Set frequency
    SetRfFrequencyRaw(E22_Params.frequency);
    
    // Set PA config and TX power
    SetPaConfig(E22_Params.txPower);
    SetTxParams(E22_Params.txPower, 0x07);  // 3.4ms ramp time
}

void E22_GetParams(E22_Params_t* params)
{
    memcpy(params, &E22_Params, sizeof(E22_Params_t));
}

void E22_Send(uint8_t* data, uint8_t size)
{
    if(size > E22_MAX_PACKET_SIZE) size = E22_MAX_PACKET_SIZE;
    
    E22_TxBusy = true;
    
    // 1. 配置IRQ: 启用TX_DONE中断
    SetDioIrqParams(E22_IRQ_TX_DONE | E22_IRQ_TIMEOUT, 
                    E22_IRQ_TX_DONE | E22_IRQ_TIMEOUT,  // DIO1
                    0, 0);
    
    // 2. 清除之前的IRQ状态
    ClearIrqStatus(E22_IRQ_TX_DONE | E22_IRQ_TIMEOUT);
    
    // 3. 设置数据包参数(payload长度)
    SetPacketParams(E22_Params.preambleLen, 0x00, size, 0x01, 0x00);
    
    // 4. 写入数据到FIFO
    WriteBuffer(0x00, data, size);
    
    // 5. 切换RF IO进入TX模式
    RfIoEnterTxMode();
    
    // 6. 发送命令 (timeout = 0 表示无超时)
    uint8_t txCmd[3] = {0, 0, 0};
    WriteCommand(CMD_SET_TX, txCmd, 3);
    
    // 7. 等待TX完成 (阻塞模式，最长等待60秒)
    uint16_t irqStatus = 0;
    uint32_t timeout = 60000;  // 60秒超时
    do {
        Delay_Ms(1);
        irqStatus = GetIrqStatus();
        timeout--;
    } while(!(irqStatus & (E22_IRQ_TX_DONE | E22_IRQ_TIMEOUT)) && timeout > 0);
    
    // 8. 清除IRQ状态
    ClearIrqStatus(irqStatus);
    
    E22_TxBusy = false;
    
    // 9. 回调
    if(irqStatus & E22_IRQ_TX_DONE) {
        if(txDoneCallback) {
            txDoneCallback();
        }
    } else if(errorCallback) {
        errorCallback(irqStatus);
    }
    
    // 10. 返回接收模式
    E22_StartReceive();
}

void E22_StartReceive(void)
{
    // 1. 配置IRQ: 启用RX_DONE和错误中断
    SetDioIrqParams(E22_IRQ_RX_DONE | E22_IRQ_CRC_ERROR | E22_IRQ_HEADER_ERROR,
                    E22_IRQ_RX_DONE,  // DIO1映射到RX_DONE
                    0, 0);
    
    // 2. 设置数据包参数(最大长度接收)
    SetPacketParams(E22_Params.preambleLen, 0x00, 0xFF, 0x01, 0x00);
    
    // 3. 清除之前的IRQ状态
    ClearIrqStatus(0xFFFF);  // 清除所有IRQ
    
    // 4. 切换RF IO进入RX模式
    RfIoEnterRxMode();
    
    // 5. 进入连续接收模式 (0xFFFFFF = 无超时，连续接收)
    uint8_t rxCmd[3] = {0xFF, 0xFF, 0xFF};
    WriteCommand(CMD_SET_RX, rxCmd, 3);
}

void E22_SetSleep(void)
{
    // 关闭RF开关
    RfIoDisable();
    
    uint8_t data = 0x00;  // Cold start, no config retention
    WriteCommand(CMD_SET_SLEEP, &data, 1);
}

void E22_SetStandby(void)
{
    SetStandby(STDBY_RC);
}

void E22_PollTask(void)
{
    uint16_t irqStatus = GetIrqStatus();
    
    if(irqStatus != 0) {
        // 清除IRQ状态
        ClearIrqStatus(irqStatus);
        
        if(irqStatus & E22_IRQ_RX_DONE) {
            // 获取接收缓冲区状态
            uint8_t payloadLen, rxStartPtr;
            GetRxBufferStatus(&payloadLen, &rxStartPtr);
            
            if(payloadLen > 0 && payloadLen <= E22_MAX_PACKET_SIZE) {
                // 读取数据
                ReadBuffer(rxStartPtr, rxBuffer, payloadLen);
                
                // 获取包状态 (RSSI, SNR)
                int8_t rssi, snr;
                GetPacketStatusRaw(&rssi, &snr);
                
                lastPacketStatus.rssi = rssi;
                lastPacketStatus.snr = snr;
                lastPacketStatus.status = irqStatus & 0xFF;
                
                // 回调
                if(rxDoneCallback) {
                    rxDoneCallback(rxBuffer, payloadLen, rssi, snr);
                }
            }
            
            // 继续接收
            E22_StartReceive();
        }
        else if(irqStatus & (E22_IRQ_CRC_ERROR | E22_IRQ_HEADER_ERROR)) {
            // CRC或头部错误
            if(errorCallback) {
                errorCallback(irqStatus);
            }
            // 继续接收
            E22_StartReceive();
        }
        else if(irqStatus & E22_IRQ_TIMEOUT) {
            // 超时 (连续接收模式下不应该发生)
            if(errorCallback) {
                errorCallback(irqStatus);
            }
            // 重新进入接收
            E22_StartReceive();
        }
    }
}

void E22_SetTxDoneCallback(E22_TxDoneCallback_t cb)
{
    txDoneCallback = cb;
}

void E22_SetRxDoneCallback(E22_RxDoneCallback_t cb)
{
    rxDoneCallback = cb;
}

void E22_SetErrorCallback(E22_ErrorCallback_t cb)
{
    errorCallback = cb;
}

bool E22_IsBusy(void)
{
    return LR_BUSY_READ();
}

bool E22_IsTxBusy(void)
{
    return E22_TxBusy;
}

void E22_GetPacketStatus(E22_PacketStatus_t* status)
{
    memcpy(status, &lastPacketStatus, sizeof(E22_PacketStatus_t));
}

const char* E22_GetBandwidthString(uint8_t bw)
{
    if(bw > 9) return "???";
    return BW_Strings[bw];
}

uint32_t E22_GetBandwidthHz(uint8_t bw)
{
    if(bw > 9) return 0;
    return BW_Hz[bw];
}