#include "nrf24l01.h"

#define NRF_CMD_R_REGISTER       0x00U
#define NRF_CMD_W_REGISTER       0x20U
#define NRF_CMD_R_RX_PAYLOAD     0x61U
#define NRF_CMD_FLUSH_RX         0xE2U
#define NRF_CMD_NOP              0xFFU

#define NRF_REG_CONFIG           0x00U
#define NRF_REG_EN_AA            0x01U
#define NRF_REG_EN_RXADDR        0x02U
#define NRF_REG_SETUP_AW         0x03U
#define NRF_REG_SETUP_RETR       0x04U
#define NRF_REG_RF_CH            0x05U
#define NRF_REG_RF_SETUP         0x06U
#define NRF_REG_STATUS           0x07U
#define NRF_REG_RX_ADDR_P0       0x0AU
#define NRF_REG_TX_ADDR          0x10U
#define NRF_REG_RX_PW_P0         0x11U
#define NRF_REG_FIFO_STATUS      0x17U
#define NRF_REG_DYNPD            0x1CU
#define NRF_REG_FEATURE          0x1DU

#define NRF_STATUS_RX_DR         0x40U
#define NRF_STATUS_TX_DS         0x20U
#define NRF_STATUS_MAX_RT        0x10U
#define NRF_FIFO_RX_EMPTY        0x01U

extern SPI_HandleTypeDef hspi2;

static const uint8_t rx_address[5] = {'C', 'A', 'R', '0', '1'};
static uint32_t last_packet_tick = 0;
//CSN = 0：选中 NRF24L01，开始 SPI 通信  CSN = 1：取消选中
static void csn_low(void)
{
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
}
static void csn_high(void)
{
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
}
//CE = 0：待机/配置 CE = 1：进入接收或发送工作状态
static void ce_low(void)
{
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);
}

static void ce_high(void)
{
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);
}
//SPI 收发函数, NRF24L01 通过 SPI 通信。STM32 发一个字节给 NRF，同时也会收到一个字节。
static NRF24L01_Status spi_txrx(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (HAL_SPI_TransmitReceive(&hspi2, tx, rx, len, 100) != HAL_OK) {
        csn_high();
        return NRF24L01_SPI_ERROR;
    }
    return NRF24L01_OK;
}
//读寄存器
static uint8_t read_register(uint8_t reg)
{
    uint8_t tx[2] = {NRF_CMD_R_REGISTER | (reg & 0x1FU), NRF_CMD_NOP};
    uint8_t rx[2] = {0};

    csn_low();
    (void)spi_txrx(tx, rx, 2);
    csn_high();

    return rx[1];
}
//写寄存器
static NRF24L01_Status write_register(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {NRF_CMD_W_REGISTER | (reg & 0x1FU), value};
    uint8_t rx[2] = {0};
    NRF24L01_Status ret;

    csn_low();
    ret = spi_txrx(tx, rx, 2);
    csn_high();

    return ret;
}

static NRF24L01_Status write_register_buf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t tx[6] = {0};
    uint8_t rx[6] = {0};
    NRF24L01_Status ret;
    uint8_t i;

    if (len > 5U) {
        len = 5U;
    }

    tx[0] = NRF_CMD_W_REGISTER | (reg & 0x1FU);
    for (i = 0; i < len; i++) {
        tx[i + 1U] = buf[i];
    }

    csn_low();
    ret = spi_txrx(tx, rx, (uint16_t)(len + 1U));
    csn_high();

    return ret;
}

static NRF24L01_Status send_command(uint8_t cmd)
{
    uint8_t tx = cmd;
    uint8_t rx = 0;
    NRF24L01_Status ret;

    csn_low();
    ret = spi_txrx(&tx, &rx, 1);
    csn_high();

    return ret;
}

static uint8_t checksum7(const uint8_t *data)
{
    uint8_t sum = 0;
    uint8_t i;

    for (i = 0; i < 7U; i++) {
        sum = (uint8_t)(sum + data[i]);
    }

    return sum;
}
//初始化接收模式
NRF24L01_Status NRF24L01_Init(void)
{
    ce_low(); // CE引脚拉低 → NRF进入待机模式（先别工作）
    csn_high(); // CSN引脚拉高 → SPI片选无效（先别通信）
    HAL_Delay(5);// 等5ms → 让NRF稳定上电
		//0x0C不上电写完所有配置
    if (write_register(NRF_REG_CONFIG, 0x0CU) != NRF24L01_OK) {
        return NRF24L01_SPI_ERROR;
    }
		
    write_register(NRF_REG_EN_AA, 0x01U);//0x01U = 0000 0001，只对管道0(Pipe0)开启自动应答
    write_register(NRF_REG_EN_RXADDR, 0x01U);//只使能管道0(Pipe0)接收数据
    write_register(NRF_REG_SETUP_AW, 0x03U);
    write_register(NRF_REG_SETUP_RETR, 0x00U);//自动重传 关闭
    write_register(NRF_REG_RF_CH, 40U);//无线频道 = 40号（2.4GHz + 40MHz = 2.440GHz）
    write_register(NRF_REG_RF_SETUP, 0x06U);
    write_register_buf(NRF_REG_RX_ADDR_P0, rx_address, 5U);
    write_register_buf(NRF_REG_TX_ADDR, rx_address, 5U);
    write_register(NRF_REG_RX_PW_P0, NRF24L01_PAYLOAD_SIZE);
    write_register(NRF_REG_DYNPD, 0x00U);
    write_register(NRF_REG_FEATURE, 0x00U);
    write_register(NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    send_command(NRF_CMD_FLUSH_RX);
		//0x0F上电
    write_register(NRF_REG_CONFIG, 0x0FU);
    HAL_Delay(2);
    ce_high();

    last_packet_tick = 0;
    return NRF24L01_OK;
}

NRF24L01_Status NRF24L01_ReadPacket(NRF24L01_Packet *packet)
{
    uint8_t status;// NRF状态寄存器的值
    uint8_t fifo_status;// FIFO状态寄存器的值
    uint8_t tx[NRF24L01_PAYLOAD_SIZE + 1U];// SPI发送缓冲区
    uint8_t rx[NRF24L01_PAYLOAD_SIZE + 1U];// SPI接收缓冲区
    uint8_t payload[NRF24L01_PAYLOAD_SIZE]; // 最终解出来的有效数据
    uint8_t i;
    NRF24L01_Status ret;

    if (packet == 0) {
        return NRF24L01_SPI_ERROR;
    }

    status = read_register(NRF_REG_STATUS);// 读状态寄存器
    fifo_status = read_register(NRF_REG_FIFO_STATUS);// 读FIFO状态
    //判断有无数据，没有直接返回
		if (((status & NRF_STATUS_RX_DR) == 0U) && ((fifo_status & NRF_FIFO_RX_EMPTY) != 0U)) {
        return NRF24L01_NO_DATA;
    }

    tx[0] = NRF_CMD_R_RX_PAYLOAD;// 第0个字节 = 读RX Payload命令
    for (i = 1U; i <= NRF24L01_PAYLOAD_SIZE; i++) {
        tx[i] = NRF_CMD_NOP;
    }

    csn_low();// 拉低CSN → 开始SPI通信
    ret = spi_txrx(tx, rx, NRF24L01_PAYLOAD_SIZE + 1U);
    csn_high(); // 拉高CSN → 结束SPI通信
    if (ret != NRF24L01_OK) {
        return ret;
    }

    write_register(NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);

    for (i = 0; i < NRF24L01_PAYLOAD_SIZE; i++) {
        payload[i] = rx[i + 1U];
    }

    if (checksum7(payload) != payload[7]) {
        return NRF24L01_CHECKSUM_ERROR;
    }

    packet->speed = (int16_t)((uint16_t)payload[0] | ((uint16_t)payload[1] << 8));
    packet->turn = (int16_t)((uint16_t)payload[2] | ((uint16_t)payload[3] << 8));
    packet->yaw = (int16_t)((uint16_t)payload[4] | ((uint16_t)payload[5] << 8));
    packet->seq = payload[6];
    packet->checksum = payload[7];
    last_packet_tick = HAL_GetTick();//记录收到时间

    return NRF24L01_OK;
}
//判断是否还连着
uint8_t NRF24L01_IsConnected(uint32_t now_ms)
{
		// 从来没收到过数据 → 不算连接
    if (last_packet_tick == 0U) {
        return 0;
    }

    return ((uint32_t)(now_ms - last_packet_tick) <= NRF24L01_PACKET_TIMEOUT_MS) ? 1U : 0U;
}
//返回最后收到的时间
uint32_t NRF24L01_LastPacketTick(void)
{
    return last_packet_tick;
}
