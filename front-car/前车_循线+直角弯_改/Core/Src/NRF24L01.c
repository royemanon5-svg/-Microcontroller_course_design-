#include "NRF24L01.h"

#define NRF_CMD_R_REGISTER     0x00U
#define NRF_CMD_W_REGISTER     0x20U
#define NRF_CMD_W_TX_PAYLOAD   0xA0U
#define NRF_CMD_FLUSH_TX       0xE1U
#define NRF_CMD_FLUSH_RX       0xE2U
#define NRF_CMD_NOP            0xFFU

#define NRF_REG_CONFIG         0x00U
#define NRF_REG_EN_AA          0x01U
#define NRF_REG_EN_RXADDR      0x02U
#define NRF_REG_SETUP_AW       0x03U
#define NRF_REG_SETUP_RETR     0x04U
#define NRF_REG_RF_CH          0x05U
#define NRF_REG_RF_SETUP       0x06U
#define NRF_REG_STATUS         0x07U
#define NRF_REG_RX_ADDR_P0     0x0AU
#define NRF_REG_TX_ADDR        0x10U
#define NRF_REG_RX_PW_P0       0x11U
#define NRF_REG_DYNPD          0x1CU
#define NRF_REG_FEATURE        0x1DU

#define NRF_STATUS_TX_DS       0x20U
#define NRF_STATUS_MAX_RT      0x10U
#define NRF_STATUS_IRQ_MASK    0x70U

static const uint8_t nrf_address[5] = {'C', 'A', 'R', '0', '1'};
static uint8_t tx_sequence = 1U;
static uint8_t last_tx_sequence = 0U;

static void nrf_delay_us(uint32_t us)
{
    uint32_t loops = (SystemCoreClock / 5000000U) * us;
    while (loops-- != 0U)
    {
        __NOP();
    }
}

static uint8_t nrf_spi_transfer(uint8_t data)
{
    uint8_t received = 0U;

    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin,
                          (data & 0x80U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data <<= 1;
        HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
        received <<= 1;
        if (HAL_GPIO_ReadPin(MISO_GPIO_Port, MISO_Pin) == GPIO_PIN_SET)
        {
            received |= 1U;
        }
        HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
    }
    return received;
}

static uint8_t nrf_command(uint8_t command)
{
    uint8_t status;
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    status = nrf_spi_transfer(command);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
    return status;
}

static void nrf_write_register(uint8_t reg, uint8_t value)
{
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    nrf_spi_transfer(NRF_CMD_W_REGISTER | (reg & 0x1FU));
    nrf_spi_transfer(value);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
}

static void nrf_write_registers(uint8_t reg, const uint8_t *data, uint8_t length)
{
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    nrf_spi_transfer(NRF_CMD_W_REGISTER | (reg & 0x1FU));
    while (length-- != 0U)
    {
        nrf_spi_transfer(*data++);
    }
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
}

uint8_t NRF24L01_ReadRegister(uint8_t reg)
{
    uint8_t value;
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    nrf_spi_transfer(NRF_CMD_R_REGISTER | (reg & 0x1FU));
    value = nrf_spi_transfer(NRF_CMD_NOP);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
    return value;
}

static void nrf_write_payload(const uint8_t payload[NRF24L01_PAYLOAD_SIZE])
{
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    nrf_spi_transfer(NRF_CMD_W_TX_PAYLOAD);
    for (uint8_t i = 0U; i < NRF24L01_PAYLOAD_SIZE; i++)
    {
        nrf_spi_transfer(payload[i]);
    }
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
}

uint8_t NRF24L01_Init(void)
{
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);

    nrf_write_register(NRF_REG_CONFIG, 0x0EU);
    /* Telemetry is one-way; do not make transmission depend on an ACK. */
    nrf_write_register(NRF_REG_EN_AA, 0x00U);
    nrf_write_register(NRF_REG_EN_RXADDR, 0x01U);
    nrf_write_register(NRF_REG_SETUP_AW, 0x03U);
    nrf_write_register(NRF_REG_SETUP_RETR, 0x00U);
    nrf_write_register(NRF_REG_RF_CH, 40U);
    nrf_write_register(NRF_REG_RF_SETUP, 0x06U);
    nrf_write_register(NRF_REG_RX_PW_P0, NRF24L01_PAYLOAD_SIZE);
    nrf_write_register(NRF_REG_DYNPD, 0x00U);
    nrf_write_register(NRF_REG_FEATURE, 0x00U);
    nrf_write_registers(NRF_REG_TX_ADDR, nrf_address, sizeof(nrf_address));
    nrf_write_registers(NRF_REG_RX_ADDR_P0, nrf_address, sizeof(nrf_address));
    nrf_write_register(NRF_REG_STATUS, NRF_STATUS_IRQ_MASK);
    nrf_command(NRF_CMD_FLUSH_TX);
    nrf_command(NRF_CMD_FLUSH_RX);
    HAL_Delay(2);

    return (NRF24L01_ReadRegister(NRF_REG_RF_CH) == 40U) ? 1U : 0U;
}

NRF24L01_TxResult NRF24L01_SendCarData(int16_t speed,
                                      int16_t turn,
                                      int16_t yaw_x10,
                                      uint32_t path_ticks)
{
    uint8_t payload[NRF24L01_PAYLOAD_SIZE];
    uint8_t checksum = 0U;
    uint32_t start;

    payload[0] = (uint8_t)speed;
    payload[1] = (uint8_t)((uint16_t)speed >> 8);
    payload[2] = (uint8_t)turn;
    payload[3] = (uint8_t)((uint16_t)turn >> 8);
    payload[4] = (uint8_t)yaw_x10;
    payload[5] = (uint8_t)((uint16_t)yaw_x10 >> 8);
    payload[6] = (uint8_t)path_ticks;
    payload[7] = (uint8_t)(path_ticks >> 8);
    payload[8] = (uint8_t)(path_ticks >> 16);
    payload[9] = (uint8_t)(path_ticks >> 24);
    payload[10] = tx_sequence++;
    last_tx_sequence = payload[10];
    for (uint8_t i = 0U; i < (NRF24L01_PAYLOAD_SIZE - 1U); i++)
    {
        checksum = (uint8_t)(checksum + payload[i]);
    }
    payload[11] = checksum;

    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);
    nrf_write_register(NRF_REG_STATUS, NRF_STATUS_IRQ_MASK);
    nrf_command(NRF_CMD_FLUSH_TX);
    nrf_write_payload(payload);
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);
    nrf_delay_us(20U);
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 5U)
    {
        uint8_t status = NRF24L01_ReadRegister(NRF_REG_STATUS);
        if ((status & NRF_STATUS_TX_DS) != 0U)
        {
            nrf_write_register(NRF_REG_STATUS, NRF_STATUS_TX_DS);
            return NRF24L01_TX_OK;
        }
        if ((status & NRF_STATUS_MAX_RT) != 0U)
        {
            nrf_write_register(NRF_REG_STATUS, NRF_STATUS_MAX_RT);
            nrf_command(NRF_CMD_FLUSH_TX);
            return NRF24L01_TX_MAX_RETRY;
        }
    }

    nrf_command(NRF_CMD_FLUSH_TX);
    return NRF24L01_TX_TIMEOUT;
}

uint8_t NRF24L01_LastSequence(void)
{
    return last_tx_sequence;
}
