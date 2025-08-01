// nslp_dma.c
#include "nslp_dma.h"
#include <string.h>

static UART_HandleTypeDef *nslp_uart;
static CRC_HandleTypeDef *nslp_crc;

static uint8_t rx_buffer[MAX_PACKET_SIZE];
static uint8_t tx_buffer[MAX_PACKET_SIZE];

static void (*rx_callback)(struct Packet *) = NULL;

// TX queue
static struct Packet *tx_queue[TX_QUEUE_LENGTH];
static uint8_t tx_head = 0, tx_tail = 0, tx_count = 0;
static uint8_t tx_busy = 0;

// Init function
void nslp_init(UART_HandleTypeDef *huart, CRC_HandleTypeDef *hcrc) {
    nslp_uart = huart;
    nslp_crc = hcrc;

    __HAL_UART_ENABLE_IT(nslp_uart, UART_IT_IDLE);
    HAL_StatusTypeDef r = HAL_UARTEx_ReceiveToIdle_DMA(nslp_uart, rx_buffer, MAX_PACKET_SIZE);
    if (r != HAL_OK) {
        Error_Handler(); // or blink LED here
    }
    __HAL_DMA_DISABLE_IT(nslp_uart->hdmarx, DMA_IT_HT);

}

void nslp_set_rx_callback(void (*callback)(struct Packet *)) {
    rx_callback = callback;
}

static void start_tx(void) {
    if (tx_count == 0) return;

    struct Packet *p = tx_queue[tx_tail];

    size_t packet_size = HEADER_SIZE + p->size;
    size_t total_size = FRAME_START_SIZE + packet_size + CHECKSUM_SIZE;

    tx_buffer[0] = FRAME_START;
    tx_buffer[1] = p->type;
    tx_buffer[2] = p->size;

    memcpy(&tx_buffer[3], p->payload, p->size);

    uint32_t crc = HAL_CRC_Calculate(nslp_crc, (uint32_t *)&tx_buffer[1], HEADER_SIZE + p->size);
    memcpy(&tx_buffer[3 + p->size], &crc, 4);

    tx_busy = 1;
    HAL_UART_Transmit_DMA(nslp_uart, tx_buffer, total_size);
}

void nslp_send_packet(struct Packet *packet) {
    if (!packet || packet->size > MAX_PAYLOAD_SIZE || tx_count >= TX_QUEUE_LENGTH) return;

    tx_queue[tx_head] = packet;
    tx_head = (tx_head + 1) % TX_QUEUE_LENGTH;
    tx_count++;

    if (!tx_busy) {
        start_tx();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart != nslp_uart) return;

    tx_tail = (tx_tail + 1) % TX_QUEUE_LENGTH;
    tx_count--;
    tx_busy = 0;
    start_tx();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    if (huart != nslp_uart || size < FRAME_START_SIZE + HEADER_SIZE + CHECKSUM_SIZE) {
        HAL_UARTEx_ReceiveToIdle_DMA(nslp_uart, rx_buffer, MAX_PACKET_SIZE);
        __HAL_DMA_DISABLE_IT(nslp_uart->hdmarx, DMA_IT_HT);
        return;
    }

    if (rx_buffer[0] != FRAME_START) {
        HAL_UARTEx_ReceiveToIdle_DMA(nslp_uart, rx_buffer, MAX_PACKET_SIZE);
        return;
    }

    uint8_t type = rx_buffer[1];
    uint8_t payload_size = rx_buffer[2];

    if (payload_size > MAX_PAYLOAD_SIZE || size < FRAME_START_SIZE + HEADER_SIZE + payload_size + CHECKSUM_SIZE) {
        HAL_UARTEx_ReceiveToIdle_DMA(nslp_uart, rx_buffer, MAX_PACKET_SIZE);
        return;
    }

    uint32_t received_crc;
    memcpy(&received_crc, &rx_buffer[3 + payload_size], 4);

    __HAL_CRC_DR_RESET(nslp_crc);
    uint32_t computed_crc = HAL_CRC_Calculate(nslp_crc, (uint32_t *)&rx_buffer[1], HEADER_SIZE + payload_size);
    if (received_crc != computed_crc) {
        HAL_UARTEx_ReceiveToIdle_DMA(nslp_uart, rx_buffer, MAX_PACKET_SIZE);
        return;
    }

    struct Packet pkt = {
        .type = type,
        .size = payload_size,
        .payload = &rx_buffer[3]
    };

    if (rx_callback) {
        rx_callback(&pkt);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(nslp_uart, rx_buffer, MAX_PACKET_SIZE);
    __HAL_DMA_DISABLE_IT(nslp_uart->hdmarx, DMA_IT_HT);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == nslp_uart) {
        // Set a breakpoint or log error
    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    }
}

