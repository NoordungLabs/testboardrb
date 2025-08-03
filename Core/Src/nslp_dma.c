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

// RX state machine
static enum {
    RX_WAIT_START,
    RX_HEADER,
    RX_PAYLOAD,
    RX_CRC
} rx_state = RX_WAIT_START;

static uint8_t rx_tmp[4 + MAX_PAYLOAD_SIZE] __attribute__((aligned(4)));
static uint8_t rx_frame_start = 0;
static uint8_t rx_payload_size = 0;

static struct Packet rx_packet;

void nslp_init(UART_HandleTypeDef *huart, CRC_HandleTypeDef *hcrc) {
    nslp_uart = huart;
    nslp_crc = hcrc;

    rx_state = RX_WAIT_START;
    HAL_UART_Receive_DMA(nslp_uart, &rx_frame_start, 1);
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart != nslp_uart) return;

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);  // Debug indicator

    switch (rx_state) {
        case RX_WAIT_START:
            if (rx_frame_start == FRAME_START) {
                rx_state = RX_HEADER;
                if (HAL_UART_Receive_DMA(nslp_uart, rx_tmp, 2) != HAL_OK) {
                    rx_state = RX_WAIT_START;
                    HAL_UART_Receive_DMA(nslp_uart, &rx_frame_start, 1);
                }
            } else {
            	rx_state = RX_WAIT_START;
                HAL_UART_Receive_DMA(nslp_uart, &rx_frame_start, 1);
            }
            break;

        case RX_HEADER:
            rx_packet.type = rx_tmp[0];
            rx_payload_size = rx_tmp[1];

            if (rx_payload_size > MAX_PAYLOAD_SIZE) {
                rx_state = RX_WAIT_START;
                HAL_UART_Receive_DMA(nslp_uart, &rx_frame_start, 1);
                return;
            }

            rx_packet.size = rx_payload_size;
            rx_packet.payload = &rx_tmp[2];

            rx_state = RX_PAYLOAD;
            if (HAL_UART_Receive_DMA(nslp_uart, &rx_tmp[2], rx_payload_size) != HAL_OK) {
                rx_state = RX_WAIT_START;
                HAL_UART_Receive_DMA(nslp_uart, &rx_frame_start, 1);
            }
            break;

        case RX_PAYLOAD:
            rx_state = RX_CRC;
            if (HAL_UART_Receive_DMA(nslp_uart, &rx_tmp[2 + rx_payload_size], 4) != HAL_OK) {
                rx_state = RX_WAIT_START;
                HAL_UART_Receive_DMA(nslp_uart, &rx_frame_start, 1);
            }
            break;

        case RX_CRC: {
            uint32_t received_crc;
            memcpy(&received_crc, &rx_tmp[2 + rx_payload_size], 4);

            __HAL_CRC_DR_RESET(nslp_crc);
            uint32_t computed_crc = HAL_CRC_Calculate(nslp_crc, (uint32_t *)rx_tmp, 2 + rx_payload_size);

            if (received_crc == computed_crc) {
                if (rx_callback) {
                    rx_callback(&rx_packet);
                }
            }

            rx_state = RX_WAIT_START;
            HAL_UART_Receive_DMA(nslp_uart, &rx_frame_start, 1);
            break;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart != nslp_uart) return;

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);  // Error indicator
    rx_state = RX_WAIT_START;
    HAL_UART_Receive_DMA(nslp_uart, &rx_frame_start, 1);
}

// Optional polling fallback
struct Packet* receive_packet(UART_HandleTypeDef *huart, CRC_HandleTypeDef *hcrc) {
    static uint8_t receiveData[HEADER_SIZE + 255 + CHECKSUM_SIZE];
    static struct Packet receiveInstance = { 0x00, 0, NULL };

    uint8_t frameStart = 0;
    if (HAL_UART_Receive(huart, &frameStart, FRAME_START_SIZE, 100) != HAL_OK) {
        return &receiveInstance;
    }
    if (frameStart != FRAME_START) {
        return &receiveInstance;
    }

    if (HAL_UART_Receive(huart, receiveData, HEADER_SIZE, 100) != HAL_OK) {
        return &receiveInstance;
    }

    receiveInstance.type = (char) receiveData[0];
    receiveInstance.size = receiveData[1];

    if (receiveInstance.size == 0) {
        return &receiveInstance;
    }

    if (HAL_UART_Receive(huart, &receiveData[HEADER_SIZE], receiveInstance.size, 100) != HAL_OK) {
        return &receiveInstance;
    }
    receiveInstance.payload = &receiveData[HEADER_SIZE];

    if (HAL_UART_Receive(huart, &receiveData[HEADER_SIZE + receiveInstance.size], CHECKSUM_SIZE, 100) != HAL_OK) {
        return &receiveInstance;
    }

    uint32_t calculatedCrc = HAL_CRC_Calculate(hcrc, (uint32_t *) receiveData, HEADER_SIZE + receiveInstance.size);
    uint32_t receivedCrc = *(uint32_t *) (&receiveData[HEADER_SIZE + receiveInstance.size]);
    if (receivedCrc != calculatedCrc) {
        return &receiveInstance;
    }

    return &receiveInstance;
}
