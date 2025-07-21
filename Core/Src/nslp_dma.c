// nslp_dma.c
#include "nslp_dma.h"
#include <string.h>

NSLP_DMA nslp_dma_ctx;
static uint8_t txDone = 1;

// --- Transmit queue ---
static struct Packet *txQueue[NSLP_TX_QUEUE_SIZE];
static int txHead = 0;
static int txTail = 0;

static int tx_queue_is_empty() {
    return txHead == txTail;
}

static int tx_queue_is_full() {
    return ((txTail + 1) % NSLP_TX_QUEUE_SIZE) == txHead;
}

static void tx_queue_enqueue(struct Packet *p) {
    if (!tx_queue_is_full()) {
        txQueue[txTail] = p;
        txTail = (txTail + 1) % NSLP_TX_QUEUE_SIZE;
    }
}

static struct Packet* tx_queue_dequeue() {
    if (tx_queue_is_empty()) return NULL;
    struct Packet *p = txQueue[txHead];
    txHead = (txHead + 1) % NSLP_TX_QUEUE_SIZE;
    return p;
}

void nslp_dma_init(UART_HandleTypeDef *huart, CRC_HandleTypeDef *hcrc) {
    nslp_dma_ctx.uart = huart;
    nslp_dma_ctx.crc = hcrc;
    txHead = txTail = 0;
    txDone = 1;

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    nslp_start_rx_idle_dma();
}

void send_packet_dma(struct Packet *p) {
    if (!p || p->size > MAX_PACKET_SIZE) return;

    tx_queue_enqueue(p);

    if (txDone) {
        struct Packet *next = tx_queue_dequeue();
        if (next) {
            uint16_t packetSize = HEADER_SIZE + next->size;
            uint16_t totalSize = FRAME_START_SIZE + packetSize + CHECKSUM_SIZE;

            nslp_dma_ctx.txBuffer[0] = FRAME_START;
            nslp_dma_ctx.txBuffer[1] = next->type;
            nslp_dma_ctx.txBuffer[2] = next->size;

            memcpy(&nslp_dma_ctx.txBuffer[3], next->payload, next->size);

            uint32_t crc = HAL_CRC_Calculate(nslp_dma_ctx.crc, (uint32_t *)&nslp_dma_ctx.txBuffer[1], packetSize);
            memcpy(&nslp_dma_ctx.txBuffer[3 + next->size], &crc, CHECKSUM_SIZE);

            txDone = 0;
            HAL_UART_Transmit_DMA(nslp_dma_ctx.uart, nslp_dma_ctx.txBuffer, totalSize);
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart != nslp_dma_ctx.uart) return;

    struct Packet *next = tx_queue_dequeue();
    if (next) {
        uint16_t packetSize = HEADER_SIZE + next->size;
        uint16_t totalSize = FRAME_START_SIZE + packetSize + CHECKSUM_SIZE;

        nslp_dma_ctx.txBuffer[0] = FRAME_START;
        nslp_dma_ctx.txBuffer[1] = next->type;
        nslp_dma_ctx.txBuffer[2] = next->size;

        memcpy(&nslp_dma_ctx.txBuffer[3], next->payload, next->size);

        uint32_t crc = HAL_CRC_Calculate(nslp_dma_ctx.crc, (uint32_t *)&nslp_dma_ctx.txBuffer[1], packetSize);
        memcpy(&nslp_dma_ctx.txBuffer[3 + next->size], &crc, CHECKSUM_SIZE);

        HAL_UART_Transmit_DMA(nslp_dma_ctx.uart, nslp_dma_ctx.txBuffer, totalSize);
    } else {
        txDone = 1;
    }
}

void nslp_start_rx_idle_dma(void) {
    HAL_UART_Receive_DMA(nslp_dma_ctx.uart, nslp_dma_ctx.rxBuffer, NSLP_RX_BUFFER_SIZE);
}

void HAL_UART_IDLECallback(UART_HandleTypeDef *huart) {
    if (huart != nslp_dma_ctx.uart) return;

    __HAL_UART_CLEAR_IDLEFLAG(huart);
    HAL_UART_DMAStop(huart);

    uint16_t rxLen = NSLP_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    if (rxLen < FRAME_START_SIZE + HEADER_SIZE + CHECKSUM_SIZE) {
        nslp_start_rx_idle_dma();
        return;
    }

    if (nslp_dma_ctx.rxBuffer[0] != FRAME_START) {
        nslp_start_rx_idle_dma();
        return;
    }

    uint8_t size = nslp_dma_ctx.rxBuffer[2];
    if ((size + FRAME_START_SIZE + HEADER_SIZE + CHECKSUM_SIZE) > rxLen) {
        nslp_start_rx_idle_dma();
        return;
    }

    memcpy(nslp_dma_ctx.rxData, nslp_dma_ctx.rxBuffer, size + FRAME_START_SIZE + HEADER_SIZE + CHECKSUM_SIZE);

    uint32_t crc_calc = HAL_CRC_Calculate(nslp_dma_ctx.crc, (uint32_t *)&nslp_dma_ctx.rxData[1], HEADER_SIZE + size);
    uint32_t crc_recv = *(uint32_t *)&nslp_dma_ctx.rxData[FRAME_START_SIZE + HEADER_SIZE + size];

    if (crc_calc == crc_recv) {
        nslp_dma_ctx.rxPacket.type = nslp_dma_ctx.rxData[1];
        nslp_dma_ctx.rxPacket.size = nslp_dma_ctx.rxData[2];
        nslp_dma_ctx.rxPacket.payload = &nslp_dma_ctx.rxData[3];
    }

    nslp_start_rx_idle_dma();
}

struct Packet* nslp_get_received_packet(void) {
    return &nslp_dma_ctx.rxPacket;
}
