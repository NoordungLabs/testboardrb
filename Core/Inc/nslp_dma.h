// nslp_dma.h
#ifndef NSLP_DMA_H
#define NSLP_DMA_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

#define FRAME_START 0x7E
#define FRAME_START_SIZE 1
#define HEADER_SIZE 2
#define CHECKSUM_SIZE 4
#define MAX_PACKET_SIZE 255
#define NSLP_TX_QUEUE_SIZE 8
#define NSLP_RX_BUFFER_SIZE 300

struct Packet {
    uint8_t type;
    uint8_t size;
    uint8_t *payload;
};

typedef struct {
    UART_HandleTypeDef *uart;
    CRC_HandleTypeDef *crc;
    uint8_t txBuffer[FRAME_START_SIZE + HEADER_SIZE + MAX_PACKET_SIZE + CHECKSUM_SIZE];
    uint8_t rxBuffer[NSLP_RX_BUFFER_SIZE];
    uint8_t rxData[FRAME_START_SIZE + HEADER_SIZE + MAX_PACKET_SIZE + CHECKSUM_SIZE];
    struct Packet rxPacket;
} NSLP_DMA;

extern NSLP_DMA nslp_dma_ctx;

void nslp_dma_init(UART_HandleTypeDef *huart, CRC_HandleTypeDef *hcrc);
void send_packet_dma(struct Packet *p);
void nslp_start_rx_idle_dma(void);
struct Packet* nslp_get_received_packet(void);

#endif // NSLP_DMA_H
