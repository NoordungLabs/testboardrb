#ifndef NSLP_DMA_H
#define NSLP_DMA_H

#include "stm32g4xx_hal.h"

#define FRAME_START 0x7E
#define MAX_PAYLOAD_SIZE 256
#define TX_QUEUE_SIZE 256

struct Packet {
	uint8_t type;
	uint8_t size;
	uint8_t *payload;
};

void nslp_init(UART_HandleTypeDef *huart);

void send_packet_dma(struct Packet *p);

extern void handle_received_packet(struct Packet *p);

#endif // NSLP_DMA_H
