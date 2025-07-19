#ifndef NSLP_DMA_H
#define NSLP_DMA_H

#include "stm32g4xx_hal.h"

#define FRAME_START 0x7E
#define MAX_PAYLOAD_SIZE 256
#define TX_QUEUE_SIZE 4

struct Packet {
	uint8_t type;
	uint8_t size;
	uint8_t *payload;
};

// Call once at init
void nslp_init(UART_HandleTypeDef *huart);

// Send a packet (non-blocking, queued)
void send_packet_dma(struct Packet *p);

// Optional: override this in your app
extern void handle_received_packet(struct Packet *p);

#endif // NSLP_DMA_H
