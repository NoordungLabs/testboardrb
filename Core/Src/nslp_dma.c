#include "nslp_dma.h"
#include <string.h>

static UART_HandleTypeDef *nslp_uart;

// === CRC ===
static uint8_t calculate_crc(uint8_t *data, uint16_t length) {
	uint8_t crc = 0x00;
	for (uint16_t i = 0; i < length; ++i) {
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; ++j)
			crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
	}
	return crc;
}

// === RX ===
static uint8_t rxFrameStart;
static uint8_t rxHeader[2];
static uint8_t rxPayload[MAX_PAYLOAD_SIZE];
static uint8_t rxCRC;

typedef enum { RX_WAIT_START, RX_WAIT_HEADER, RX_WAIT_PAYLOAD, RX_WAIT_CRC } RX_State;
static RX_State rx_state = RX_WAIT_START;

static struct Packet currentRxPacket;
struct Packet safeCopy;
uint8_t safePayload[MAX_PAYLOAD_SIZE];

// === TX ===
static struct Packet *txQueue[TX_QUEUE_SIZE];
static uint8_t txHead = 0, txTail = 0;
static uint8_t txInProgress = 0;
static uint8_t txBuffer[1 + 2 + MAX_PAYLOAD_SIZE + 1]; // FRAME + HEADER + PAYLOAD + CRC

// Callback pointer
__weak void handle_received_packet(struct Packet *p) {

}

void nslp_init(UART_HandleTypeDef *huart) {
	nslp_uart = huart;
	txHead = txTail = txInProgress = 0;
	rx_state = RX_WAIT_START;
	HAL_UART_Receive_DMA(nslp_uart, &rxFrameStart, 1);
}

// RX Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart != nslp_uart) return;

	switch (rx_state) {
		case RX_WAIT_START:
			if (rxFrameStart == FRAME_START) {
				rx_state = RX_WAIT_HEADER;
				HAL_UART_Receive_DMA(nslp_uart, rxHeader, 2);
			} else {
				HAL_UART_Receive_DMA(nslp_uart, &rxFrameStart, 1);
			}
			break;

		case RX_WAIT_HEADER:
			currentRxPacket.type = rxHeader[0];
			currentRxPacket.size = rxHeader[1];
			currentRxPacket.payload = rxPayload;

			if (currentRxPacket.size > 0 && currentRxPacket.size <= MAX_PAYLOAD_SIZE) {
				rx_state = RX_WAIT_PAYLOAD;
				HAL_UART_Receive_DMA(nslp_uart, rxPayload, currentRxPacket.size);
			} else if (currentRxPacket.size == 0) {
				rx_state = RX_WAIT_CRC;
				HAL_UART_Receive_DMA(nslp_uart, &rxCRC, 1);
			} else {
				rx_state = RX_WAIT_START;
				HAL_UART_Receive_DMA(nslp_uart, &rxFrameStart, 1);
			}
			break;

		case RX_WAIT_PAYLOAD:
			rx_state = RX_WAIT_CRC;
			HAL_UART_Receive_DMA(nslp_uart, &rxCRC, 1);
			break;

		case RX_WAIT_CRC: {
			uint8_t temp[2 + MAX_PAYLOAD_SIZE];
			temp[0] = currentRxPacket.type;
			temp[1] = currentRxPacket.size;
			memcpy(&temp[2], currentRxPacket.payload, currentRxPacket.size);
			uint8_t calc_crc = calculate_crc(temp, 2 + currentRxPacket.size);

			if (calc_crc == rxCRC) {
				// Make safe copy for app use
				safeCopy.type = currentRxPacket.type;
				safeCopy.size = currentRxPacket.size;
				memcpy(safePayload, currentRxPacket.payload, currentRxPacket.size);
				safeCopy.payload = safePayload;

				handle_received_packet(&safeCopy); // ISR-safe if quick
			}

			rx_state = RX_WAIT_START;
			HAL_UART_Receive_DMA(nslp_uart, &rxFrameStart, 1);
			break;
		}
	}
}

// Prepares next packet
static void start_next_transmit() {
	if (txHead == txTail) {
		txInProgress = 0;
		return;
	}

	struct Packet *p = txQueue[txTail];
	txTail = (txTail + 1) % TX_QUEUE_SIZE;

	uint16_t len = 0;
	txBuffer[len++] = FRAME_START;
	txBuffer[len++] = p->type;
	txBuffer[len++] = p->size;
	memcpy(&txBuffer[len], p->payload, p->size);
	len += p->size;

	uint8_t crc = calculate_crc(&txBuffer[1], 2 + p->size);
	txBuffer[len++] = crc;

	txInProgress = 1;
	HAL_UART_Transmit_DMA(nslp_uart, txBuffer, len);
}

//send selected packet
void send_packet_dma(struct Packet *p) {
	if (!p || p->size > MAX_PAYLOAD_SIZE) return;

	uint8_t nextHead = (txHead + 1) % TX_QUEUE_SIZE;
	if (nextHead == txTail) {
		// Queue full — drop
		return;
	}

	txQueue[txHead] = p;
	txHead = nextHead;

	if (!txInProgress) {
		start_next_transmit();
	}
}

// TX Callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == nslp_uart) {
		start_next_transmit();
	}
}
