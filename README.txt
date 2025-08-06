Here I leave with you the documentation about the FADEC software running on stm32g431rbt6

1. Comunication
Comunication is done via nslp v1 protocol designed from a header, payload and crc32 ending.
The header consist of a start frame + packet type + payload size
The payload is data converted to raw bytes.
Current micsrocontroller side works with DMA to work fast in non blocking method.
Start nslp with nslp_init(&youhuart, &yourhcrc)

1.1 Sending
Should you want to send data, define your packets like so:
struct Packet syourPacket = {
	.type = uint8_t;
	.size = uint8_t; //size of your payload
	.payload = &uint8_t;
}:
c
Call nslp_send_data(&syourPacket) to add to to tx queue
The packets should be added with a time delay (non blocking) in regards to your packet size, number of packets and baud rate
otherwise it will ovewrite the tx queue and will absolutely not want to receive data within that span of time.
The current packets are:

struct Packet Pressure = {
		.type = 0xA0,
		.size = sizeof(pressureArray),
		.payload = pressureArray
	};

  struct Packet Temperature = {
		.type = 0xA1,
		.size = sizeof(temperatureArray),
		.payload = temperatureArray
	};
	
	 struct Packet Bal1CurrentPos = {
		.type = 0xB3,
		.size = sizeof(bal1.current_openness),
		.payload = &bal1.current_openness
	};
	
	 struct Packet Bal2CurrentPos = {
		.type = 0xC3,
		.size = sizeof(bal2.current_openness),
		.payload = &bal2.current_openness
	};

  struct Packet SolIsCon = {
		.type = 0xD0,
		.size = sizeof(isCon),
		.payload = &isCon
	};

  struct Packet SolIsFun = {
		.type = 0xD2,
		.size = sizeof(isFun),
		.payload = &isFun
	};
	
1.2 Receiving
Should you want to receive data call nslp_set_rx_callback(fyourCallback) where fyourCallback is your tx handler
The one used in FADEC v6.1 uses the commands sent in two byte packets:
 MACRO:	First array member	Description
 BAL1_CAL	0xB1 	-> Will trigger ball valve calibration
 BAL2_CAL	0xC1	-> Will trigger ball valve calibration
 BAL1_SET	0xB0	-> Will set target openess for ball valve with the second array member being the target openess
 BAL2_SET	0xC0	-> Will set target openess for ball valve with the second array member being the target openess
 ISYS_RST	0xFF	-> Will trigger system reset
 SOLS_SET	0x00	-> Will set solenoids to the value of the second member of array
 ISYS_ARM	0x0F	-> Will arm system to start main loop
 AUTO_INI	0x01	-> Will start auto run sequence
 AUTO_STR 	0x02	-> Will start auto run countdown
 AUTO_EMS	0x03	-> Will close all valves and vent system after 10 seconds
 