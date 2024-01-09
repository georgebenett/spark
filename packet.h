

#ifndef PACKET_H_
#define PACKET_H_

#include <stdint.h>
#include <stdbool.h>

// Settings
#ifndef PACKET_RX_TIMEOUT
#define PACKET_RX_TIMEOUT		100
#endif

#ifndef PACKET_HANDLERS
#define PACKET_HANDLERS			2
#endif

#ifndef PACKET_MAX_PL_LEN
#define PACKET_MAX_PL_LEN		512
#endif

// Functions
void packet_init(void (*s_func)(unsigned char *data, unsigned int len),
		void (*p_func)(unsigned char *data, unsigned int len), int handler_num);
void packet_reset(int handler_num);
void packet_process_byte(uint8_t rx_data, int handler_num);
void packet_timerfunc(void);
void packet_send_packet(unsigned char *data, unsigned int len, int handler_num);

#endif /* PACKET_H_ */
