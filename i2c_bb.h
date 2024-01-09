

#ifndef I2C_BB_H_
#define I2C_BB_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
	int sda_pin;
	int scl_pin;
	bool has_started;
	bool has_error;
} i2c_bb_state;

void i2c_bb_init(i2c_bb_state *s);
void i2c_bb_restore_bus(i2c_bb_state *s);
bool i2c_bb_tx_rx(i2c_bb_state *s, uint16_t addr, uint8_t *txbuf, size_t txbytes, uint8_t *rxbuf, size_t rxbytes);

#endif /* I2C_BB_H_ */
