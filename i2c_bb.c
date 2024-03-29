

#include "i2c_bb.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

// This is based on https://en.wikipedia.org/wiki/I%C2%B2C

// Macros
#define SDA_LOW()				nrf_gpio_cfg_output(s->sda_pin);
#define SDA_HIGH()				nrf_gpio_cfg_input(s->sda_pin, NRF_GPIO_PIN_PULLUP);
#define SCL_LOW()				nrf_gpio_cfg_output(s->scl_pin);
#define SCL_HIGH()				nrf_gpio_cfg_input(s->scl_pin, NRF_GPIO_PIN_PULLUP);
#define READ_SDA()				nrf_gpio_pin_read(s->sda_pin)
#define READ_SCL()				nrf_gpio_pin_read(s->scl_pin)

// Private functions
static void i2c_start_cond(i2c_bb_state *s);
static void i2c_stop_cond(i2c_bb_state *s);
static void i2c_write_bit(i2c_bb_state *s, bool bit);
static bool i2c_read_bit(i2c_bb_state *s);
static bool i2c_write_byte(i2c_bb_state *s, bool send_start, bool send_stop, unsigned char byte);
static unsigned char i2c_read_byte(i2c_bb_state *s, bool nack, bool send_stop);
static bool clock_stretch_timeout(i2c_bb_state *s);
static void i2c_delay(void);

void i2c_bb_init(i2c_bb_state *s) {
	nrf_gpio_pin_clear(s->scl_pin);
	nrf_gpio_pin_clear(s->sda_pin);
	s->has_started = false;
	s->has_error = false;
}

void i2c_bb_restore_bus(i2c_bb_state *s) {
	SCL_HIGH();
	SDA_HIGH();

	nrf_delay_us(10);

	for(int i = 0;i < 16;i++) {
		SCL_LOW();
		nrf_delay_us(10);
		SCL_HIGH();
		nrf_delay_us(10);
	}

	s->has_started = false;

	i2c_start_cond(s);
	i2c_stop_cond(s);

	s->has_error = false;
}

bool i2c_bb_tx_rx(i2c_bb_state *s, uint16_t addr, uint8_t *txbuf, size_t txbytes, uint8_t *rxbuf, size_t rxbytes) {
	i2c_write_byte(s, true, false, addr << 1);

	for (unsigned int i = 0;i < txbytes;i++) {
		i2c_write_byte(s, false, false, txbuf[i]);
	}

	if (rxbytes > 0) {
		i2c_write_byte(s, true, false, addr << 1 | 1);

		for (unsigned int i = 0;i < rxbytes;i++) {
			rxbuf[i] = i2c_read_byte(s, i == (rxbytes - 1), false);
		}
	}

	i2c_stop_cond(s);

	return !s->has_error;
}

static void i2c_start_cond(i2c_bb_state *s) {
	if (s->has_started) {
		// if started, do a restart condition
		SDA_HIGH();
		i2c_delay();
		SCL_HIGH();

		if (!clock_stretch_timeout(s)) {
			return;
		}

		// Repeated start setup time, minimum 4.7us
		i2c_delay();
	}

	if (READ_SDA() == 0) {
//		arbitration_lost();
		s->has_error = true;
	}

	// SCL is high, set SDA from 1 to 0.
	SDA_LOW();
	i2c_delay();
	SCL_LOW();
	s->has_started = true;
}

static void i2c_stop_cond(i2c_bb_state *s) {
	SDA_LOW();
	i2c_delay();

	SCL_HIGH();

	if (!clock_stretch_timeout(s)) {
		return;
	}

	// Stop bit setup time, minimum 4us
	i2c_delay();

	// SCL is high, set SDA from 0 to 1
	SDA_HIGH();

	i2c_delay();

	if (READ_SDA() == 0) {
//		arbitration_lost();
		s->has_error = true;
	}

	s->has_started = false;
}

static void i2c_write_bit(i2c_bb_state *s, bool bit) {
	if (bit) {
		SDA_HIGH();
	} else {
		SDA_LOW();
	}

	// SDA change propagation delay
	i2c_delay();

	// Set SCL high to indicate a new valid SDA value is available
	SCL_HIGH();

	// Wait for SDA value to be read by slave, minimum of 4us for standard mode
	i2c_delay();

	if (!clock_stretch_timeout(s)) {
		return;
	}

	// SCL is high, now data is valid

	// If SDA is high, check that nobody else is driving SDA
	if (bit && (READ_SDA() == 0)) {
//		arbitration_lost();
		s->has_error = true;
	}

	// Clear the SCL to low in preparation for next change
	SCL_LOW();
}

static bool i2c_read_bit(i2c_bb_state *s) {
	bool bit;

	// Let the slave drive data
	SDA_HIGH();

	// Wait for SDA value to be written by slave, minimum of 4us for standard mode
	i2c_delay();

	// Set SCL high to indicate a new valid SDA value is available
	SCL_HIGH();

	if (!clock_stretch_timeout(s)) {
		return false;
	}

	// Wait for SDA value to be written by slave, minimum of 4us for standard mode
	i2c_delay();

	// SCL is high, read out bit
	bit = READ_SDA();

	// Set SCL low in preparation for next operation
	SCL_LOW();

	return bit;
}

static bool i2c_write_byte(i2c_bb_state *s, bool send_start, bool send_stop, unsigned char byte) {
	unsigned bit;
	bool nack;

	if (send_start) {
		i2c_start_cond(s);
	}

	for (bit = 0;bit < 8;bit++) {
		i2c_write_bit(s, (byte & 0x80) != 0);
		byte <<= 1;
	}

	nack = i2c_read_bit(s);

	if (send_stop) {
		i2c_stop_cond(s);
	}

	return nack;
}

static unsigned char i2c_read_byte(i2c_bb_state *s, bool nack, bool send_stop) {
	unsigned char byte = 0;
	unsigned char bit;

	for (bit = 0;bit < 8;bit++) {
		byte = (byte << 1) | i2c_read_bit(s);
	}

	i2c_write_bit(s, nack);

	if (send_stop) {
		i2c_stop_cond(s);
	}

	return byte;
}

static bool clock_stretch_timeout(i2c_bb_state *s) {
	volatile uint32_t cnt = 0;
	while(READ_SCL() == 0) {
		cnt++;

		if (cnt >= 6000000) {
			return false;
		}
	}

	return true;
}

static void i2c_delay(void) {
	nrf_delay_us(1);
}
