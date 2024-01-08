#pragma once

#include <nrf_pwm.h>
#include <stdint.h>

#include "error.h"

#define EPWM_NO_INIT				(EPWM_BASE + 0x00)
#define EPWM_INVALID_ARG			(EPWM_BASE + 0x01)
#define EPWM_MAX_CLIENTS			(EPWM_BASE + 0x02)
#define EPWM_MAX_CHANNELS			(EPWM_BASE + 0x03)
#define EPWM_MUTEX					(EPWM_BASE + 0x04)
#define EPWM_INVALID_PWM			(EPWM_BASE + 0x05)
#define EPWM_NO_FREE_PWM			(EPWM_BASE + 0x06)

typedef void (*pwm_seq_end_callback)(void);

struct pwm_board {
	uint8_t max_nbr_clients;
};

err_code pwm_channels_reg(NRF_PWM_Type **pp_reg,
					const uint8_t *p_pins, const uint8_t nbr_pins);
void pwm_channels_config(NRF_PWM_Type *p_reg,
					nrf_pwm_clk_t prescaler, uint16_t countertop,
					nrf_pwm_dec_load_t dec_load);
err_code pwm_channels_start_seq(NRF_PWM_Type *p_reg,
						const nrf_pwm_sequence_t *p_sequence);
err_code pwm_set_seq_end_handler(NRF_PWM_Type *p_reg, pwm_seq_end_callback cb);
err_code pwm_stop(NRF_PWM_Type *p_reg);
err_code pwm_init(const struct pwm_board * const p_board);
