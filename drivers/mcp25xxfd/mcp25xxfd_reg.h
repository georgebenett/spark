#pragma once

#include "drivers/mcp25xxfd/mcp25xxfd_def.h"

/* SPI Instruction Set */
#define MCP25_INST_RESET		0x00
#define MCP25_INST_READ			0x03
#define MCP25_INST_READ_CRC		0x0B
#define MCP25_INST_WRITE		0x02
#define MCP25_INST_WRITE_CRC	0x0A
#define MCP25_INST_WRITE_SAFE	0x0C

/* CAN FD Controller */
#define MCP25_REG_CiCON			0x000
#define MCP25_REG_CiNBTCFG		0x004
#define MCP25_REG_CiDBTCFG		0x008
#define MCP25_REG_CiTDC			0x00C

#define MCP25_REG_CiTBC			0x010
#define MCP25_REG_CiTSCON		0x014
#define MCP25_REG_CiVEC			0x018
#define MCP25_REG_CiINT			0x01C
#define MCP25_REG_CiINTFLAG		MCP25_REG_CiINT
#define MCP25_REG_CiINTENABLE	(MCP25_REG_CiINT + 2)

#define MCP25_REG_CiRXIF		0x020
#define MCP25_REG_CiTXIF		0x024
#define MCP25_REG_CiRXOVIF		0x028
#define MCP25_REG_CiTXATIF		0x02C

#define MCP25_REG_CiTXREQ		0x030
#define MCP25_REG_CiTREC		0x034
#define MCP25_REG_CiBDIAG0		0x038
#define MCP25_REG_CiBDIAG1		0x03C

#define MCP25_REG_CiTEFCON		0x040
#define MCP25_REG_CiTEFSTA		0x044
#define MCP25_REG_CiTEFUA		0x048
#define MCP25_REG_CiFIFOBA		0x04C

#define MCP25_REG_CiFIFOCON		0x050
#define MCP25_REG_CiFIFOSTA		0x054
#define MCP25_REG_CiFIFOUA		0x058
#define CiFIFO_OFFSET			(3 * 4)

#define MCP25_REG_CiTXQCON		0x050
#define MCP25_REG_CiTXQSTA		0x054
#define MCP25_REG_CiTXQUA		0x058

/* The filters start right after the FIFO control/status registers */
#define MCP25_REG_CiFLTCON		(MCP25_REG_CiFIFOCON + \
									(CiFIFO_OFFSET * NUM_CAN_FIFO_CHANNELS))
#define MCP25_REG_CiFLTOBJ		(MCP25_REG_CiFLTCON + NUM_CAN_FIFO_CHANNELS)
#define MCP25_REG_CiMASK		(MCP25_REG_CiFLTOBJ + 4)

#define CiFILTER_OFFSET			(2 * 4)

/* MCP25xxFD Specific */
#define MCP25_REG_OSC			0xE00
#define MCP25_REG_IOCON			0xE04
#define MCP25_REG_CRC			0xE08
#define MCP25_REG_ECCCON		0xE0C
#define MCP25_REG_ECCSTA		0xE10
#ifndef MCP2517FD
#define MCP25_REG_DEVID			0xE14
#endif

/* RAM addresses */
#if defined(MCP2517FD) || defined(MCP2518FD)
#define MCP25_RAM_SIZE			2048
#endif

#define MCP25_RAMADDR_START		0x400
#define MCP25_RAMADDR_END		(MCP25_RAMADDR_START + MCP25_RAM_SIZE)

union mcp25_word {
	uint8_t byte[4];
	uint32_t word;
};

/* CAN Control Register */
union mcp25_reg_cicon {
	struct {
		uint32_t dnet_filter_cnt : 5;
		uint32_t iso_crc_enable : 1;
		uint32_t prot_except_evt_disable : 1;
		uint32_t dummy_0 : 1;
		uint32_t wakeup_filter_enable : 1;
		uint32_t wakeup_filter_time : 2;
		uint32_t dummy_1 : 1;
		uint32_t bit_rate_switch_disable : 1;
		uint32_t dummy_2 : 3;
		uint32_t restrict_re_tx_attempts : 1;
		uint32_t esi_in_gateway_mode : 1;
		uint32_t sys_err_to_listen_only : 1;
		uint32_t store_in_tef : 1;
		uint32_t txq_enable : 1;
		uint32_t op_mode : 3;
		uint32_t req_op_mode : 3;
		uint32_t abort_all_tx : 1;
		uint32_t tx_bw_sharing : 4;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* Nominal Bit Time Configuration Register */
union mcp25_reg_cinbtcfg {
	struct {
		uint32_t sjw : 7;
		uint32_t dummy_0 : 1;
		uint32_t tseg2 : 7;
		uint32_t dummy_1 : 1;
		uint32_t tseg1 : 8;
		uint32_t brp : 8;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* Data Bit Time Configuration Register */
union mcp25_reg_cidbtcfg {
	struct {
		uint32_t sjw : 4;
		uint32_t dummy_0 : 4;
		uint32_t tseg2 : 4;
		uint32_t dummy_1 : 4;
		uint32_t tseg1 : 5;
		uint32_t dummy_2 : 3;
		uint32_t brp : 8;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* Transmitter Delay Compensation Register */
union mcp25_reg_citdc {
	struct {
		uint32_t tdc_val : 6;
		uint32_t dummy_0 : 2;
		uint32_t tdc_offset : 7;
		uint32_t dummy_1 : 1;
		uint32_t tdc_mode : 2;
		uint32_t dummy_2 : 6;
		uint32_t sid11_enable : 1;
		uint32_t edge_filter_enable : 1;
		uint32_t dummy_3 : 6;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* Time Stamp Configuration Register */
union mcp25_reg_citscon {
	struct {
		uint32_t tbc_prescaler : 10;
		uint32_t dummy_0 : 6;
		uint32_t tbc_enable : 1;
		uint32_t ts_eof : 1;
		uint32_t dummy_1 : 14;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* Interrupt Vector Register */
union mcp25_reg_civec {
	struct {
		uint32_t icode : 7;
		uint32_t dummy_0 : 1;
		uint32_t filter_hit : 5;
		uint32_t dummy_1 : 3;
		uint32_t txcode : 7;
		uint32_t dummy_2 : 1;
		uint32_t rxcode : 7;
		uint32_t dummy_3 : 1;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* Interrupt Flags */
struct mcp25_can_int_flag {
	uint32_t txif : 1;
	uint32_t rxif : 1;
	uint32_t tbcif : 1;
	uint32_t modif : 1;
	uint32_t tefif : 1;
	uint32_t dummy_0 : 3;

	uint32_t eccif : 1;
	uint32_t spicrcif : 1;
	uint32_t txatif : 1;
	uint32_t rxovif : 1;
	uint32_t serrif : 1;
	uint32_t cerrif : 1;
	uint32_t wakif : 1;
	uint32_t ivmif : 1;
};

/* Interrupt Enables */
struct mcp25_can_int_enable {
	uint32_t txie : 1;
	uint32_t rxie : 1;
	uint32_t tbcie : 1;
	uint32_t modie : 1;
	uint32_t tefie : 1;
	uint32_t dummy_1 : 3;

	uint32_t eccie : 1;
	uint32_t spicrcie : 1;
	uint32_t txatie : 1;
	uint32_t rxovie : 1;
	uint32_t serrie : 1;
	uint32_t cerrie : 1;
	uint32_t wakie : 1;
	uint32_t ivmie : 1;
};

/* Interrupt Register */
union mcp25_reg_ciint {
	struct {
		struct mcp25_can_int_flag flag;
		struct mcp25_can_int_enable enable;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* Interrupt Flag Register */
union mcp25_reg_ciint_flag {
	struct mcp25_can_int_flag flag;
	uint16_t word;
	uint8_t byte[2];
};

/* Interrupt enable Register */
union mcp25_reg_ciint_enable {
	struct mcp25_can_int_enable enable;
	uint16_t word;
	uint8_t byte[2];
};

/* Transmit/Receive Error Count Register */
union mcp25_reg_citrec {
	struct {
		uint32_t rx_err_cnt : 8;
		uint32_t tx_err_cnt : 8;
		uint32_t err_state_warn : 1;
		uint32_t rx_err_state_warn : 1;
		uint32_t tx_err_state_warn : 1;
		uint32_t rx_err_state_passive : 1;
		uint32_t tx_err_state_passive : 1;
		uint32_t tx_err_state_bus_off : 1;
		uint32_t dummy_0 : 10;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* Diagnostic Register 0*/
union mcp25_reg_cibdiag0 {
	struct {
		uint32_t nrx_err_cnt : 8;
		uint32_t ntx_err_cnt : 8;
		uint32_t drx_err_cnt : 8;
		uint32_t dtx_err_cnt : 8;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* Diagnostic Register 1*/
union mcp25_reg_cibdiag1 {
	struct {
		uint32_t err_free_msg_cnt : 16;
		uint32_t nbit0_err : 1;
		uint32_t nbit1_err : 1;
		uint32_t nack_err : 1;
		uint32_t nform_err : 1;
		uint32_t NStuffError : 1;
		uint32_t ncrc_err : 1;
		uint32_t dummy_0 : 1;
		uint32_t txbo_err : 1;
		uint32_t dbit0_err : 1;
		uint32_t dbit1_err : 1;
		uint32_t dack_err : 1;
		uint32_t dform_err : 1;
		uint32_t dstuff_err : 1;
		uint32_t dcrc_err : 1;
		uint32_t esi : 1;
		uint32_t dummy_1 : 1;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* Transmit Event FIFO Control Register */
union mcp25_reg_citefcon {
	struct {
		uint32_t tefneie : 1;
		uint32_t tefhfie : 1;
		uint32_t teffullie : 1;
		uint32_t tefovie : 1;
		uint32_t dummy_0 : 1;
		uint32_t ts_enable : 1;
		uint32_t dummy_1 : 2;
		uint32_t uinc : 1;
		uint32_t dummy_2 : 1;
		uint32_t freset : 1;
		uint32_t dummy_3 : 13;
		uint32_t fifo_size : 5;
		uint32_t dummy4 : 3;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* Transmit Event FIFO Status Register */
union mcp25_reg_citefsta {
	struct {
		uint32_t tef_not_empty_if : 1;
		uint32_t tef_half_full_if : 1;
		uint32_t tef_full_if : 1;
		uint32_t tef_ov_if : 1;
		uint32_t dummy_0 : 28;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* Transmit Queue Control Register */
union mcp25_reg_citxqcon {
	struct {
		uint32_t tx_not_full_ie : 1;
		uint32_t dummy_0 : 1;
		uint32_t tx_empty_ie : 1;
		uint32_t dummy_1 : 1;
		uint32_t tx_attempt_ie : 1;
		uint32_t dummy_2 : 2;
		uint32_t tx_enable : 1;
		uint32_t uinc : 1;
		uint32_t tx_req : 1;
		uint32_t freset : 1;
		uint32_t dummy_3 : 5;
		uint32_t tx_prio : 5;
		uint32_t tx_attempts : 2;
		uint32_t dummy4 : 1;
		uint32_t fifo_size : 5;
		uint32_t payload_size : 3;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* Transmit Queue Status Register */
union mcp25_reg_citxqsta {
	struct {
		uint32_t tx_not_full_if : 1;
		uint32_t dummy_0 : 1;
		uint32_t tx_empty_if : 1;
		uint32_t dummy_1 : 1;
		uint32_t tx_attempt_if : 1;
		uint32_t tx_err : 1;
		uint32_t tx_lost_arb : 1;
		uint32_t tx_aborted : 1;
		uint32_t fifo_idx : 5;
		uint32_t dummy_2 : 19;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* FIFO Control Register */
union mcp25_reg_cififocon {
	struct {
		uint32_t RxNotEmptyIE : 1;
		uint32_t RxHalfFullIE : 1;
		uint32_t RxFullIE : 1;
		uint32_t RxOverFlowIE : 1;
		uint32_t dummy_0 : 1;
		uint32_t rx_ts_enable : 1;
		uint32_t dummy_1 : 1;
		uint32_t tx_enable : 1;
		uint32_t uinc : 1;
		uint32_t dummy_2 : 1;
		uint32_t freset : 1;
		uint32_t dummy_3 : 13;
		uint32_t fifo_size : 5;
		uint32_t payload_size : 3;
	} rx_bf;

	struct {
		uint32_t tx_not_full_ie : 1;
		uint32_t TxHalfFullIE : 1;
		uint32_t tx_empty_ie : 1;
		uint32_t dummy_0 : 1;
		uint32_t tx_attempt_ie : 1;
		uint32_t dummy_1 : 1;
		uint32_t rtr_enable : 1;
		uint32_t tx_enable : 1;
		uint32_t uinc : 1;
		uint32_t tx_req : 1;
		uint32_t freset : 1;
		uint32_t dummy_2 : 5;
		uint32_t tx_prio : 5;
		uint32_t tx_attempts : 2;
		uint32_t dummy_3 : 1;
		uint32_t fifo_size : 5;
		uint32_t payload_size : 3;
	} tx_bf;

	uint32_t word;
	uint8_t byte[4];
};

/* FIFO Status Register */
union mcp25_reg_cififosta {
	struct {
		uint32_t rx_not_empty_if : 1;
		uint32_t rx_half_full_if : 1;
		uint32_t rx_full_if : 1;
		uint32_t rx_ov_if : 1;
		uint32_t dummy_0 : 4;
		uint32_t fifo_idx : 5;
		uint32_t dummy_1 : 19;
	} rx_bf;

	struct {
		uint32_t tx_not_full_if : 1;
		uint32_t tx_half_full_if : 1;
		uint32_t tx_empty_if : 1;
		uint32_t dummy_0 : 1;
		uint32_t tx_attempt_if : 1;
		uint32_t tx_err : 1;
		uint32_t tx_lost_arb : 1;
		uint32_t tx_aborted : 1;
		uint32_t fifo_idx : 5;
		uint32_t dummy_1 : 19;
	} tx_bf;

	uint32_t word;
	uint8_t byte[4];
};

/* FIFO User Address Register */
union mcp25_reg_cififoua {
	struct {
		uint32_t usr_addr : 12;
		uint32_t dummy_0 : 20;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* Filter Control Register */
union mcp25_reg_cifltcon_byte {
	struct {
		uint32_t buf_ptr : 5;
		uint32_t dummy_0 : 2;
		uint32_t enable : 1;
	} bf;

	uint8_t byte;
};

/* Filter Object Register */
union mcp25_reg_cifltobj {
	struct mcp25_can_filterobj_id bf;
	uint32_t word;
	uint8_t byte[4];
};

/* Mask Object Register */
union mcp25_reg_cimask {
	struct mcp25_can_maskobj_id bf;
	uint32_t word;
	uint8_t byte[4];
};


/* Oscillator Control Register */
union mcp25_reg_osc {
	struct {
		uint32_t pll_enable : 1;
		uint32_t dummy_0 : 1;
		uint32_t osc_disable : 1;
#ifdef MCP2517FD
		uint32_t dummy_1 : 1;
#else
		uint32_t low_pwr_mode_enable : 1;
#endif
		uint32_t sclkdiv : 1;
		uint32_t clkodiv : 2;
		uint32_t dummy_2 : 1;
		uint32_t pll_rdy : 1;
		uint32_t dummy_3 : 1;
		uint32_t osc_rdy : 1;
		uint32_t dummy4 : 1;
		uint32_t sclk_rdy : 1;
		uint32_t dummy5 : 19;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* I/O Control Register */
union mcp25_reg_iocon {
	struct {
		uint32_t tris0 : 1;
		uint32_t tris1 : 1;
		uint32_t dummy_0 : 2;
		uint32_t clr_autosleep_on_match : 1;
		uint32_t autosleep_enable : 1;
		uint32_t xcr_stby_enable : 1;
		uint32_t dummy_1 : 1;
		uint32_t lat0 : 1;
		uint32_t lat1 : 1;
		uint32_t dummy_2 : 5;
		uint32_t hvdetsel : 1;
		uint32_t GPIO0 : 1;
		uint32_t GPIO1 : 1;
		uint32_t dummy_3 : 6;
		uint32_t pin_mode0 : 1;
		uint32_t pin_mode1 : 1;
		uint32_t dummy4 : 2;
		uint32_t txcan_open_drain : 1;
		uint32_t sof_output_enable : 1;
		uint32_t int_pin_open_drain : 1;
		uint32_t dummy5 : 1;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* crc Regsiter */
union mcp25_reg_crc {
	struct {
		uint32_t crc : 16;
		uint32_t crcerrif : 1;
		uint32_t ferrif : 1;
		uint32_t dummy_0 : 6;
		uint32_t crcerrie : 1;
		uint32_t ferrie : 1;
		uint32_t dummy_1 : 6;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* ECC Control Register */
union mcp25_reg_ecccon {
	struct {
		uint32_t ecc_en : 1;
		uint32_t secie : 1;
		uint32_t dedie : 1;
		uint32_t dummy_0 : 5;
		uint32_t parity : 7;
		uint32_t dummy_1 : 17;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* ECC Status Register */
union mcp25_reg_eccsta {
	struct {
		uint32_t dummy_0 : 1;
		uint32_t secif : 1;
		uint32_t dedif : 1;
		uint32_t dummy_1 : 13;
		uint32_t err_addr : 12;
		uint32_t dummy_2 : 4;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};

/* DEVID Register */
union mcp25_reg_devid {
	struct {
		uint32_t rev : 4;
		uint32_t dev : 4;
		uint32_t dummy : 24;
	} bf;

	uint32_t word;
	uint8_t byte[4];
};
