#pragma once

#include <stdint.h>

typedef uint32_t err_code;

#define ERR_CHECK(r) \
	do { \
		if (r != ERROR_OK) \
			return r; \
	} while (0)

#define ERR_CHECK_GOTO(r, label) \
	do { \
		if (r != ERROR_OK) \
			goto label; \
	} while (0)

#define ERROR_OK				(0)
#define ERROR_BASE				(0x80000000)
#define EBT_BASE				(ERROR_BASE + (0x01 << 16))
#define EUART_BASE				(ERROR_BASE + (0x02 << 16))
#define EBLE_NUS_BASE			(ERROR_BASE + (0x03 << 16))
#define EBOARD_BASE				(ERROR_BASE + (0x04 << 16))
#define EUICR_BASE				(ERROR_BASE + (0x05 << 16))
#define EFLASH_BASE				(ERROR_BASE + (0x06 << 16))
#define EGPIO_IRQ_BASE			(ERROR_BASE + (0x07 << 16))
#define ELOG_BASE				(ERROR_BASE + (0x08 << 16))
#define EWDT_BASE				(ERROR_BASE + (0x09 << 16))
#define EBTADV_BASE				(ERROR_BASE + (0x0A << 16))
#define EBTCONNINT_BASE			(ERROR_BASE + (0x0B << 16))
#define EBTSVC_PERI_BASE		(ERROR_BASE + (0x0C << 16))
#define EUARTE_BASE				(ERROR_BASE + (0x0D << 16))
#define EPOWER_BASE				(ERROR_BASE + (0x0E << 16))
#define EEVENTPUMP_BASE			(ERROR_BASE + (0x0F << 16))
#define EWALLTIME_BASE			(ERROR_BASE + (0x10 << 16))
#define ETASKTRACKER_BASE		(ERROR_BASE + (0x11 << 16))
#define EBTSVCFOTA_BASE			(ERROR_BASE + (0x12 << 16))
#define EDMS_BASE				(ERROR_BASE + (0x13 << 16))
#define ECMDOPS_BASE			(ERROR_BASE + (0x14 << 16))
#define EBUZZ_BASE				(ERROR_BASE + (0x15 << 16))
#define EPWM_LEDS_BASE			(ERROR_BASE + (0x16 << 16))
#define ETWIM_BASE				(ERROR_BASE + (0x17 << 16))
#define EA2235_BASE				(ERROR_BASE + (0x18 << 16))
#define EHDC2010_BASE			(ERROR_BASE + (0x19 << 16))
#define LSM6DS3_BASE			(ERROR_BASE + (0x1A << 16))
#define EUTIL_BASE				(ERROR_BASE + (0x1B << 16))
#define EGZLL_BASE				(ERROR_BASE + (0x1C << 16))
#define ECONN_MGR_BASE			(ERROR_BASE + (0x1D << 16))
#define EPOS_BASE				(ERROR_BASE + (0x1E << 16))
#define ETHRTL_BASE				(ERROR_BASE + (0x1F << 16))
#define EMSGPACK_BASE			(ERROR_BASE + (0x20 << 16))
#define EW25N0X_BASE			(ERROR_BASE + (0x21 << 16))
#define EPWM_BASE				(ERROR_BASE + (0x22 << 16))
#define ELED_MGR_BASE			(ERROR_BASE + (0x23 << 16))
#define EUSB_BASE				(ERROR_BASE + (0x24 << 16))
#define ESESSION_WRTR_BASE		(ERROR_BASE + (0x25 << 16))
#define ESIXP_BMS_BASE			(ERROR_BASE + (0x26 << 16))
#define EGPS_UBLOX_BASE			(ERROR_BASE + (0x27 << 16))
#define EMCP25XXFD_BASE			(ERROR_BASE + (0x28 << 16))
#define ETWIM_SPIM_IRQ_BASE		(ERROR_BASE + (0x29 << 16))
#define EVUTIL_BASE				(ERROR_BASE + (0x2A << 16))
#define EVESC_BASE				(ERROR_BASE + (0x2B << 16))
#define EVBMS_BASE				(ERROR_BASE + (0x2C << 16))
#define ECAN_BASE				(ERROR_BASE + (0x2D << 16))
#define EESPWIFI_BASE			(ERROR_BASE + (0x2E << 16))
#define EMAX6964_BASE			(ERROR_BASE + (0x2F << 16))
#define ELEDS_BASE				(ERROR_BASE + (0x30 << 16))
#define EFOTA_BASE				(ERROR_BASE + (0x31 << 16))
#define EWIFI_BASE				(ERROR_BASE + (0x32 << 16))
#define ESESSION_MGR_BASE		(ERROR_BASE + (0x33 << 16))
#define EBTSVC_CENT_BASE		(ERROR_BASE + (0x34 << 16))
#define EWIFI_MGR_BASE			(ERROR_BASE + (0x35 << 16))
#define EBTPERI_BASE			(ERROR_BASE + (0x36 << 16))
#define EBTCENTRAL_BASE			(ERROR_BASE + (0x37 << 16))
#define EMEMBRANE_PANEL_BASE	(ERROR_BASE + (0x38 << 16))
#define EADC_BASE				(ERROR_BASE + (0x39 << 16))
#define EAPEMBTN_BASE			(ERROR_BASE + (0x3A << 16))
#define ERHCBAT_BASE			(ERROR_BASE + (0x3B << 16))
#define ESETTINGS_BASE			(ERROR_BASE + (0x3C << 16))
#define ERHCMGR_BASE			(ERROR_BASE + (0x3D << 16))
#define EADXL_BASE				(ERROR_BASE + (0x3E << 16))
#define ESPIM_BASE				(ERROR_BASE + (0x3F << 16))
#define ELED_BARGRAPH_BASE		(ERROR_BASE + (0x40 << 16))
