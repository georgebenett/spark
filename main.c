

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "bsp_btn_ble.h"
#include "nrf_delay.h"
#include "storage.h"
#include "nrf_ble_lesc.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "nrf_drv_saadc.h"
#include "nrf_gpio.h"
#include "nrf_uart.h"
#include "nrf_uarte.h"
#include "packet.h"
#include "buffer.h"
#include "datatypes.h"
#include "esb_timeslot.h"
#include "crc.h"

#define USE_SLEEP						0
#define USE_USB							0

#define DEVICE_NAME 					"SPARK"

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  1                                           /**< Man In The Middle protection required (applicable when display module is detected). */
#define SEC_PARAM_LESC                  1                                           /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_DISPLAY_ONLY                /**< Display I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

static pm_peer_id_t m_peer_to_be_deleted = PM_PEER_ID_INVALID;

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_DURATION                100                                         /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(30, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                2048
#define UART_RX_BUF_SIZE                8192


#define ANALOG_INPUT_PIN 				NRF_GPIO_PIN_MAP(0, 30)

// Private variables

APP_TIMER_DEF(m_nrf_timer);

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
		{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

static volatile bool m_is_enabled = true;
static volatile bool m_uart_error = false;
static volatile int m_other_comm_disable_time = 0;
static volatile int m_no_sleep_cnt = 0;
static volatile int m_disconnect_cnt = 0;
static volatile int m_reset_timer = 0;

app_uart_comm_params_t m_uart_comm_params =
{
		.tx_pin_no = NRF_GPIO_PIN_MAP(0, 22),
		.rx_pin_no = NRF_GPIO_PIN_MAP(0, 23),
		.rts_pin_no   = 0,
		.cts_pin_no   = 0,
		.flow_control = APP_UART_FLOW_CONTROL_DISABLED,
		.use_parity   = false,
		.baud_rate    = NRF_UARTE_BAUDRATE_115200

};

// Functions
void ble_printf(const char* format, ...);
static void start_advertising(void);


static void pm_evt_handler(pm_evt_t const * p_evt) {
	ret_code_t err_code;

	pm_handler_on_pm_evt(p_evt);
	pm_handler_disconnect_on_sec_failure(p_evt);
	pm_handler_flash_clean(p_evt);

	switch (p_evt->evt_id) {
	case PM_EVT_CONN_SEC_SUCCEEDED: {
		pm_conn_sec_status_t conn_sec_status;

		// Check if the link is authenticated (meaning at least MITM).
		err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
		APP_ERROR_CHECK(err_code);

		if (conn_sec_status.mitm_protected) {

		} else {
			// The peer did not use MITM, disconnect.
			err_code = pm_peer_id_get(m_conn_handle, &m_peer_to_be_deleted);
			APP_ERROR_CHECK(err_code);
			err_code = sd_ble_gap_disconnect(m_conn_handle,
					BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(err_code);
		}
	} break;

	case PM_EVT_CONN_SEC_FAILED:
		m_conn_handle = BLE_CONN_HANDLE_INVALID;
		break;

	case PM_EVT_PEERS_DELETE_SUCCEEDED:
		start_advertising();
		break;

	case PM_EVT_CONN_SEC_CONFIG_REQ: {
		// Allow re-pairing
		pm_conn_sec_config_t config = {.allow_repairing = true};
		pm_conn_sec_config_reply(p_evt->conn_handle, &config);
		break;
	}

	default:
		break;
	}
}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void gap_params_init(void) {
	uint32_t                err_code;
	ble_gap_conn_params_t   gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	char *dev_name = DEVICE_NAME;
	if (m_config.name_set) {
		dev_name = m_config.name;
	}

	sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)dev_name, strlen(dev_name));

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency     = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);

	if (m_config.pin_set) {
		ble_gap_opt_t gap_opt;
		gap_opt.passkey.p_passkey = (uint8_t*)m_config.pin;
		sd_ble_opt_set(BLE_GAP_OPT_PASSKEY,(const ble_opt_t *)&gap_opt);
	}
}

static void start_advertising(void) {
	ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
	sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, 8);
}

static void nrf_qwr_error_handler(uint32_t nrf_error) {
	APP_ERROR_HANDLER(nrf_error);
}


void saadc_init(void) {
    ret_code_t err_code;

    nrf_saadc_channel_config_t channel_config =
    NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(ANALOG_INPUT_PIN);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
}

static void nus_data_handler(ble_nus_evt_t * p_evt) {
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint16_t length = p_evt->params.rx_data.length;

        // Check if the received data is one character
        if (length == 1)
       {
            nrf_saadc_value_t value;
            ret_code_t err_code = nrf_drv_saadc_sample_convert(0, &value);
            APP_ERROR_CHECK(err_code);
			nrf_drv_saadc_uninit();

            // Convert the analog value to a string
            char str[10];
            sprintf(str, "%d", value);
            uint16_t str_length = strlen(str);
            ble_nus_data_send(&m_nus, (uint8_t*)str, &str_length, m_conn_handle);
        }
    }
}

static void services_init(void) {
	ble_nus_init_t     nus_init;
	nrf_ble_qwr_init_t qwr_init = {0};

	// Initialize Queued Write Module.
	qwr_init.error_handler = nrf_qwr_error_handler;
	nrf_ble_qwr_init(&m_qwr, &qwr_init);

	// Initialize NUS.
	memset(&nus_init, 0, sizeof(nus_init));
	nus_init.data_handler = nus_data_handler;
	ble_nus_init(&m_nus, &nus_init, m_config.pin_set);
}

static void conn_params_error_handler(uint32_t nrf_error) {
	APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void) {
	ble_conn_params_init_t cp_init;

	memset(&cp_init, 0, sizeof(cp_init));

	cp_init.p_conn_params                  = NULL;
	cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
	cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
	cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
	cp_init.disconnect_on_fail             = true;
	cp_init.evt_handler                    = NULL;
	cp_init.error_handler                  = conn_params_error_handler;

	ble_conn_params_init(&cp_init);
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
	switch (ble_adv_evt) {
	case BLE_ADV_EVT_FAST:
//		bsp_indication_set(BSP_INDICATE_ADVERTISING);
		break;
	case BLE_ADV_EVT_IDLE:
		start_advertising();
		break;
	default:
		break;
	}
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) {
	if (m_config.pin_set) {
		pm_handler_secure_on_connection(p_ble_evt);
	}

	switch (p_ble_evt->header.evt_id) {
	case BLE_GAP_EVT_CONNECTED:
		m_peer_to_be_deleted = PM_PEER_ID_INVALID;
		m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
		sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, 8);
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		m_conn_handle = BLE_CONN_HANDLE_INVALID;
		m_disconnect_cnt = 100;
		if (m_peer_to_be_deleted != PM_PEER_ID_INVALID) {
			pm_peer_delete(m_peer_to_be_deleted);
			m_peer_to_be_deleted = PM_PEER_ID_INVALID;
		}
		break;

	case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
		ble_gap_phys_t const phys =
		{
				.rx_phys = BLE_GAP_PHY_AUTO,
				.tx_phys = BLE_GAP_PHY_AUTO,
		};
		sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
	} break;

	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		// Pairing not supported
		if (!m_config.pin_set) {
			sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
		}
		break;

	case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		// No system attributes have been stored.
		sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
		break;

	case BLE_GATTC_EVT_TIMEOUT:
		// Disconnect on GATT Client timeout event.
		sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		break;

	case BLE_GATTS_EVT_TIMEOUT:
		// Disconnect on GATT Server timeout event.
		sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		break;

	default:
		// No implementation needed.
		break;
	}
}

static void ble_stack_init(void) {
	nrf_sdh_enable_request();

	// Configure the BLE stack using the default settings.
	// Fetch the start address of the application RAM.
	uint32_t ram_start = 0;
	nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);

	// Enable BLE stack.
	nrf_sdh_ble_enable(&ram_start);

	// Register a handler for BLE events.
	NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt) {
	if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
		m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
//		ble_printf("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
	}
}

void gatt_init(void) {
	nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
	nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
}

void uart_event_handle(app_uart_evt_t * p_event) {
	switch (p_event->evt_type) {
	case APP_UART_DATA_READY: {
//		uint8_t byte;
//		while (app_uart_get(&byte) == NRF_SUCCESS) {
//			process packet here
//		}
	} break;

	case APP_UART_COMMUNICATION_ERROR:
		m_uart_error = true;
		break;

	case APP_UART_FIFO_ERROR:
		m_uart_error = true;
		break;

	default:
		break;
	}
}

static void peer_manager_init(void) {
    pm_init();

    ble_gap_sec_params_t sec_param;
    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    pm_sec_params_set(&sec_param);
    pm_register(pm_evt_handler);
}

static void uart_init(void) {
	uint32_t err_code;
	APP_UART_FIFO_INIT(&m_uart_comm_params,
			UART_RX_BUF_SIZE,
			UART_TX_BUF_SIZE,
			uart_event_handle,
			APP_IRQ_PRIORITY_LOW,
			err_code);
	APP_ERROR_CHECK(err_code);
}

static void advertising_init(void) {
	uint32_t err_code;
	ble_advertising_init_t init;

	memset(&init, 0, sizeof(init));

	init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
	init.advdata.include_appearance = false;
	init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

	init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

	init.config.ble_adv_fast_enabled  = true;
	init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
	init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
	init.evt_handler = on_adv_evt;

	err_code = ble_advertising_init(&m_advertising, &init);
	APP_ERROR_CHECK(err_code);

	ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}
static void esb_timeslot_data_handler(void *p_data, uint16_t length) {
	if (m_other_comm_disable_time == 0) {
		uint8_t buffer[length + 1];
		buffer[0] = COMM_EXT_NRF_ESB_RX_DATA;
		memcpy(buffer + 1, p_data, length);
		CRITICAL_REGION_ENTER();

		CRITICAL_REGION_EXIT();
	}

	m_no_sleep_cnt = 20;
}

static void nrf_timer_handler(void *p_context) {
	(void)p_context;
	app_timer_start(m_nrf_timer, APP_TIMER_TICKS(1000), NULL);
	// Reload watchdog
	NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

int main(void) {

	// Watchdog
	NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
	NRF_WDT->CRV = 5 * 32768; // 5s timout
	NRF_WDT->RREN |= WDT_RREN_RR0_Msk;
	NRF_WDT->TASKS_START = 1;

	storage_init();
	uart_init();
	app_timer_init();
	nrf_pwr_mgmt_init();
	ble_stack_init();
	gap_params_init();
	gatt_init();
	services_init();
	advertising_init();
	conn_params_init();



	if (m_config.pin_set) {
		peer_manager_init();
	}

	app_timer_create(&m_nrf_timer, APP_TIMER_MODE_SINGLE_SHOT, nrf_timer_handler);
	app_timer_start(m_nrf_timer, APP_TIMER_TICKS(1200), NULL);

	esb_timeslot_init(esb_timeslot_data_handler);
	esb_timeslot_sd_start();

	start_advertising();

	// Write "hello" on the UART
	const char* message = "iniciated\r\n";
	size_t message_length = strlen(message);
	for (size_t i = 0; i < message_length; i++) {
		while (app_uart_put(message[i]) != NRF_SUCCESS) {
			// Handle UART transmission error
			// For example, close and reinitialize UART
			app_uart_close();
			uart_init();
		}
	}

	for (;;) {

		uint8_t byte;
		while (app_uart_get(&byte) == NRF_SUCCESS) {

		}

		// https://devzone.nordicsemi.com/f/nordic-q-a/15243/high-power-consumption-when-using-fpu
		__set_FPSCR(__get_FPSCR()  & ~(0x0000009F));
		(void)__get_FPSCR();
		NVIC_ClearPendingIRQ(FPU_IRQn);

		if (m_config.pin_set) {
			nrf_ble_lesc_request_handler();
		}

		sd_app_evt_wait();

		// Reload watchdog
		NRF_WDT->RR[0] = WDT_RR_RR_Reload;
	}
}
