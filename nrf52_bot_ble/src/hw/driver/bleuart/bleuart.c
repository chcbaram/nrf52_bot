/*
 * bleuart.c
 *
 *  Created on: 2020. 4. 30.
 *      Author: Baram
 */




#include "bleuart.h"
#include "led.h"
#include "qbuffer.h"


#ifdef _USE_HW_BLEUART

#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_ble_qwr.h"
#include "nrf_ble_gatt.h"
#include "ble_nus.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "app_timer.h"



#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */


#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */



BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */



static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};



#define BLE_ERROR_CHECK(err) if (err) {return err;}




static uint32_t ble_stack_init(void);
static uint32_t gap_params_init(void);
static uint32_t gatt_init(void);
static uint32_t services_init(void);
static uint32_t advertising_init(void);
static uint32_t conn_params_init(void);
static uint32_t advertising_start(void);




#define BLEUART_RX_BUF_LENGTH      512


static uint8_t  rx_buf[BLEUART_RX_BUF_LENGTH];
static qbuffer_t qbuffer_rx;


volatile static bool is_init = false;
volatile static bool is_connected = false;
volatile static bool is_tx_ready = false;



#define APP_TIMER_TEST    0

#if APP_TIMER_TEST == 1
static uint32_t pre_time;
static app_timer_t timer_data;
static app_timer_id_t  timer_id = (app_timer_id_t)&timer_data;


void update_timeout_handler(void * p_context)
{
  uartPrintf(_DEF_UART2, "timeout %d\n", millis()-pre_time);
  pre_time = millis();
}
#endif


bool bleUartInit(void)
{
  uint32_t err_code = 0;

  app_timer_init();

#if APP_TIMER_TEST == 1
  app_timer_create(&timer_id, APP_TIMER_MODE_REPEATED, update_timeout_handler);
  app_timer_start(timer_id, 10000, NULL);
  pre_time = millis();
#endif

  err_code |= ble_stack_init();
  err_code |= gap_params_init();
  err_code |= gatt_init();
  err_code |= services_init();
  err_code |= advertising_init();
  err_code |= conn_params_init();
  err_code |= advertising_start();

  if (err_code != 0)
  {
    return false;
  }

  is_init = true;

  qbufferCreate(&qbuffer_rx, rx_buf, BLEUART_RX_BUF_LENGTH);




  return true;
}

bool bleUartUpdate(void)
{
  app_timer_update();
  return true;
}

bool bleUartIsConnected(void)
{
  return is_connected;
}

uint32_t bleUartAvailable(void)
{
  uint32_t ret = true;

  ret = qbufferAvailable(&qbuffer_rx);

  return ret;
}

bool bleUartSendPacket(uint8_t *p_data, uint16_t length)
{
  bool ret = false;
  uint32_t pre_time;
  uint32_t err_code;


  is_tx_ready = false;
  err_code = ble_nus_data_send(&m_nus, p_data, &length, m_conn_handle);
  if (err_code != 0)
  {
    return false;
  }

  pre_time = millis();
  while(1)
  {
    if (is_tx_ready == true)
    {
      ret = true;
      break;
    }
    if (millis()-pre_time >= 100)
    {
      break;
    }
#ifdef _USE_HW_RTOS
    osThreadYield();
#endif
  }

  return ret;
}

int32_t bleUartWrite(uint8_t *p_data, uint32_t length)
{
  int32_t ret = 0;
  int32_t total_len = 0;
  uint16_t tx_len = 0;
  uint32_t pre_time;


  if (is_init != true) return 0;
  if (is_connected != true) return 0;


  pre_time = millis();
  while(total_len < length)
  {
    tx_len = length;
    if (tx_len > BLE_NUS_MAX_DATA_LEN)
    {
      tx_len = BLE_NUS_MAX_DATA_LEN;
    }

    if (bleUartSendPacket(&p_data[total_len], tx_len) != true)
    {
      break;
    }

    total_len += tx_len;

    if (millis()-pre_time >= 100)
    {
      break;
    }
#ifdef _USE_HW_RTOS
    osThreadYield();
#endif
  }

  ret = total_len;

  return ret;
}

uint8_t bleUartRead(void)
{
  uint8_t ret = 0;

  qbufferRead(&qbuffer_rx, &ret, 1);

  return ret;
}

int32_t bleUartPrintf(const char *fmt, ...)
{
  int32_t ret = 0;
  va_list arg;
  va_start (arg, fmt);
  int32_t len;
  static char print_buffer[128];


  len = vsnprintf(print_buffer, 128, fmt, arg);
  va_end (arg);

  ret = bleUartWrite((uint8_t *)print_buffer, len);

  return ret;
}


static void handler_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);
static void handler_gatt_evt(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt);
static void handler_nrf_qwr_error(uint32_t nrf_error);
static void handler_nus_data(ble_nus_evt_t * p_evt);
static void handler_on_adv_evt(ble_adv_evt_t ble_adv_evt);
static void handler_on_conn_params_evt(ble_conn_params_evt_t * p_evt);
static void handler_conn_params_error(uint32_t nrf_error);


uint32_t ble_stack_init(void)
{
  ret_code_t err_code;


  NVIC_DisableIRQ(POWER_CLOCK_IRQn);

  err_code = nrf_sdh_enable_request();
  BLE_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  BLE_ERROR_CHECK(err_code);

  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  BLE_ERROR_CHECK(err_code);

  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, handler_ble_evt, NULL);


  return (uint32_t)err_code;
}

uint32_t gap_params_init(void)
{
  uint32_t                err_code;
  ble_gap_conn_params_t   gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
                                        (const uint8_t *) DEVICE_NAME,
                                        strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency     = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  BLE_ERROR_CHECK(err_code);

  return err_code;
}

uint32_t gatt_init(void)
{
  ret_code_t err_code;

  err_code = nrf_ble_gatt_init(&m_gatt, handler_gatt_evt);
  BLE_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  BLE_ERROR_CHECK(err_code);

  return (uint32_t)err_code;
}

uint32_t services_init(void)
{
  uint32_t           err_code;
  ble_nus_init_t     nus_init;
  nrf_ble_qwr_init_t qwr_init = {0};

  // Initialize Queued Write Module.
  qwr_init.error_handler = handler_nrf_qwr_error;

  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  BLE_ERROR_CHECK(err_code);

  // Initialize NUS.
  memset(&nus_init, 0, sizeof(nus_init));

  nus_init.data_handler = handler_nus_data;

  err_code = ble_nus_init(&m_nus, &nus_init);
  BLE_ERROR_CHECK(err_code);

  return err_code;
}

uint32_t advertising_init(void)
{
  uint32_t               err_code;
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
  init.evt_handler = handler_on_adv_evt;

  err_code = ble_advertising_init(&m_advertising, &init);
  BLE_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

  return err_code;
}

uint32_t conn_params_init(void)
{
  uint32_t               err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params                  = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail             = false;
  cp_init.evt_handler                    = handler_on_conn_params_evt;
  cp_init.error_handler                  = handler_conn_params_error;

  err_code = ble_conn_params_init(&cp_init);
  BLE_ERROR_CHECK(err_code);

  return err_code;
}

uint32_t advertising_start(void)
{
  uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
  BLE_ERROR_CHECK(err_code);

  return err_code;
}







void handler_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
  uint32_t err_code;

  switch (p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
      NRF_LOG_INFO("Connected");
      ledOn(1);
      is_connected = true;
#if 0
      err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
      APP_ERROR_CHECK(err_code);
#endif
      m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
      err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
      APP_ERROR_CHECK(err_code);

      break;

    case BLE_GAP_EVT_DISCONNECTED:
      NRF_LOG_INFO("Disconnected");
      ledOff(1);
      is_connected = false;
      // LED indication will be changed when advertising starts.
      m_conn_handle = BLE_CONN_HANDLE_INVALID;
      break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
      NRF_LOG_DEBUG("PHY update request.");
      ble_gap_phys_t const phys =
      {
          .rx_phys = BLE_GAP_PHY_AUTO,
          .tx_phys = BLE_GAP_PHY_AUTO,
      };
      err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
      APP_ERROR_CHECK(err_code);
    } break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      // Pairing not supported
      err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
      // No system attributes have been stored.
      err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTC_EVT_TIMEOUT:
      // Disconnect on GATT Client timeout event.
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_TIMEOUT:
      // Disconnect on GATT Server timeout event.
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    default:
      // No implementation needed.
      break;
  }
}


void handler_gatt_evt(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
  if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
  {
    m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
  }
  NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                p_gatt->att_mtu_desired_central,
                p_gatt->att_mtu_desired_periph);
}

void handler_nrf_qwr_error(uint32_t nrf_error)
{
  //TODO: 향후 에러처리 추가함.
  //APP_ERROR_HANDLER(nrf_error);
}

void handler_nus_data(ble_nus_evt_t * p_evt)
{
  if (p_evt->type == BLE_NUS_EVT_RX_DATA)
  {
    qbufferWrite(&qbuffer_rx, (uint8_t *)p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
  }
  if (p_evt->type == BLE_NUS_EVT_TX_RDY)
  {
    is_tx_ready = true;
  }
}

void handler_on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
  //uint32_t err_code;

  switch (ble_adv_evt)
  {
    case BLE_ADV_EVT_FAST:
      //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
      //APP_ERROR_CHECK(err_code);
      break;
    case BLE_ADV_EVT_IDLE:
      //sleep_mode_enter();
      ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
      break;
    default:
      break;
  }
}

void handler_on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
  uint32_t err_code;

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
  {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}

void handler_conn_params_error(uint32_t nrf_error)
{
  //TODO: 향후 에러처리 추가함.
  //APP_ERROR_HANDLER(nrf_error);
}





#endif
