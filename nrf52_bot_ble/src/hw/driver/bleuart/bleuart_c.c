/*
 * bleuart_c.c
 *
 *  Created on: 2020. 5. 2.
 *      Author: Baram
 */




#include "bleuart_c.h"
#include "led.h"
#include "qbuffer.h"


#ifdef _USE_HW_BLEUART_C

#include "nordic_common.h"
#include "app_error.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */


#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */



BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid =
{
    .uuid = BLE_UUID_NUS_SERVICE,
    .type = NUS_SERVICE_UUID_TYPE
};



#define BLE_ERROR_CHECK(err) if (err) {return err;}



#define BLEUART_RX_BUF_LENGTH      512


static uint8_t  rx_buf[BLEUART_RX_BUF_LENGTH];
static qbuffer_t qbuffer_rx;


volatile static bool is_init = false;
volatile static bool is_connected = false;
volatile static bool is_tx_ready = false;



static uint32_t db_discovery_init();
static uint32_t ble_stack_init(void);
static uint32_t gatt_init(void);
static uint32_t nus_c_init(void);
static uint32_t scan_init(void);
static uint32_t scan_start(void);


bool bleUartInit(void)
{
  uint32_t err_code = 0;

  app_timer_init();

  qbufferCreate(&qbuffer_rx, rx_buf, BLEUART_RX_BUF_LENGTH);


  err_code |= db_discovery_init();
  err_code |= ble_stack_init();
  err_code |= gatt_init();
  err_code |= nus_c_init();
  err_code |= scan_init();

  err_code |= scan_start();

  if (err_code != 0)
  {
    return false;
  }

  is_init = true;



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

  err_code = ble_nus_c_string_send(&m_ble_nus_c, p_data, length);
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



static void handler_db_disc(ble_db_discovery_evt_t * p_evt);
static void handler_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);
static void handler_gatt_evt(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt);
static void handler_ble_nus_c_evt(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt);
static void handler_nus_error(uint32_t nrf_error);
static void handler_scan_evt(scan_evt_t const * p_scan_evt);



uint32_t db_discovery_init(void)
{
  ble_db_discovery_init_t db_init;

  memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

  db_init.evt_handler  = handler_db_disc;
  db_init.p_gatt_queue = &m_ble_gatt_queue;

  ret_code_t err_code = ble_db_discovery_init(&db_init);
  BLE_ERROR_CHECK(err_code);

  return err_code;
}

uint32_t ble_stack_init(void)
{
  ret_code_t err_code;

  // sd에서 해당 인터럽트를 활성화 하기에 이미 enable 되어 있으면 에러 발생함.
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

  return err_code;
}

uint32_t gatt_init(void)
{
  ret_code_t err_code;

  err_code = nrf_ble_gatt_init(&m_gatt, handler_gatt_evt);
  BLE_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  BLE_ERROR_CHECK(err_code);

  return err_code;
}

uint32_t nus_c_init(void)
{
  ret_code_t       err_code;
  ble_nus_c_init_t init;

  init.evt_handler   = handler_ble_nus_c_evt;
  init.error_handler = handler_nus_error;
  init.p_gatt_queue  = &m_ble_gatt_queue;

  err_code = ble_nus_c_init(&m_ble_nus_c, &init);
  BLE_ERROR_CHECK(err_code);

  return err_code;
}

uint32_t scan_init(void)
{
  ret_code_t          err_code;
  nrf_ble_scan_init_t init_scan;

  memset(&init_scan, 0, sizeof(init_scan));

  init_scan.connect_if_match = true;
  init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

  err_code = nrf_ble_scan_init(&m_scan, &init_scan, handler_scan_evt);
  BLE_ERROR_CHECK(err_code);

  err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
  BLE_ERROR_CHECK(err_code);

  err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
  BLE_ERROR_CHECK(err_code);

  return err_code;
}

uint32_t scan_start(void)
{
  ret_code_t ret;

  ret = nrf_ble_scan_start(&m_scan);
  BLE_ERROR_CHECK(ret);

  //ret = bsp_indication_set(BSP_INDICATE_SCANNING);
  //BLE_ERROR_CHECK(ret);

  return ret;
}




void handler_db_disc(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}
volatile int cnt = 0;

void handler_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
  ret_code_t            err_code;
  ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

  switch (p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
      err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
      APP_ERROR_CHECK(err_code);

      //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
      //APP_ERROR_CHECK(err_code);

      // start discovery of services. The NUS Client waits for a discovery result
      err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
      APP_ERROR_CHECK(err_code);

      is_connected = true;
      break;

    case BLE_GAP_EVT_DISCONNECTED:

      NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                   p_gap_evt->conn_handle,
                   p_gap_evt->params.disconnected.reason);

      is_connected = false;
      break;

    case BLE_GAP_EVT_TIMEOUT:
      if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
      {
          NRF_LOG_INFO("Connection Request timed out.");
      }
      break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      // Pairing not supported.
      err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
      // Accepting parameters requested by peer.
      err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                              &p_gap_evt->params.conn_param_update_request.conn_params);
      APP_ERROR_CHECK(err_code);
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

    case BLE_GATTC_EVT_TIMEOUT:
      // Disconnect on GATT Client timeout event.
      NRF_LOG_DEBUG("GATT Client Timeout.");
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_TIMEOUT:
      // Disconnect on GATT Server timeout event.
      NRF_LOG_DEBUG("GATT Server Timeout.");
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTC_EVT_WRITE_RSP:
    case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
      is_tx_ready = true;
      break;

    default:
      break;
  }
}

void handler_gatt_evt(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
  if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
  {
    NRF_LOG_INFO("ATT MTU exchange completed.");

    m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
  }
}

void handler_ble_nus_c_evt(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
  ret_code_t err_code;

  switch (p_ble_nus_evt->evt_type)
  {
    case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
      NRF_LOG_INFO("Discovery complete.");
      err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
      APP_ERROR_CHECK(err_code);

      err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
      APP_ERROR_CHECK(err_code);
      NRF_LOG_INFO("Connected to device with Nordic UART Service.");
      break;

    case BLE_NUS_C_EVT_NUS_TX_EVT:
      qbufferWrite(&qbuffer_rx, (uint8_t *)p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
      break;

    case BLE_NUS_C_EVT_DISCONNECTED:
      NRF_LOG_INFO("Disconnected.");
      scan_start();
      break;
  }
}

void handler_nus_error(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}

void handler_scan_evt(scan_evt_t const * p_scan_evt)
{
  ret_code_t err_code;

  switch(p_scan_evt->scan_evt_id)
  {
     case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
     {
        err_code = p_scan_evt->params.connecting_err.err_code;
        APP_ERROR_CHECK(err_code);
     } break;

     case NRF_BLE_SCAN_EVT_CONNECTED:
     {
        ble_gap_evt_connected_t const * p_connected =
                         p_scan_evt->params.connected.p_connected;
       // Scan is automatically stopped by the connection.
       NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                p_connected->peer_addr.addr[0],
                p_connected->peer_addr.addr[1],
                p_connected->peer_addr.addr[2],
                p_connected->peer_addr.addr[3],
                p_connected->peer_addr.addr[4],
                p_connected->peer_addr.addr[5]
                );
     } break;

     case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
     {
       NRF_LOG_INFO("Scan timed out.");
       scan_start();
     } break;

     default:
         break;
  }
}

#endif
