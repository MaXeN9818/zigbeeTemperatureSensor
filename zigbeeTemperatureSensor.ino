#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#define uS_TO_S_FACTOR 1000000ULL   /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 30            /* Time ESP32 will go to sleep (in seconds) */
#define LED_PIN RGB_BUILTIN         /* Pin internal led */
#define SENSORPIN 12                /* Pin Analog Sensor */
#define REPETITION 50              /* Reading repetitions */
#define SENSOR_SCAN 20              /* represents the number of sensor readings for the average */ 

/********************* Zigbee Configuration **************************/
#define INSTALLCODE_POLICY_ENABLE false /* enable the install code policy for security */
#define ED_AGING_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE 3000                                               /* millisecond */
#define HA_ESP_SENSOR_ENDPOINT 1                                        /*Non ho capito cosa fa ma mettendolo ad uno invia ogni secondo*/
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use */
#define ESP_ZB_ZED_CONFIG() \
  { \
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED, \
    .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
    .nwk_cfg = { \
      .zed_cfg = { \
        .ed_timeout = ED_AGING_TIMEOUT, \
        .keep_alive = ED_KEEP_ALIVE, \
      }, \
    }, \
  }
#define ESP_ZB_DEFAULT_RADIO_CONFIG() \
  { \
    .radio_mode = ZB_RADIO_MODE_NATIVE, \
  }

#define ESP_ZB_DEFAULT_HOST_CONFIG() \
  { \
    .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, \
  }

/********************* Variable **************************/
static bool isZigbeeConnection = false;  // Get if the Zigbee connection was successful
static const char *TAG = "ZGB_TEMP";    // Log Tag

/********************* Utility **************************/
// Logging Wakeup reason
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: ESP_LOGI(TAG, "Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: ESP_LOGI(TAG, "Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: ESP_LOGI(TAG, "Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: ESP_LOGI(TAG, "Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: ESP_LOGI(TAG, "Wakeup caused by ULP program"); break;
    default:
      ESP_LOGI(TAG, "Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
  }
}

// Blink Handle
TaskHandle_t blinkTaskHandle;
int r,g,b;
void set_led_color(int red, int green,int blue)
{
  r = red;
  g = green;
  b = blue;
}

void blinkTask(void *parameter)
{
  while (true) {
    neopixelWrite(LED_PIN, r, g, b);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    neopixelWrite(LED_PIN, 0, 0, 0);
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

/********************* Zigbee functions **************************/
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
  uint32_t *p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)*p_sg_p;
  switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
      ESP_LOGI(TAG, "Zigbee stack initialized");
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
      break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
      if (err_status == ESP_OK) {
        ESP_LOGI(TAG, "Start network steering");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
      } else {
        /* commissioning failed */
        ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s)\n", esp_err_to_name(err_status));
        vTaskDelay((5 * 1000) / portTICK_PERIOD_MS);
      }
      break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
      if (err_status == ESP_OK) {
        esp_zb_ieee_addr_t extended_pan_id;
        esp_zb_get_extended_pan_id(extended_pan_id);
        ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)\n",
                 extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                 extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                 esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        isZigbeeConnection = true;
      } else {
        ESP_LOGE(TAG, "Network steering was not successful (status: %s)\n", esp_err_to_name(err_status));
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
      }
      break;
    default:
      ESP_LOGW(TAG, "ZDO signal: %s (0x%x), status: %s\n", esp_zb_zdo_signal_to_string(sig_type), sig_type,
               esp_err_to_name(err_status));
      break;
  }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
  ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

// Define zigbee type and start
static void esp_zb_task(void *pvParameters) {
  // Define custom mfg/model
  char manufname[] = { 9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', '2' };
  char modelid[] = { 14, 'E', 'S', 'P', '3', '2', 'C', '6', '.', 'S', 'e', 'n', 's', 'o', 'r' };


  // Define attributes
  int *pressValueParam = (int *)(1);
  void *pressValuevoid = &pressValueParam;

  int *pressMinParam = (int *)(0);
  void *pressMinvoid = &pressMinParam;

  int *pressMaxParam = (int *)(5000);
  void *pressMaxvoid = &pressMaxParam;

  ESP_LOGI(TAG, "Zigbee start thread");
  esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
  esp_zb_init(&zb_nwk_cfg);
  // Create genBasic cluster/attribute list
  esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);


  // Add & set custom mfg/model attributes
  ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, &manufname[0]));
  ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, &modelid[0]));


  // Create temperature cluster/attribute list
  esp_zb_attribute_list_t *esp_zb_pressure_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT);
  ESP_ERROR_CHECK(esp_zb_pressure_meas_cluster_add_attr(esp_zb_pressure_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, pressValuevoid));
  ESP_ERROR_CHECK(esp_zb_pressure_meas_cluster_add_attr(esp_zb_pressure_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MIN_VALUE_ID, pressMinvoid));
  ESP_ERROR_CHECK(esp_zb_pressure_meas_cluster_add_attr(esp_zb_pressure_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MAX_VALUE_ID, pressMaxvoid));

  // Create cluster list
  esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();

  // Add clusters/attribute lists to cluster list
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_pressure_meas_cluster(esp_zb_cluster_list, esp_zb_pressure_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  // Create endpoint list
  esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

  esp_zb_endpoint_config_t endpoint_config = {
    .endpoint = HA_ESP_SENSOR_ENDPOINT, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID, .app_device_version = 0
  };
  esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);

  // // Register endpoint list
  esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
  ESP_ERROR_CHECK(esp_zb_device_register(esp_zb_ep_list));
  ESP_ERROR_CHECK(esp_zb_start(false));
  esp_zb_main_loop_iteration();
}

// Setting and startup Zigbee task
void zb_start() {
  /* Zigbee platform configuration */
  esp_zb_platform_config_t config = {
    .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
    .host_config = ESP_ZB_DEFAULT_HOST_CONFIG()
  };

  ESP_ERROR_CHECK(esp_zb_platform_config(&config));

  xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}

/********************* Zigbee sensor function **************************/
static esp_zb_zcl_report_attr_cmd_t pressure_cmd_req = {
  .zcl_basic_cmd = {
    .src_endpoint = HA_ESP_SENSOR_ENDPOINT,  // Specifica l'endpoint sorgente
  },
  .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
  .clusterID = ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
  .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
  .attributeID = ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID
};

// Handle send pressure Zigbee
esp_err_t zb_update_pressure(int32_t pressure) {
  esp_zb_zcl_status_t state = esp_zb_zcl_set_attribute_val(
    HA_ESP_SENSOR_ENDPOINT,
    ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
    &pressure,
    false);

  /* Check for error */
  if (state != ESP_ZB_ZCL_STATUS_SUCCESS) {
    ESP_LOGI(TAG, "Setting pressure attribute failed!");
    return ESP_FAIL;
  }

  /* Request sending new pressure */
  esp_zb_zcl_report_attr_cmd_req(&pressure_cmd_req);

  /* Check for error */
  if (state != ESP_ZB_ZCL_STATUS_SUCCESS) {
    ESP_LOGI(TAG, "Sending pressure attribute report command failed!");
    return ESP_FAIL;
  }

  return ESP_OK;
}

// Get pressure value and convert Mpa in Hpa
int32_t get_pressure() {

  double x, pMpa, pHpa;
  int analogValue, analogMilliVolts;
  double sumHpa = 0;


  for (int i = 0; i < SENSOR_SCAN; i++) {
      analogValue = analogRead(0);
      analogMilliVolts = analogReadMilliVolts(0);

      x = (float)analogMilliVolts / 1000.0; // convert in Volt
      pMpa = ((0.124 * x) - 0.058);
      pHpa = pMpa * 10000;

      sumHpa += pHpa;

      // print out the values you read:
      ESP_LOGI(TAG, "ADC analog value = %d\n", analogValue);
      ESP_LOGI(TAG, "ADC millivolts value = %d\n", analogMilliVolts);
      ESP_LOGI(TAG, "ADC Volt value = %f\n", x);
      ESP_LOGI(TAG, "ADC Mpa value = %f\n", pMpa);
      ESP_LOGI(TAG, "ADC Hpa value = %f\n", pHpa);
  }

  double avgHpa = sumHpa / SENSOR_SCAN;
  ESP_LOGI(TAG, "Average ADC Hpa value = %f\n", avgHpa);
  return (int)round(avgHpa);
}

/********************* SETUP **************************/
void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  print_wakeup_reason();

  // init pixel Red
  set_led_color(255, 0, 0);
  // init led blink thread 
  xTaskCreate(blinkTask, "Blink Task", 1024, NULL, 1, &blinkTaskHandle);

  // init analog read for sensor
  analogReadResolution(SENSORPIN);

  // Start Zigbee
  isZigbeeConnection = false;
  ESP_LOGI(TAG, "Startup Zigbee\n");
  zb_start();

  bool ScanCompleted = false;


  int iteratorCounter = 0;
  while (!ScanCompleted) {
    int32_t pressureHpa = get_pressure();
    float pressureBar = (float)pressureHpa / 1000;
    if (isZigbeeConnection) {
      // Send pressure Zigbee
      ESP_LOGI(TAG, "Send Pressure\n");
      ESP_ERROR_CHECK(zb_update_pressure(pressureHpa));
      set_led_color(0, 255, 0);
    } else {
      ESP_LOGI(TAG, "Await to connection zigbee\n");
      set_led_color(255, 128, 13);
    }

    iteratorCounter++;
    if (iteratorCounter >= REPETITION)
      break;

    ESP_LOGD(TAG, "%d scans left until deep sleep", (REPETITION - iteratorCounter));
    vTaskDelay((1 * 1000) / portTICK_PERIOD_MS);

  }

  set_led_color(0, 0, 0);
  vTaskDelay((1 * 1000) / portTICK_PERIOD_MS);
  vTaskDelete(blinkTaskHandle);
 

  // Setting DeepSleep Mode
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  ESP_LOGI(TAG, "Setup ESP32 to sleep for every %d Seconds", TIME_TO_SLEEP);
  // Enter deep sleep mode
  ESP_LOGI(TAG, "Going to sleep now");
  Serial.flush();
  esp_deep_sleep_start();
}

void loop() {
  // deepsleep restart setup
}
