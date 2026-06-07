#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdatomic.h>
#include <stdarg.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/projdefs.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <driver/gpio.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include <cJSON.h>
#include "esp_http_server.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_task_wdt.h"
#include "mdns.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "lwip/inet.h"

#include "sdkconfig.h"
#include "frontend.h"
#include "setup_frontend.h"

// ============================================================================
// Configuration validation
// ============================================================================

#if !defined(CONFIG_PUMP_PIN)
#error "CONFIG_PUMP_PIN must be defined in menuconfig"
#endif

#if !defined(CONFIG_AP_WIFI_SSID)
#error "CONFIG_AP_WIFI_SSID must be defined in menuconfig"
#endif

#if !defined(CONFIG_AP_IP) || !defined(CONFIG_AP_GATEWAY) || !defined(CONFIG_AP_NETMASK)
#error "AP network configuration must be defined in menuconfig"
#endif

// ============================================================================
// Constants
// ============================================================================

#define AP_MAX_CONN         4
#define AP_CHANNEL          6

#define ADC_CHAN0           ADC_CHANNEL_4
#define ADC_CHAN1           ADC_CHANNEL_5
#define ADC_ATTEN_DB        ADC_ATTEN_DB_12

#define THRESHOLD_UP_NVS_NAME   "threshold_up"
#define THRESHOLD_LOW_NVS_NAME  "threshold_low"
#define NVS_PARTITION           "nvs"

#define SENSOR_ADC_CHAN     0
#define MAX_JSON_CONTENT    512

#define FILTER_SAMPLES      5

#define PRIORITY_HTTP       1
#define PRIORITY_CONTROL    2
#define PRIORITY_SENSOR     3

#define MDNS_DOMAIN         "pumpctl"

#define RESET_BTN_GPIO      GPIO_NUM_15
#define LED_GPIO            GPIO_NUM_2

#define WIFI_SSID_MAX_LEN   32
#define WIFI_PASS_MAX_LEN   64

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

#define DEFAULT_THRESHOLD_LOW   0
#define DEFAULT_THRESHOLD_UP    300

#define MAX_AP_SCAN_RESULTS 20

// ============================================================================
// Error codes
// ============================================================================

typedef enum {
    APP_ERR_OK = 0,
    APP_ERR_NVS_INIT_FAIL,
    APP_ERR_NVS_OPEN_FAIL,
    APP_ERR_NVS_READ_FAIL,
    APP_ERR_NVS_WRITE_FAIL,
    APP_ERR_ADC_INIT_FAIL,
    APP_ERR_ADC_CALIB_FAIL,
    APP_ERR_PUMP_INIT_FAIL,
    APP_ERR_WIFI_INIT_FAIL,
    APP_ERR_WIFI_CONNECT_FAIL,
    APP_ERR_MDNS_INIT_FAIL,
    APP_ERR_HTTP_SERVER_START_FAIL,
    APP_ERR_TASK_CREATE_FAIL,
    APP_ERR_GPIO_CONFIG_FAIL,
    APP_ERR_INVALID_CONFIG,
    APP_ERR_MEMORY_ALLOC_FAIL,
    APP_ERR_JSON_PARSE_FAIL
} app_error_t;

// ============================================================================
// Global error handler
// ============================================================================

static const char *TAG = "Pump Controller";

typedef struct {
    app_error_t code;
    const char *message;
    bool fatal;
} error_info_t;

static const error_info_t error_table[] = {
    {APP_ERR_OK, "Success", false},
    {APP_ERR_NVS_INIT_FAIL, "NVS initialization failed", true},
    {APP_ERR_NVS_OPEN_FAIL, "NVS open failed", true},
    {APP_ERR_NVS_READ_FAIL, "NVS read failed", false},
    {APP_ERR_NVS_WRITE_FAIL, "NVS write failed", false},
    {APP_ERR_ADC_INIT_FAIL, "ADC initialization failed", true},
    {APP_ERR_ADC_CALIB_FAIL, "ADC calibration failed", false},
    {APP_ERR_PUMP_INIT_FAIL, "Pump GPIO initialization failed", true},
    {APP_ERR_WIFI_INIT_FAIL, "WiFi initialization failed", true},
    {APP_ERR_WIFI_CONNECT_FAIL, "WiFi connection failed", false},
    {APP_ERR_MDNS_INIT_FAIL, "mDNS service initialization failed", false},
    {APP_ERR_HTTP_SERVER_START_FAIL, "HTTP server start failed", false},
    {APP_ERR_TASK_CREATE_FAIL, "Task creation failed", true},
    {APP_ERR_GPIO_CONFIG_FAIL, "GPIO configuration failed", true},
    {APP_ERR_INVALID_CONFIG, "Invalid configuration", true},
    {APP_ERR_MEMORY_ALLOC_FAIL, "Memory allocation failed", false},
    {APP_ERR_JSON_PARSE_FAIL, "JSON parse failed", false}
};

static void handle_error(app_error_t error) {
    const char *message = "Unknown error";
    bool fatal = true;

    for (int i = 0; i < sizeof(error_table) / sizeof(error_info_t); i++) {
        if (error_table[i].code == error) {
            message = error_table[i].message;
            fatal = error_table[i].fatal;
            break;
        }
    }

    ESP_LOGE(TAG, "ERROR [%d]: %s", error, message);

    if (fatal) {
        ESP_LOGE(TAG, "Fatal error! Restarting in 5 seconds...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }
}

#define CHECK_ERROR(expr, error_code) \
    do { \
        esp_err_t __err = (expr); \
        if (__err != ESP_OK) { \
            ESP_LOGE(TAG, "Error at %s:%d: %s", __FILE__, __LINE__, esp_err_to_name(__err)); \
            handle_error(error_code); \
            return error_code; \
        } \
    } while(0)

#define CHECK_COND(cond, error_code) \
    do { \
        if (!(cond)) { \
            ESP_LOGE(TAG, "Condition failed at %s:%d", __FILE__, __LINE__); \
            handle_error(error_code); \
            return error_code; \
        } \
    } while(0)

#define CHECK_PTR(ptr, error_code) \
    do { \
        if ((ptr) == NULL) { \
            ESP_LOGE(TAG, "Null pointer at %s:%d", __FILE__, __LINE__); \
            handle_error(error_code); \
            return error_code; \
        } \
    } while(0)

// ============================================================================
// Global state
// ============================================================================

static EventGroupHandle_t wifi_event_group = NULL;
static int retry_count = 0;
static const int MAX_RETRY_COUNT = 5;

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t cali_handle = NULL;
static bool is_calibrated = false;

static atomic_int g_threshold_low = 0;
static atomic_int g_threshold_up = 0;
static atomic_int g_current_pressure = 0;

static bool g_pump_enabled = false;
static bool g_setup_mode = false;

// ============================================================================
// Forward declarations
// ============================================================================

static app_error_t nvs_init(void);
static app_error_t gpio_init(void);
static app_error_t adc_init(void);
static app_error_t pump_init(void);
static app_error_t load_thresholds(void);
static app_error_t save_thresholds_to_nvs(int low, int up);
static app_error_t wifi_softap_init(void);
static app_error_t wifi_sta_init(const char *ssid, const char *password);
static app_error_t mdns_init_service(void);
static app_error_t http_server_start(void);
static app_error_t setup_http_server_start(void);
static app_error_t tasks_create(void);
static bool is_wifi_config_exists(void);
static void reset_settings(void);

// ============================================================================
// Utility functions
// ============================================================================

static int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static int convert_adc_to_pressure_atm(int adc_value) {
    return map(adc_value, 330, 3145, 0, 1184);
}

static void pump_disable(void) {
    gpio_set_level(CONFIG_PUMP_PIN, false);
    g_pump_enabled = false;
}

static void pump_enable(void) {
    gpio_set_level(CONFIG_PUMP_PIN, true);
    g_pump_enabled = true;
}

static void indicate_led(void) {
    for (int i = 0; i < 3; i++) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ============================================================================
// ADC functions
// ============================================================================

static app_error_t adc_init(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    CHECK_ERROR(adc_oneshot_new_unit(&init_config, &adc_handle), APP_ERR_ADC_INIT_FAIL);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    CHECK_ERROR(adc_oneshot_config_channel(adc_handle, ADC_CHAN0, &config), APP_ERR_ADC_INIT_FAIL);
    CHECK_ERROR(adc_oneshot_config_channel(adc_handle, ADC_CHAN1, &config), APP_ERR_ADC_INIT_FAIL);

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);
    if (ret == ESP_OK) {
        is_calibrated = true;
        ESP_LOGI(TAG, "ADC calibration successful");
    } else if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "ADC calibration not available");
        is_calibrated = false;
    } else {
        CHECK_ERROR(ret, APP_ERR_ADC_CALIB_FAIL);
    }

    return ERR_OK;
}

static int adc_read_raw(uint8_t channel) {
    adc_channel_t adc_channel;

    if (channel == 0) {
        adc_channel = ADC_CHAN0;
    } else if (channel == 1) {
        adc_channel = ADC_CHAN1;
    } else {
        ESP_LOGE(TAG, "Invalid ADC channel: %d", channel);
        return -1;
    }

    CHECK_PTR(adc_handle, APP_ERR_ADC_INIT_FAIL);

    int raw_value = 0;
    CHECK_ERROR(adc_oneshot_read(adc_handle, adc_channel, &raw_value), APP_ERR_ADC_INIT_FAIL);
    return raw_value;
}

static int adc_read_voltage(uint8_t channel) {
    int raw_value = adc_read_raw(channel);
    if (raw_value < 0) return -1;

    if (is_calibrated) {
        int voltage_mv = 0;
        CHECK_ERROR(adc_cali_raw_to_voltage(cali_handle, raw_value, &voltage_mv), APP_ERR_ADC_INIT_FAIL);
        return voltage_mv;
    } else {
        return (raw_value * 3300) / 4095;
    }
}

static int read_voltage_filtered(void) {
    int sum = 0;
    for (int i = 0; i < FILTER_SAMPLES; i++) {
        int voltage = adc_read_voltage(SENSOR_ADC_CHAN);
        if (voltage < 0) return -1;
        sum += voltage;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return sum / FILTER_SAMPLES;
}

// ============================================================================
// NVS functions
// ============================================================================

static app_error_t nvs_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        CHECK_ERROR(nvs_flash_erase(), APP_ERR_NVS_INIT_FAIL);
        CHECK_ERROR(nvs_flash_init(), APP_ERR_NVS_INIT_FAIL);
    } else {
        CHECK_ERROR(ret, APP_ERR_NVS_INIT_FAIL);
    }
    return ERR_OK;
}

static bool is_wifi_config_exists(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_PARTITION, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return false;
    }

    char ssid[WIFI_SSID_MAX_LEN];
    size_t ssid_len = sizeof(ssid);
    err = nvs_get_str(nvs_handle, "wifi_ssid", ssid, &ssid_len);
    nvs_close(nvs_handle);

    return (err == ESP_OK);
}

static app_error_t load_thresholds(void) {
    nvs_handle_t my_handle;
    CHECK_ERROR(nvs_open(NVS_PARTITION, NVS_READWRITE, &my_handle), APP_ERR_NVS_OPEN_FAIL);

    int32_t threshold_low = DEFAULT_THRESHOLD_LOW;
    esp_err_t err = nvs_get_i32(my_handle, THRESHOLD_LOW_NVS_NAME, &threshold_low);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(my_handle);
        CHECK_ERROR(err, APP_ERR_NVS_READ_FAIL);
    }
    atomic_store(&g_threshold_low, threshold_low);

    int32_t threshold_up = DEFAULT_THRESHOLD_UP;
    err = nvs_get_i32(my_handle, THRESHOLD_UP_NVS_NAME, &threshold_up);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(my_handle);
        CHECK_ERROR(err, APP_ERR_NVS_READ_FAIL);
    }
    atomic_store(&g_threshold_up, threshold_up);

    nvs_close(my_handle);

    ESP_LOGI(TAG, "Thresholds loaded: low=%d, up=%d", threshold_low, threshold_up);
    return ERR_OK;
}

static app_error_t save_thresholds_to_nvs(int low, int up) {
    nvs_handle_t my_handle;
    CHECK_ERROR(nvs_open(NVS_PARTITION, NVS_READWRITE, &my_handle), APP_ERR_NVS_OPEN_FAIL);

    CHECK_ERROR(nvs_set_i32(my_handle, THRESHOLD_LOW_NVS_NAME, low), APP_ERR_NVS_WRITE_FAIL);
    CHECK_ERROR(nvs_set_i32(my_handle, THRESHOLD_UP_NVS_NAME, up), APP_ERR_NVS_WRITE_FAIL);
    CHECK_ERROR(nvs_commit(my_handle), APP_ERR_NVS_WRITE_FAIL);

    nvs_close(my_handle);
    return ERR_OK;
}

static void save_wifi_config(const char *ssid, const char *password) {
    nvs_handle_t nvs_handle;
    if (nvs_open(NVS_PARTITION, NVS_READWRITE, &nvs_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for WiFi config");
        return;
    }

    nvs_set_str(nvs_handle, "wifi_ssid", ssid);
    nvs_set_str(nvs_handle, "wifi_pass", password);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    ESP_LOGI(TAG, "WiFi config saved: SSID=%s", ssid);
}

static void reset_settings(void) {
    ESP_LOGI(TAG, "Resetting settings...");
    nvs_flash_erase();
    nvs_flash_init();
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ============================================================================
// GPIO functions
// ============================================================================

static app_error_t gpio_init(void) {
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    CHECK_ERROR(gpio_config(&led_conf), APP_ERR_GPIO_CONFIG_FAIL);
    gpio_set_level(LED_GPIO, 0);

    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << RESET_BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    CHECK_ERROR(gpio_config(&btn_conf), APP_ERR_GPIO_CONFIG_FAIL);

    return ERR_OK;
}

static app_error_t pump_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONFIG_PUMP_PIN),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 1,
        .pull_up_en = 0,
    };
    CHECK_ERROR(gpio_config(&io_conf), APP_ERR_PUMP_INIT_FAIL);
    pump_disable();
    return ERR_OK;
}

// ============================================================================
// WiFi functions
// ============================================================================

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Attempting to connect to WiFi...");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_count < MAX_RETRY_COUNT) {
            esp_wifi_connect();
            retry_count++;
            ESP_LOGI(TAG, "Retry connecting (%d/%d)...", retry_count, MAX_RETRY_COUNT);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Failed to connect after %d retries", MAX_RETRY_COUNT);
            handle_error(APP_ERR_WIFI_CONNECT_FAIL);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static app_error_t wifi_softap_init(void) {
    CHECK_ERROR(esp_netif_init(), APP_ERR_WIFI_INIT_FAIL);
    CHECK_ERROR(esp_event_loop_create_default(), APP_ERR_WIFI_INIT_FAIL);

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    CHECK_PTR(ap_netif, APP_ERR_WIFI_INIT_FAIL);

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    CHECK_PTR(sta_netif, APP_ERR_WIFI_INIT_FAIL);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    CHECK_ERROR(esp_wifi_init(&cfg), APP_ERR_WIFI_INIT_FAIL);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = CONFIG_AP_WIFI_SSID,
            .ssid_len = strlen(CONFIG_AP_WIFI_SSID),
            .password = CONFIG_AP_WIFI_PASS,
            .max_connection = AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .channel = AP_CHANNEL,
        },
    };

    if (strlen(CONFIG_AP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    CHECK_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), APP_ERR_WIFI_INIT_FAIL);

    CHECK_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifi_config), APP_ERR_WIFI_INIT_FAIL);

    wifi_config_t sta_config = {
        .sta = {
            .ssid = "",
            .password = "",
            .scan_method = WIFI_FAST_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold.rssi = -127,
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };

    CHECK_ERROR(esp_wifi_set_config(WIFI_IF_STA, &sta_config), APP_ERR_WIFI_INIT_FAIL);

    CHECK_ERROR(esp_wifi_start(), APP_ERR_WIFI_INIT_FAIL);

    esp_netif_ip_info_t ip_info;
    ip_info.ip.addr = inet_addr(CONFIG_AP_IP);
    ip_info.gw.addr = inet_addr(CONFIG_AP_GATEWAY);
    ip_info.netmask.addr = inet_addr(CONFIG_AP_NETMASK);

    CHECK_ERROR(esp_netif_dhcps_stop(ap_netif), APP_ERR_WIFI_INIT_FAIL);
    CHECK_ERROR(esp_netif_set_ip_info(ap_netif, &ip_info), APP_ERR_WIFI_INIT_FAIL);
    CHECK_ERROR(esp_netif_dhcps_start(ap_netif), APP_ERR_WIFI_INIT_FAIL);

    return ERR_OK;
}

static app_error_t wifi_sta_init(const char* ssid, const char* password) {
    wifi_event_group = xEventGroupCreate();
    CHECK_PTR(wifi_event_group, APP_ERR_WIFI_INIT_FAIL);

    CHECK_ERROR(esp_netif_init(), APP_ERR_WIFI_INIT_FAIL);
    CHECK_ERROR(esp_event_loop_create_default(), APP_ERR_WIFI_INIT_FAIL);
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    CHECK_ERROR(esp_wifi_init(&cfg), APP_ERR_WIFI_INIT_FAIL);

    CHECK_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL), APP_ERR_WIFI_INIT_FAIL);
    CHECK_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL), APP_ERR_WIFI_INIT_FAIL);

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    strcpy((char*)wifi_config.sta.ssid, ssid);
    strcpy((char*)wifi_config.sta.password, password);

    CHECK_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), APP_ERR_WIFI_INIT_FAIL);
    CHECK_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), APP_ERR_WIFI_INIT_FAIL);
    CHECK_ERROR(esp_wifi_start(), APP_ERR_WIFI_INIT_FAIL);

    ESP_LOGI(TAG, "WiFi STA started");

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Successfully connected to WiFi!");
        return ERR_OK;
    } else {
        return APP_ERR_WIFI_CONNECT_FAIL;
    }
}

// ============================================================================
// mDNS functions
// ============================================================================

static app_error_t mdns_init_service(void) {
    CHECK_ERROR(mdns_init(), APP_ERR_MDNS_INIT_FAIL);
    CHECK_ERROR(mdns_hostname_set(MDNS_DOMAIN), APP_ERR_MDNS_INIT_FAIL);
    CHECK_ERROR(mdns_instance_name_set("PumpController"), APP_ERR_MDNS_INIT_FAIL);
    CHECK_ERROR(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0), APP_ERR_MDNS_INIT_FAIL);
    return ERR_OK;
}

// ============================================================================
// HTTP handlers
// ============================================================================

static esp_err_t send_json_response(httpd_req_t *req, const char *format, ...) {
    esp_err_t ret = ESP_OK;
    char *response = NULL;
    va_list args;
    va_list args_copy;

    va_start(args, format);
    va_copy(args_copy, args);
    int len = vsnprintf(NULL, 0, format, args_copy);
    va_end(args_copy);
    va_end(args);

    if (len < 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Format error");
        return ESP_FAIL;
    }

    response = malloc(len + 1);
    if (response == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }

    va_start(args, format);
    vsnprintf(response, len + 1, format, args);
    va_end(args);

    httpd_resp_set_type(req, "application/json");
    ret = httpd_resp_send(req, response, len);

    free(response);
    return ret;
}

static esp_err_t receive_http_content(httpd_req_t *req, char **content) {
    size_t content_len = req->content_len;

    if (content_len > MAX_JSON_CONTENT) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too large");
        return ESP_FAIL;
    }

    *content = malloc(content_len + 1);
    if (!*content) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        handle_error(APP_ERR_MEMORY_ALLOC_FAIL);
        return ESP_FAIL;
    }

    int received = 0;
    while (received < content_len) {
        int ret = httpd_req_recv(req, *content + received, content_len - received);
        if (ret <= 0) break;
        received += ret;
    }
    (*content)[content_len] = '\0';

    return ESP_OK;
}

static esp_err_t parse_thresholds_json(const char *content, int *low_value, int *up_value) {
    cJSON *json = cJSON_Parse(content);
    if (!json) {
        handle_error(APP_ERR_JSON_PARSE_FAIL);
        return ESP_FAIL;
    }

    cJSON *low_item = cJSON_GetObjectItem(json, "low");
    cJSON *up_item = cJSON_GetObjectItem(json, "up");

    bool valid = cJSON_IsNumber(low_item) && cJSON_IsNumber(up_item);
    if (valid) {
        *low_value = low_item->valueint;
        *up_value = up_item->valueint;
    }

    cJSON_Delete(json);
    return valid ? ESP_OK : ESP_FAIL;
}

static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, (const char*)assets_index_html, assets_index_html_len);
    return ESP_OK;
}

static esp_err_t current_pressure_handler(httpd_req_t *req) {
    int sensor_value = atomic_load(&g_current_pressure);
    return send_json_response(req, "{\"value\":%d}", sensor_value);
}

static esp_err_t pump_state_handler(httpd_req_t *req) {
    return send_json_response(req, "{\"state\":%d}", g_pump_enabled ? 1 : 0);
}

static esp_err_t save_thresholds_handler(httpd_req_t *req) {
    char *content = NULL;
    if (receive_http_content(req, &content) != ESP_OK) {
        return ESP_FAIL;
    }

    int low_value = 0, up_value = 0;
    if (parse_thresholds_json(content, &low_value, &up_value) != ESP_OK) {
        free(content);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid 'low' or 'up' parameters");
        return ESP_FAIL;
    }
    free(content);

    if (low_value >= up_value) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Low value must be less than up value");
        return ESP_FAIL;
    }

    if (save_thresholds_to_nvs(low_value, up_value) != APP_ERR_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save to NVS");
        return ESP_FAIL;
    }

    atomic_store(&g_threshold_low, low_value);
    atomic_store(&g_threshold_up, up_value);

    ESP_LOGI(TAG, "Thresholds saved: low=%d, up=%d", low_value, up_value);
    return send_json_response(req, "{\"success\":true,\"low\":%d,\"up\":%d}", low_value, up_value);
}

static esp_err_t get_thresholds_handler(httpd_req_t *req) {
    int low_threshold = atomic_load(&g_threshold_low);
    int up_threshold = atomic_load(&g_threshold_up);
    return send_json_response(req, "{\"low\":%d,\"up\":%d}", low_threshold, up_threshold);
}

static esp_err_t set_thresholds_handler(httpd_req_t *req) {
    char *content = NULL;
    if (receive_http_content(req, &content) != ESP_OK) {
        return ESP_FAIL;
    }

    int low_value = 0, up_value = 0;
    if (parse_thresholds_json(content, &low_value, &up_value) != ESP_OK) {
        free(content);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid 'low' or 'up' parameters");
        return ESP_FAIL;
    }
    free(content);

    if (low_value >= up_value) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Low value must be less than up value");
        return ESP_FAIL;
    }

    atomic_store(&g_threshold_low, low_value);
    atomic_store(&g_threshold_up, up_value);
    ESP_LOGI(TAG, "Thresholds updated: low=%d, up=%d", low_value, up_value);

    return send_json_response(req, "{\"success\":true,\"low\":%d,\"up\":%d}", low_value, up_value);
}

// ============================================================================
// Setup mode HTTP handlers
// ============================================================================

static esp_err_t setup_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, (const char*)assets_setup_html, assets_setup_html_len);
    return ESP_OK;
}

static esp_err_t parse_wifi_settings_json(const char *content, char *ssid, char *password) {
    cJSON *json = cJSON_Parse(content);
    if (!json) {
        handle_error(APP_ERR_JSON_PARSE_FAIL);
        return ESP_FAIL;
    }

    cJSON *ssid_item = cJSON_GetObjectItem(json, "ssid");
    cJSON *pass_item = cJSON_GetObjectItem(json, "password");

    if (!ssid_item || !pass_item ||
        ssid_item->type != cJSON_String ||
        pass_item->type != cJSON_String) {
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    if (strlen(ssid_item->valuestring) >= WIFI_SSID_MAX_LEN ||
        strlen(pass_item->valuestring) >= WIFI_PASS_MAX_LEN) {
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    strlcpy(ssid, ssid_item->valuestring, WIFI_SSID_MAX_LEN);
    strlcpy(password, pass_item->valuestring, WIFI_PASS_MAX_LEN);

    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t setup_set_settings_handler(httpd_req_t *req) {
    char *content = NULL;
    if (receive_http_content(req, &content) != ESP_OK) {
        return ESP_FAIL;
    }

    char ssid[WIFI_SSID_MAX_LEN];
    char password[WIFI_PASS_MAX_LEN];
    if (parse_wifi_settings_json(content, ssid, password) != ESP_OK) {
        free(content);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid 'ssid' or 'password' parameters");
        return ESP_FAIL;
    }
    free(content);

    send_json_response(req, "{\"success\":true}");
    save_wifi_config(ssid, password);
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}


static esp_err_t setup_get_wifi_list_handler(httpd_req_t *req) {
    esp_err_t ret;
    uint16_t ap_count = 0;
    wifi_ap_record_t ap_info[MAX_AP_SCAN_RESULTS];
    memset(ap_info, 0, sizeof(ap_info));

    // Получаем текущий режим Wi-Fi
    wifi_mode_t current_mode;
    ret = esp_wifi_get_mode(&current_mode);
    if (ret != ESP_OK) {
        send_json_response(req, "{\"success\":false,\"error\":\"Failed to get WiFi mode\"}");
        return ESP_OK;
    }

    // Запускаем сканирование
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time = {
            .active = {
                .min = 100,
                .max = 300
            }
        }
    };

    ret = esp_wifi_scan_start(&scan_config, true);
    if (ret != ESP_OK) {
        send_json_response(req, "{\"success\":false,\"error\":\"Failed to start WiFi scan\"}");
        return ESP_OK;
    }

    ret = esp_wifi_scan_get_ap_num(&ap_count);
    if (ret != ESP_OK) {
        send_json_response(req, "{\"success\":false,\"error\":\"Failed to get AP count\"}");
        return ESP_OK;
    }

    if (ap_count > MAX_AP_SCAN_RESULTS) {
        ap_count = MAX_AP_SCAN_RESULTS;
    }

    ret = esp_wifi_scan_get_ap_records(&ap_count, ap_info);
    if (ret != ESP_OK) {
        send_json_response(req, "{\"success\":false,\"error\":\"Failed to get AP records\"}");
        return ESP_OK;
    }

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        send_json_response(req, "{\"success\":false,\"error\":\"JSON creation failed\"}");
        return ESP_OK;
    }

    cJSON_AddBoolToObject(root, "success", true);

    cJSON *networks_array = cJSON_CreateArray();
    if (networks_array == NULL) {
        cJSON_Delete(root);
        send_json_response(req, "{\"success\":false,\"error\":\"JSON array creation failed\"}");
        return ESP_OK;
    }

    for (int i = 0; i < ap_count; i++) {
        cJSON *network = cJSON_CreateObject();
        if (network == NULL) {
            continue;
        }

        char ssid_str[33];
        memcpy(ssid_str, ap_info[i].ssid, 32);
        ssid_str[32] = '\0';
        cJSON_AddStringToObject(network, "ssid", ssid_str);

        // RSSI (сигнал)
        cJSON_AddNumberToObject(network, "rssi", ap_info[i].rssi);

        // Тип шифрования (в читаемом виде)
        const char *auth_mode_str;
        switch (ap_info[i].authmode) {
            case WIFI_AUTH_OPEN:
                auth_mode_str = "Open";
                break;
            case WIFI_AUTH_WEP:
                auth_mode_str = "WEP";
                break;
            case WIFI_AUTH_WPA_PSK:
                auth_mode_str = "WPA-PSK";
                break;
            case WIFI_AUTH_WPA2_PSK:
                auth_mode_str = "WPA2-PSK";
                break;
            case WIFI_AUTH_WPA_WPA2_PSK:
                auth_mode_str = "WPA/WPA2-PSK";
                break;
            case WIFI_AUTH_WPA3_PSK:
                auth_mode_str = "WPA3-PSK";
                break;
            case WIFI_AUTH_WPA2_WPA3_PSK:
                auth_mode_str = "WPA2/WPA3-PSK";
                break;
            default:
                auth_mode_str = "Unknown";
                break;
        }
        cJSON_AddStringToObject(network, "auth_mode", auth_mode_str);

        cJSON_AddItemToArray(networks_array, network);
    }

    cJSON_AddItemToObject(root, "networks", networks_array);
    cJSON_AddNumberToObject(root, "count", ap_count);

    char *json_string = cJSON_PrintUnformatted(root);
    if (json_string == NULL) {
        cJSON_Delete(root);
        send_json_response(req, "{\"success\":false,\"error\":\"JSON string conversion failed\"}");
        return ESP_OK;
    }

    send_json_response(req, json_string);

    free(json_string);
    cJSON_Delete(root);

    return ESP_OK;
}


// ============================================================================
// HTTP server tasks
// ============================================================================

static app_error_t http_server_start(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CONFIG_WEBINTERFACE_PORT;
    config.max_uri_handlers = 10;
    config.stack_size = 4096;

    CHECK_ERROR(httpd_start(&server, &config), APP_ERR_HTTP_SERVER_START_FAIL);
    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);

    httpd_uri_t uris[] = {
        {.uri = "/", .method = HTTP_GET, .handler = root_get_handler},
        {.uri = "/pressure", .method = HTTP_GET, .handler = current_pressure_handler},
        {.uri = "/state", .method = HTTP_GET, .handler = pump_state_handler},
        {.uri = "/thresholds", .method = HTTP_GET, .handler = get_thresholds_handler},
        {.uri = "/thresholds", .method = HTTP_POST, .handler = set_thresholds_handler},
        {.uri = "/persist_thresholds", .method = HTTP_POST, .handler = save_thresholds_handler},
    };

    for (int i = 0; i < sizeof(uris) / sizeof(httpd_uri_t); i++) {
        CHECK_ERROR(httpd_register_uri_handler(server, &uris[i]), APP_ERR_HTTP_SERVER_START_FAIL);
    }

    return ERR_OK;
}

static app_error_t setup_http_server_start(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CONFIG_WEBINTERFACE_PORT;
    config.max_uri_handlers = 10;
    config.stack_size = 8192;

    CHECK_ERROR(httpd_start(&server, &config), APP_ERR_HTTP_SERVER_START_FAIL);
    ESP_LOGI(TAG, "Setup HTTP server started on port %d", config.server_port);

    httpd_uri_t root = {
        .uri = "/", .method = HTTP_GET, .handler = setup_get_handler
    };
    CHECK_ERROR(httpd_register_uri_handler(server, &root), APP_ERR_HTTP_SERVER_START_FAIL);

    httpd_uri_t settings = {
        .uri = "/settings", .method = HTTP_POST, .handler = setup_set_settings_handler
    };
    CHECK_ERROR(httpd_register_uri_handler(server, &settings), APP_ERR_HTTP_SERVER_START_FAIL);

    httpd_uri_t wifi_list = {
        .uri = "/wifi_list", .method = HTTP_GET, .handler = setup_get_wifi_list_handler
    };
    CHECK_ERROR(httpd_register_uri_handler(server, &wifi_list), APP_ERR_HTTP_SERVER_START_FAIL);

    return ERR_OK;
}

// ============================================================================
// Application tasks
// ============================================================================

static void vReadSensorTask(void *pvParameters) {
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());

    while (1) {
        int voltage = read_voltage_filtered();
        if (voltage >= 0) {
            int pressure = convert_adc_to_pressure_atm(voltage);
            atomic_store(&g_current_pressure, pressure);
        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void vPumpControlTask(void *pvParameters) {
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());

    while (1) {
        int current_pressure = atomic_load(&g_current_pressure);
        int low_threshold = atomic_load(&g_threshold_low);
        int up_threshold = atomic_load(&g_threshold_up);

        if ((current_pressure < low_threshold) && !g_pump_enabled) {
            pump_enable();
            ESP_LOGI(TAG, "Pump enabled (pressure=%d < low=%d)", current_pressure, low_threshold);
        } else if ((current_pressure >= up_threshold) && g_pump_enabled) {
            pump_disable();
            ESP_LOGI(TAG, "Pump disabled (pressure=%d >= up=%d)", current_pressure, up_threshold);
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// Tasks creation
// ============================================================================

static app_error_t tasks_create(void) {
    CHECK_ERROR(xTaskCreate(vReadSensorTask, "read_sensor", 2048, NULL, PRIORITY_SENSOR, NULL) == pdPASS ? ESP_OK : ESP_FAIL, APP_ERR_TASK_CREATE_FAIL);
    CHECK_ERROR(xTaskCreate(vPumpControlTask, "pump_control", 2048, NULL, PRIORITY_CONTROL, NULL) == pdPASS ? ESP_OK : ESP_FAIL, APP_ERR_TASK_CREATE_FAIL);
    return ERR_OK;
}

// ============================================================================
// Mode initialization
// ============================================================================

static app_error_t setup_mode_init(void) {
    g_setup_mode = true;

    CHECK_ERROR(wifi_softap_init(), APP_ERR_WIFI_INIT_FAIL);
    CHECK_ERROR(setup_http_server_start(), APP_ERR_HTTP_SERVER_START_FAIL);

    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "Setup mode active");
    ESP_LOGI(TAG, "📱 Connect to Wi-Fi: %s", CONFIG_AP_WIFI_SSID);
    ESP_LOGI(TAG, "🔑 Password: %s", strlen(CONFIG_AP_WIFI_PASS) ? CONFIG_AP_WIFI_PASS : "Open network");
    ESP_LOGI(TAG, "🌐 Open browser: http://%s:%d", CONFIG_AP_IP, CONFIG_WEBINTERFACE_PORT);
    ESP_LOGI(TAG, "=========================================");

    return ERR_OK;
}

static app_error_t ap_normal_mode_init() {
    CHECK_ERROR(wifi_softap_init(), APP_ERR_WIFI_INIT_FAIL);

    CHECK_ERROR(load_thresholds(), APP_ERR_NVS_READ_FAIL);
    CHECK_ERROR(adc_init(), APP_ERR_ADC_INIT_FAIL);
    CHECK_ERROR(pump_init(), APP_ERR_PUMP_INIT_FAIL);

    CHECK_ERROR(http_server_start(), APP_ERR_HTTP_SERVER_START_FAIL);
    CHECK_ERROR(tasks_create(), APP_ERR_TASK_CREATE_FAIL);

    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "AP normal mode active");
    ESP_LOGI(TAG, "📱 Connect to Wi-Fi: %s", CONFIG_AP_WIFI_SSID);
    ESP_LOGI(TAG, "🔑 Password: %s", strlen(CONFIG_AP_WIFI_PASS) ? CONFIG_AP_WIFI_PASS : "Open network");
    ESP_LOGI(TAG, "🌐 Open browser: http://%s:%d", CONFIG_AP_IP, CONFIG_WEBINTERFACE_PORT);
    ESP_LOGI(TAG, "=========================================");

    return ERR_OK;
}

static app_error_t normal_mode_init(void) {
    g_setup_mode = false;

    CHECK_ERROR(load_thresholds(), APP_ERR_NVS_READ_FAIL);
    CHECK_ERROR(adc_init(), APP_ERR_ADC_INIT_FAIL);
    CHECK_ERROR(pump_init(), APP_ERR_PUMP_INIT_FAIL);

    nvs_handle_t nvs_handle;
    CHECK_ERROR(nvs_open(NVS_PARTITION, NVS_READONLY, &nvs_handle), APP_ERR_NVS_OPEN_FAIL);

    char ssid[WIFI_SSID_MAX_LEN];
    char password[WIFI_PASS_MAX_LEN];
    size_t ssid_len = sizeof(ssid);
    size_t pass_len = sizeof(password);

    CHECK_ERROR(nvs_get_str(nvs_handle, "wifi_ssid", ssid, &ssid_len), APP_ERR_NVS_READ_FAIL);
    CHECK_ERROR(nvs_get_str(nvs_handle, "wifi_pass", password, &pass_len), APP_ERR_NVS_READ_FAIL);
    nvs_close(nvs_handle);

    ESP_LOGI(TAG, "Connecting to WiFi: %s", ssid);
    CHECK_ERROR(wifi_sta_init(ssid, password), APP_ERR_WIFI_CONNECT_FAIL);

    mdns_init_service();
    CHECK_ERROR(http_server_start(), APP_ERR_HTTP_SERVER_START_FAIL);
    CHECK_ERROR(tasks_create(), APP_ERR_TASK_CREATE_FAIL);

    return ERR_OK;
}

// ============================================================================
// Main application
// ============================================================================

void app_main(void) {
    // Initialize NVS
    if (nvs_init() != APP_ERR_OK) {
        handle_error(APP_ERR_NVS_INIT_FAIL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    // Initialize GPIO
    if (gpio_init() != APP_ERR_OK) {
        handle_error(APP_ERR_GPIO_CONFIG_FAIL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

#if CONFIG_WIFI_AP
    if (ap_normal_mode_init() != APP_ERR_OK) {
        handle_error(APP_ERR_WIFI_INIT_FAIL);
    }
#else // CONFIG_WIFI_AP
    // Check reset button
    if (gpio_get_level(RESET_BTN_GPIO) == 0) {
        ESP_LOGI(TAG, "RESET button pressed, resetting settings...");
        reset_settings();
        indicate_led();
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
        return;
    }

    // Start appropriate mode
    if (!is_wifi_config_exists()) {
        if (setup_mode_init() != APP_ERR_OK) {
            handle_error(APP_ERR_WIFI_INIT_FAIL);
        }
    } else {
        if (normal_mode_init() != APP_ERR_OK) {
            handle_error(APP_ERR_WIFI_CONNECT_FAIL);
        }
    }
#endif // CONFIG_WIFI_AP
    vTaskDelete(NULL);
}
