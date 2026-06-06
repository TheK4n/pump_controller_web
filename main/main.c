#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdatomic.h>
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

#if !defined(CONFIG_PUMP_PIN)
#error "CONFIG_PUMP_PIN must be defined in menuconfig"
#endif

#if !defined(CONFIG_AP_WIFI_SSID)
#error "CONFIG_AP_WIFI_SSID must be defined in menuconfig"
#endif

#if !defined(CONFIG_AP_IP) || !defined(CONFIG_AP_GATEWAY) || !defined(CONFIG_AP_NETMASK)
#error "AP network configuration must be defined in menuconfig"
#endif

#define AP_MAX_CONN         4
#define AP_CHANNEL          6

#define ADC_CHAN0          ADC_CHANNEL_4
#define ADC_CHAN1          ADC_CHANNEL_5
#define ADC_ATTEN_DB       ADC_ATTEN_DB_12

#define THRESHOLD_UP_NVS_NAME "threshold_up"
#define THRESHOLD_LOW_NVS_NAME "threshold_low"
#define NVS_PARTITION "nvs"

#define SENSOR_ADC_CHAN 0
#define MAX_JSON_CONTENT 512

#define FILTER_SAMPLES 5

#define PRIORITY_HTTP     1
#define PRIORITY_CONTROL  2
#define PRIORITY_SENSOR   3

#define MDNS_DOMAIN "pumpctl"

#define RESET_BTN_GPIO        GPIO_NUM_15
#define LED_GPIO             GPIO_NUM_2

#define WIFI_SSID_MAX_LEN    32
#define WIFI_PASS_MAX_LEN    64

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t wifi_event_group;
static int retry_count = 0;
static const int MAX_RETRY_COUNT = 5;

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle;
static bool is_calibrated = false;

static atomic_int g_threshold_low = 0;
static atomic_int g_threshold_up = 0;
static atomic_int g_current_pressure = 0;

static const char TAG[] = "Pump Controller";

static esp_err_t parse_thresholds_json(const char *content, int *low_value, int *up_value) {
    cJSON *json = cJSON_Parse(content);
    if (!json) {
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

static esp_err_t receive_http_content(httpd_req_t *req, char **content) {
    size_t content_len = req->content_len;

    if (content_len > MAX_JSON_CONTENT) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too large");
        return ESP_FAIL;
    }

    *content = malloc(content_len + 1);
    if (!*content) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }

    int ret;
    int received = 0;
    while (received < content_len) {
        ret = httpd_req_recv(req, *content + received, content_len - received);
        if (ret <= 0) break;
        received += ret;
    }
    (*content)[content_len] = '\0';

    return ESP_OK;
}

static esp_err_t send_json_response(httpd_req_t *req, const char *format, ...) {
    char response[100];
    va_list args;
    va_start(args, format);
    vsnprintf(response, sizeof(response), format, args);
    va_end(args);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

esp_err_t start_mdns_service() {
    esp_err_t err = mdns_init();
    if (err) {
        return err;
    }

    mdns_hostname_set(MDNS_DOMAIN);
    mdns_instance_name_set("Thek4n PumpController");

    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);

    return ESP_OK;
}

esp_err_t adc_init(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) return ret;

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_oneshot_config_channel(adc_handle, ADC_CHAN0, &config);
    if (ret != ESP_OK) return ret;

    ret = adc_oneshot_config_channel(adc_handle, ADC_CHAN1, &config);
    if (ret != ESP_OK) return ret;

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ret = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);
    if (ret == ESP_OK) {
        is_calibrated = true;
        ESP_LOGI(TAG, "ADC success calibration");
    } else if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibrating not available (eFuse doesn't written)");
    } else {
        ESP_LOGE(TAG, "Error calibrating");
    }

    return ESP_OK;
}

static esp_err_t pump_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONFIG_PUMP_PIN),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 1,
        .pull_up_en = 0,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) return ret;

    gpio_set_level(CONFIG_PUMP_PIN, false);

    return ESP_OK;
}

int adc_read_raw(uint8_t channel) {
    int raw_value = 0;
    adc_channel_t adc_channel;

    if (channel == 0) {
        adc_channel = ADC_CHAN0;
    } else if (channel == 1) {
        adc_channel = ADC_CHAN1;
    } else {
        ESP_LOGE(TAG, "Wrong ADC channel: %d", channel);
        return -1;
    }

    if (adc_handle == NULL) {
        ESP_LOGE(TAG, "ADC not initialized");
        return -1;
    }

    esp_err_t ret = adc_oneshot_read(adc_handle, adc_channel, &raw_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading ADC");
        return -1;
    }
    return raw_value;
}

int adc_read_voltage(uint8_t channel) {
    int raw_value = adc_read_raw(channel);
    if (raw_value < 0) return -1;

    if (is_calibrated) {
        int voltage_mv = 0;
        esp_err_t ret = adc_cali_raw_to_voltage(cali_handle, raw_value, &voltage_mv);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error convert to voltage");
            return -1;
        }
        return voltage_mv;
    } else {
        return (raw_value * 3300) / 4095;
    }
}

static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, (const char*)assets_index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t current_pressure_handler(httpd_req_t *req) {
    int sensor_value = atomic_load(&g_current_pressure);
    return send_json_response(req, "{\"value\":%d}", sensor_value);
}

static esp_err_t pump_state_handler(httpd_req_t *req) {
    int pump_enabled = gpio_get_level(CONFIG_PUMP_PIN);

    return send_json_response(req, "{\"state\":%d}", pump_enabled);
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

    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS");
        return ESP_FAIL;
    }

    err = nvs_set_i32(my_handle, THRESHOLD_LOW_NVS_NAME, low_value);
    if (err != ESP_OK) {
        nvs_close(my_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save");
        return ESP_FAIL;
    }

    err = nvs_set_i32(my_handle, THRESHOLD_UP_NVS_NAME, up_value);
    if (err != ESP_OK) {
        nvs_close(my_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save");
        return ESP_FAIL;
    }

    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
    }

    nvs_close(my_handle);

    atomic_store(&g_threshold_low, low_value);
    atomic_store(&g_threshold_up, up_value);

    ESP_LOGI(TAG, "Thresholds saved: low: %d, up: %d", low_value, up_value);

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
    ESP_LOGI(TAG, "Thresholds updated: low: %d, up: %d", low_value, up_value);

    return send_json_response(req, "{\"success\":true,\"low\":%d,\"up\":%d}", low_value, up_value);
}

void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

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

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info;
    ip_info.ip.addr = inet_addr(CONFIG_AP_IP);
    ip_info.gw.addr = inet_addr(CONFIG_AP_GATEWAY);
    ip_info.netmask.addr = inet_addr(CONFIG_AP_NETMASK);

    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));
}

static void pump_disable(void) {
    gpio_set_level(CONFIG_PUMP_PIN, false);
}

static void pump_enable(void) {
    gpio_set_level(CONFIG_PUMP_PIN, true);
}

static void vPumpControlTask(void *pvParameters) {
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());
    bool pump_enabled = false;

    while (1) {
        int current_pressure = atomic_load(&g_current_pressure);
        int low_threshold = atomic_load(&g_threshold_low);
        int up_threshold = atomic_load(&g_threshold_up);

        if ((current_pressure < low_threshold) && (!pump_enabled)) {
            pump_enable();
            pump_enabled = true;
            ESP_LOGI(TAG, "Pump enabled");
        } else if ((current_pressure >= up_threshold) && pump_enabled) {
            pump_disable();
            pump_enabled = false;
            ESP_LOGI(TAG, "Pump disabled");
        }

        esp_task_wdt_reset();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static int read_voltage_filtered(void) {
    int sum = 0;
    for (int i = 0; i < FILTER_SAMPLES; i++) {
        sum += adc_read_voltage(SENSOR_ADC_CHAN);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return sum / FILTER_SAMPLES;
}

int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static int convert_adc_to_pressure_atm(int adc_value) {
    return map(adc_value, 330, 3145, 0, 1184);
}

static void vReadSensorTask(void *pvParameters) {
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());

    while (1) {
        int voltage = read_voltage_filtered();
        int pressure = convert_adc_to_pressure_atm(voltage);
        atomic_store(&g_current_pressure, pressure);

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void register_http_handlers(httpd_handle_t server) {
    httpd_uri_t root = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = root_get_handler,
        .user_ctx  = NULL
    };
    if (httpd_register_uri_handler(server, &root) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GET / handler");
    }

    httpd_uri_t pressure = {
        .uri       = "/pressure",
        .method    = HTTP_GET,
        .handler   = current_pressure_handler,
        .user_ctx  = NULL
    };
    if (httpd_register_uri_handler(server, &pressure) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GET /pressure handler");
    }

    httpd_uri_t pump_state = {
        .uri       = "/state",
        .method    = HTTP_GET,
        .handler   = pump_state_handler,
        .user_ctx  = NULL
    };
    if (httpd_register_uri_handler(server, &pump_state) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GET /state handler");
    }

    httpd_uri_t get_thresholds = {
        .uri       = "/thresholds",
        .method    = HTTP_GET,
        .handler   = get_thresholds_handler,
        .user_ctx  = NULL
    };
    if (httpd_register_uri_handler(server, &get_thresholds) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GET /thresholds handler");
    }

    httpd_uri_t set_thresholds = {
        .uri       = "/thresholds",
        .method    = HTTP_POST,
        .handler   = set_thresholds_handler,
        .user_ctx  = NULL
    };
    if (httpd_register_uri_handler(server, &set_thresholds) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register POST /thresholds handler");
    }

    httpd_uri_t save_thresholds = {
        .uri       = "/persist_thresholds",
        .method    = HTTP_POST,
        .handler   = save_thresholds_handler,
        .user_ctx  = NULL
    };
    if (httpd_register_uri_handler(server, &save_thresholds) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register POST /persist_thresholds handler");
    }
}

static void vHttpServerTask(void *pvParameters) {
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CONFIG_WEBINTERFACE_PORT;
    config.max_uri_handlers = 10;
    config.stack_size = 4096;

    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "🚀 HTTP server run on port %d", config.server_port);
        register_http_handlers(server);
    } else {
        ESP_LOGE(TAG, "❌Error HTTP server running");
    }

    while (1) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static esp_err_t setup_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, (const char*)assets_setup_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void save_wifi_config(const char *ssid, const char *password) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_PARTITION, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS");
        return;
    }

    nvs_set_str(nvs_handle, "wifi_ssid", ssid);
    nvs_set_str(nvs_handle, "wifi_pass", password);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    ESP_LOGI(TAG, "WiFi config saved: SSID=%s", ssid);
}


static esp_err_t parse_wifi_settings_json(const char *content, char *ssid, char *password) {
    cJSON *json = cJSON_Parse(content);
    if (!json) {
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

static void vSetupHttpServerTask(void *pvParameters) {
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CONFIG_WEBINTERFACE_PORT;
    config.max_uri_handlers = 10;
    config.stack_size = 4096;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error HTTP server running");
    } else {
        ESP_LOGI(TAG, "🚀 Setup HTTP server run on port %d", config.server_port);

        httpd_uri_t root = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = setup_get_handler,
            .user_ctx  = NULL
        };
        if (httpd_register_uri_handler(server, &root) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register GET / handler");
        }

        httpd_uri_t settings = {
            .uri       = "/settings",
            .method    = HTTP_POST,
            .handler   = setup_set_settings_handler,
            .user_ctx  = NULL
        };
        if (httpd_register_uri_handler(server, &settings) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register GET /settings handler");
        }
    }

    while (1) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void load_thresholds_from_nvs(void) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS");
        return;
    }

    int32_t threshold_low = 0;
    err = nvs_get_i32(my_handle, THRESHOLD_LOW_NVS_NAME, &threshold_low);
    if (err == ESP_OK) {
        atomic_store(&g_threshold_low, threshold_low);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        atomic_store(&g_threshold_low, 100);
    } else {
        ESP_ERROR_CHECK(err);
    }

    int32_t threshold_up = 0;
    err = nvs_get_i32(my_handle, THRESHOLD_UP_NVS_NAME, &threshold_up);
    if (err == ESP_OK) {
        atomic_store(&g_threshold_up, threshold_up);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        atomic_store(&g_threshold_up, 300);
    } else {
        ESP_ERROR_CHECK(err);
    }

    nvs_close(my_handle);
}

bool is_wifi_config_exists() {
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

// Сброс настроек и очистка NVS
void reset_settings() {
    ESP_LOGI(TAG, "Resetting settings...");

    // Очищаем NVS
    nvs_flash_erase();
    nvs_flash_init();

    ESP_LOGI(TAG, "Settings reset, restarting...");
    vTaskDelay(pdMS_TO_TICKS(500));
}

void indicate_led() {
    for (int i = 0; i < 3; i++) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

}

// Обработчик событий WiFi
static void event_handler(void* arg, esp_event_base_t event_base,
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

            esp_restart();
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_connect_sta(const char* ssid, const char* password) {
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };

    strcpy((char*)wifi_config.sta.ssid, ssid);
    strcpy((char*)wifi_config.sta.password, password);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA started");

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Successfully connected to WiFi!");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to WiFi!");
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO, 0);

    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << RESET_BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_conf);

    vTaskDelay(pdMS_TO_TICKS(100));

    if (gpio_get_level(RESET_BTN_GPIO) == 0) {
        ESP_LOGI(TAG, "RESET button pressed, resetting settings...");
        reset_settings();
        indicate_led();

        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
        return;
    }

    bool should_run_setup_mode = !is_wifi_config_exists();
    if (should_run_setup_mode) {
        wifi_init_softap();
        ESP_LOGI(TAG, "=========================================");
        ESP_LOGI(TAG, "No WiFi settings found, starting setup mode");
        ESP_LOGI(TAG, "📱 Connect to Wi-Fi: %s", CONFIG_AP_WIFI_SSID);
        ESP_LOGI(TAG, "🔑 Wi-Fi Password: %s", strlen(CONFIG_AP_WIFI_PASS) ? CONFIG_AP_WIFI_PASS : "Open network");
        ESP_LOGI(TAG, "🌐 Open browser: http://%s:%d", CONFIG_AP_IP, CONFIG_WEBINTERFACE_PORT);
        ESP_LOGI(TAG, "=========================================");

        xTaskCreate(vSetupHttpServerTask, "setup_http_server", 8192, NULL, PRIORITY_HTTP, NULL);
    } else {
        ESP_LOGI(TAG, "WiFi settings found, starting normal mode");

        load_thresholds_from_nvs();
        ESP_ERROR_CHECK(adc_init());
        ESP_ERROR_CHECK(pump_init());

        nvs_handle_t nvs_handle;
        nvs_open(NVS_PARTITION, NVS_READONLY, &nvs_handle);

        char ssid[WIFI_SSID_MAX_LEN];
        char password[WIFI_PASS_MAX_LEN];
        size_t ssid_len = sizeof(ssid);
        size_t pass_len = sizeof(password);

        nvs_get_str(nvs_handle, "wifi_ssid", ssid, &ssid_len);
        nvs_get_str(nvs_handle, "wifi_pass", password, &pass_len);
        nvs_close(nvs_handle);

        ESP_LOGI(TAG, "Using saved SSID: %s", ssid);
        wifi_connect_sta(ssid, password);

        ESP_ERROR_CHECK(start_mdns_service());

        vTaskDelay(pdMS_TO_TICKS(1000));

        xTaskCreate(vReadSensorTask, "read_sensor", 2048, NULL, PRIORITY_SENSOR, NULL);
        xTaskCreate(vPumpControlTask, "pump_control", 2048, NULL, PRIORITY_CONTROL, NULL);
        xTaskCreate(vHttpServerTask, "http_server", 8192, NULL, PRIORITY_HTTP, NULL);
    }

    vTaskDelete(NULL);
}
