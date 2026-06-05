#include <stdint.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <driver/gpio.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include "esp_http_server.h"
#include <stdio.h>
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdatomic.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "frontend.h"
#include <cJSON.h>
#include "sdkconfig.h"

// ==================== НАСТРОЙКИ ТОЧКИ ДОСТУПА ====================
#define AP_MAX_CONN         4                      // Максимум клиентов
#define AP_CHANNEL          6                      // Wi-Fi канал

#define ADC_CHAN0          ADC_CHANNEL_4
#define ADC_CHAN1          ADC_CHANNEL_5
#define ADC_ATTEN_DB       ADC_ATTEN_DB_12

#define THRESHOLD_UP_NVS_NAME "threshold_up"
#define THRESHOLD_LOW_NVS_NAME "threshold_low"

#define SENSOR_ADC_CHAN 0

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle;
static bool is_calibrated = false;

static atomic_int g_threshold_low = 0;
static atomic_int g_threshold_up = 0;

static atomic_int g_current_pressure = 0;


static const char *TAG = "ESP32_AP_SERVER";

esp_err_t adc_init(void)
{
    // Инициализация ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // Конфигурация каналов
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHAN0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHAN1, &config));

    // Калибровка для ESP32 (Line Fitting)
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);
    if (ret == ESP_OK) {
        is_calibrated = true;
        ESP_LOGI(TAG, "ADC калибровка успешна");
    } else if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Калибровка не доступна (eFuse не записан)");
    } else {
        ESP_LOGE(TAG, "Ошибка калибровки");
    }

    return ESP_OK;
}

static void pump_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONFIG_PUMP_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 1,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
}

int adc_read_raw(uint8_t channel)
{
    int raw_value = 0;
    adc_channel_t adc_channel;

    if (channel == 0) {
        adc_channel = ADC_CHAN0;
    } else if (channel == 1) {
        adc_channel = ADC_CHAN1;
    } else {
        ESP_LOGE(TAG, "Неверный канал: %d", channel);
        return -1;
    }

    esp_err_t ret = adc_oneshot_read(adc_handle, adc_channel, &raw_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка чтения ADC");
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
            ESP_LOGE(TAG, "Ошибка конвертации в напряжение");
            return -1;
        }
        return voltage_mv;
    } else {
        // Приблизительный расчет без калибровки (12-bit ADC: 0-4095 -> 0-3300mV)
        return (raw_value * 3300) / 4095;
    }
}


static esp_err_t root_get_handler(httpd_req_t *req) {
    const char* response =
        "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "    <title>ESP32 Access Point Server</title>"
        "    <meta charset='utf-8'>"
        "    <meta name='viewport' content='width=device-width, initial-scale=1'>"
        "    <style>"
        "        * { margin: 0; padding: 0; box-sizing: border-box; }"
        "        body {"
        "            font-family: 'Segoe UI', Arial, sans-serif;"
        "            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);"
        "            min-height: 100vh;"
        "            padding: 20px;"
        "        }"
        "        .container {"
        "            max-width: 800px;"
        "            margin: 0 auto;"
        "            background: white;"
        "            border-radius: 20px;"
        "            padding: 30px;"
        "            box-shadow: 0 20px 60px rgba(0,0,0,0.3);"
        "        }"
        "        h1 {"
        "            color: #667eea;"
        "            margin-bottom: 20px;"
        "            text-align: center;"
        "        }"
        "        .status-card {"
        "            background: #f0f0f0;"
        "            border-radius: 10px;"
        "            padding: 20px;"
        "            margin: 20px 0;"
        "        }"
        "        .info {"
        "            margin: 10px 0;"
        "            padding: 10px;"
        "            background: #e8f5e9;"
        "            border-left: 4px solid #4CAF50;"
        "        }"
        "        .button {"
        "            background: #667eea;"
        "            color: white;"
        "            border: none;"
        "            padding: 12px 24px;"
        "            border-radius: 8px;"
        "            cursor: pointer;"
        "            font-size: 16px;"
        "            margin: 5px;"
        "            transition: transform 0.2s;"
        "        }"
        "        .button:hover {"
        "            transform: scale(1.05);"
        "        }"
        "        .gpio-control {"
        "            margin: 20px 0;"
        "            padding: 15px;"
        "            background: #fff3e0;"
        "            border-radius: 10px;"
        "        }"
        "        .led-status {"
        "            display: inline-block;"
        "            width: 20px;"
        "            height: 20px;"
        "            border-radius: 50%;"
        "            margin-left: 10px;"
        "        }"
        "        .led-on { background: #4CAF50; box-shadow: 0 0 10px #4CAF50; }"
        "        .led-off { background: #f44336; }"
        "    </style>"
        "</head>"
        "<body>"
        "    <div class='container'>"
        "        <h1>🎯 ESP32 Точка Доступа</h1>"
        "        <div class='status-card'>"
        "            <h3>📡 Информация о сервере</h3>"
        "            <div class='info'>✅ HTTP сервер работает</div>"
        "            <div class='info'>🌐 IP адрес: 192.168.4.1</div>"
        "            <div class='info'>💾 Свободно памяти: <span id='heap'>0</span> байт</div>"
        "            <div class='info'>🕐 Время работы: <span id='uptime'>0</span> сек</div>"
        "        </div>"
        "        <div class='gpio-control'>"
        "            <h3>💡 Управление GPIO2 (встроенный LED)</h3>"
        "            <button class='button' onclick='controlGPIO(1)'>🔆 ВКЛ</button>"
        "            <button class='button' onclick='controlGPIO(0)'>💡 ВЫКЛ</button>"
        "            <span id='ledIndicator' class='led-status led-off'></span>"
        "        </div>"
        "        <div class='gpio-control'>"
        "            <h3>📊 Получить данные с датчика</h3>"
        "            <button class='button' onclick='getSensorData()'>📈 Обновить</button>"
        "            <div id='sensorData' class='info' style='margin-top:10px;'>Данные не загружены</div>"
        "        </div>"
        "    </div>"
        "    <script>"
        "        function controlGPIO(state) {"
        "            fetch('/gpio?state=' + state)"
        "                .then(response => response.json())"
        "                .then(data => {"
        "                    const indicator = document.getElementById('ledIndicator');"
        "                    if(data.status === 'on') {"
        "                        indicator.className = 'led-status led-on';"
        "                    } else {"
        "                        indicator.className = 'led-status led-off';"
        "                    }"
        "                });"
        "        }"
        "        function getSensorData() {"
        "            fetch('/sensor')"
        "                .then(response => response.json())"
        "                .then(data => {"
        "                    document.getElementById('sensorData').innerHTML = "
        "                        '📊 Значение: ' + data.value + '<br>' +"
        "                        '📝 Сообщение: ' + data.message;"
        "                });"
        "        }"
        "        function updateStats() {"
        "            fetch('/stats')"
        "                .then(response => response.json())"
        "                .then(data => {"
        "                    document.getElementById('heap').innerText = data.free_heap;"
        "                    document.getElementById('uptime').innerText = data.uptime;"
        "                });"
        "        }"
        "        setInterval(updateStats, 2000);"
        "        getSensorData();"
        "    </script>"
        "</body>"
        "</html>";

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, (const char*)assets_index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t current_pressure_handler(httpd_req_t *req) {
    int sensor_value = atomic_load(&g_current_pressure);
    char response[100];
    snprintf(response, sizeof(response),
             "{\"value\":%d}",
             sensor_value);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

static esp_err_t save_thresholds_handler(httpd_req_t *req) {
    char response[100];
    char *content = NULL;
    size_t content_len = req->content_len;

    // Проверка длины содержимого
    if (content_len > 512) { // Ограничение размера для безопасности
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too large");
        return ESP_FAIL;
    }

    // Выделение памяти под содержимое запроса
    content = malloc(content_len + 1);
    if (!content) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }

    // Чтение тела запроса
    int ret = httpd_req_recv(req, content, content_len);
    if (ret <= 0) {
        free(content);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to receive data");
        return ESP_FAIL;
    }
    content[content_len] = '\0'; // Null-terminator

    // Парсинг JSON
    cJSON *json = cJSON_Parse(content);
    free(content);

    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    // Извлечение параметров low и up
    cJSON *low_item = cJSON_GetObjectItem(json, "low");
    cJSON *up_item = cJSON_GetObjectItem(json, "up");

    int low_value = 0;
    int up_value = 0;
    bool valid = true;

    if (cJSON_IsNumber(low_item)) {
        low_value = low_item->valueint;
    } else {
        valid = false;
    }

    if (cJSON_IsNumber(up_item)) {
        up_value = up_item->valueint;
    } else {
        valid = false;
    }

    cJSON_Delete(json);

    if (!valid) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid 'low' or 'up' parameters");
        return ESP_FAIL;
    }

    // Проверка значений (опционально)
    if (low_value >= up_value) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Low value must be less than up value");
        return ESP_FAIL;
    }

    nvs_handle_t my_handle;
    esp_err_t err;

    // 1. Открываем раздел "nvs" и пространство имен "storage" на запись/чтение
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE("TAG", "Error opening NVS");
        return 1;
    }

    err = nvs_set_i32(my_handle, THRESHOLD_LOW_NVS_NAME, low_value);
    ESP_ERROR_CHECK(err);

    err = nvs_set_i32(my_handle, THRESHOLD_UP_NVS_NAME, up_value);
    ESP_ERROR_CHECK(err);

    // 4. Сохраняем изменения во flash
    err = nvs_commit(my_handle);
    ESP_ERROR_CHECK(err);

    // 5. Закрываем хендл
    nvs_close(my_handle);

    // Формирование успешного ответа
    snprintf(response, sizeof(response), "{\"success\":true,\"low\":%d,\"up\":%d}", low_value, up_value);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));

    return ESP_OK;
}

static esp_err_t set_thresholds_handler(httpd_req_t *req) {
    char response[100];
    char *content = NULL;
    size_t content_len = req->content_len;

    // Проверка длины содержимого
    if (content_len > 512) { // Ограничение размера для безопасности
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too large");
        return ESP_FAIL;
    }

    // Выделение памяти под содержимое запроса
    content = malloc(content_len + 1);
    if (!content) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }

    // Чтение тела запроса
    int ret = httpd_req_recv(req, content, content_len);
    if (ret <= 0) {
        free(content);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to receive data");
        return ESP_FAIL;
    }
    content[content_len] = '\0'; // Null-terminator

    // Парсинг JSON
    cJSON *json = cJSON_Parse(content);
    free(content);

    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    // Извлечение параметров low и up
    cJSON *low_item = cJSON_GetObjectItem(json, "low");
    cJSON *up_item = cJSON_GetObjectItem(json, "up");

    int low_value = 0;
    int up_value = 0;
    bool valid = true;

    if (cJSON_IsNumber(low_item)) {
        low_value = low_item->valueint;
    } else {
        valid = false;
    }

    if (cJSON_IsNumber(up_item)) {
        up_value = up_item->valueint;
    } else {
        valid = false;
    }

    cJSON_Delete(json);

    if (!valid) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid 'low' or 'up' parameters");
        return ESP_FAIL;
    }

    // Проверка значений (опционально)
    if (low_value >= up_value) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Low value must be less than up value");
        return ESP_FAIL;
    }

    atomic_store(&g_threshold_low, low_value);
    atomic_store(&g_threshold_up, up_value);

    // Формирование успешного ответа
    snprintf(response, sizeof(response), "{\"success\":true,\"low\":%d,\"up\":%d}", low_value, up_value);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));

    return ESP_OK;
}

void wifi_init_softap(void) {
    // Инициализация сетевого интерфейса
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    // Инициализация Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Настройка точки доступа
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

    ip_info.ip.addr = ipaddr_addr(CONFIG_AP_IP);
    ip_info.gw.addr = ipaddr_addr(CONFIG_AP_GATEWAY);
    ip_info.netmask.addr = ipaddr_addr(CONFIG_AP_NETMASK);

    // Останавливаем DHCP сервер, который автоматически запустился
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));

    // Устанавливаем наши настройки IP
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));

    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "📶 Wifi name (SSID): %s", CONFIG_AP_WIFI_SSID);
    ESP_LOGI(TAG, "🔑 Password: %s", strlen(CONFIG_AP_WIFI_PASS) ? CONFIG_AP_WIFI_PASS : "Open network");
    ESP_LOGI(TAG, "🌐 Webinterface IP address: %s:%s", CONFIG_AP_IP, CONFIG_WEBINTERFACE_PORT);
    ESP_LOGI(TAG, "=========================================");
}

static void disablePump(void) {
    gpio_set_level(CONFIG_PUMP_PIN, false);
}

static void enablePump(void) {
    gpio_set_level(CONFIG_PUMP_PIN, true);
}

static void vPumpControllTask(void *pvParameters) {
    while (1) {
        int current_pressure = atomic_load(&g_current_pressure);
        int low_treshhold = atomic_load(&g_threshold_low);
        int up_treshhold = atomic_load(&g_threshold_up);

        if (current_pressure < low_treshhold) {
            enablePump();
        } else if (current_pressure >= up_treshhold) {
            disablePump();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void vReadSensorTask(void *pvParameters) {
    while (1) {
        int pressure = adc_read_voltage(SENSOR_ADC_CHAN);
        atomic_store(&g_current_pressure, pressure);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void vHttpServerTask(void *pvParameters) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CONFIG_WEBINTERFACE_PORT;
    config.max_uri_handlers = 10;
    config.stack_size = 8192;

    // Запуск HTTP сервера
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "🚀 HTTP сервер запущен на порту %d", config.server_port);

        // Регистрация URI обработчиков
        httpd_uri_t root = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root);

        httpd_uri_t pressure = {
            .uri       = "/pressure",
            .method    = HTTP_GET,
            .handler   = current_pressure_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &pressure);

        httpd_uri_t set_thresholds = {
            .uri       = "/thresholds",
            .method    = HTTP_POST,
            .handler   = set_thresholds_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &set_thresholds);

        httpd_uri_t save_thresholds = {
            .uri       = "/persist_thresholds",
            .method    = HTTP_POST,
            .handler   = save_thresholds_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &save_thresholds);
    } else {
        ESP_LOGE(TAG, "❌ Ошибка запуска HTTP сервера");
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nvs_handle_t my_handle;
    esp_err_t err;
    // 1. Открываем раздел "nvs" и пространство имен "storage" на запись/чтение
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE("TAG", "Error opening NVS");
    }

    int32_t threshold_up, threshold_low = 0;

    err = nvs_get_i32(my_handle, THRESHOLD_LOW_NVS_NAME, &threshold_low);
    if (err == ESP_OK) {
        atomic_store(&g_threshold_low, threshold_low);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        atomic_store(&g_threshold_low, 100);
    } else {
        ESP_ERROR_CHECK(err);
    }

    err = nvs_get_i32(my_handle, THRESHOLD_UP_NVS_NAME, &threshold_up);
    if (err == ESP_OK) {
        atomic_store(&g_threshold_up, threshold_up);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        atomic_store(&g_threshold_up, 300);
    } else {
        ESP_ERROR_CHECK(err);
    }


    adc_init();
    pump_init();

    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "ESP32 Точка Доступа + HTTP Сервер");
    ESP_LOGI(TAG, "=========================================");


    wifi_init_softap();

    vTaskDelay(pdMS_TO_TICKS(1000));

    xTaskCreate(vHttpServerTask, "http_server", 8192, NULL, 5, NULL);
    xTaskCreate(vPumpControllTask, "pump_controll", 8192, NULL, 5, NULL);
    xTaskCreate(vReadSensorTask, "read_sensor", 8192, NULL, 5, NULL);

    ESP_LOGI(TAG, "✅ Система готова к работе");
    ESP_LOGI(TAG, "📱 Подключитесь к Wi-Fi: %s", CONFIG_AP_WIFI_SSID);
    ESP_LOGI(TAG, "🌐 Откройте браузер: http://192.168.4.1");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));

        wifi_sta_list_t sta_list;
        memset(&sta_list, 0, sizeof(sta_list));
        esp_wifi_ap_get_sta_list(&sta_list);

        ESP_LOGI(TAG, "📊 Подключено клиентов: %d", sta_list.num);
    }
}

