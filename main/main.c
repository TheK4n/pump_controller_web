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

// ==================== НАСТРОЙКИ ТОЧКИ ДОСТУПА ====================
#define AP_SSID             "ESP32_Hotspot"        // Имя Wi-Fi сети
#define AP_PASS             "12345678"             // Пароль (минимум 8 символов)
#define AP_MAX_CONN         4                      // Максимум клиентов
#define AP_CHANNEL          6                      // Wi-Fi канал


#define ADC_CHAN0          ADC_CHANNEL_4
#define ADC_CHAN1          ADC_CHANNEL_5
#define ADC_ATTEN_DB       ADC_ATTEN_DB_12

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle;
static bool is_calibrated = false;


// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ====================
static const char *TAG = "ESP32_AP_SERVER";

atomic_int low_treshhold = 0;
atomic_int up_treshhold = 0;

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

int adc_read_voltage(uint8_t channel)
{
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


// ==================== ОБРАБОТЧИКИ HTTP ЗАПРОСОВ ====================

// Обработчик главной страницы
static esp_err_t root_get_handler(httpd_req_t *req)
{
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

// Управление GPIO
static esp_err_t gpio_handler(httpd_req_t *req)
{
    char param[10];
    char state_str[10];

    // Получаем параметр state из URL
    if (httpd_req_get_url_query_str(req, param, sizeof(param)) == ESP_OK) {
        if (httpd_query_key_value(param, "state", state_str, sizeof(state_str)) == ESP_OK) {
            int state = atoi(state_str);

            // Настройка GPIO2 (встроенный LED на многих ESP32)
            gpio_config_t io_conf = {
                .pin_bit_mask = (1ULL << 2),
                .mode = GPIO_MODE_OUTPUT,
                .intr_type = GPIO_INTR_DISABLE,
                .pull_down_en = 0,
                .pull_up_en = 0,
            };
            gpio_config(&io_conf);
            gpio_set_level(2, state);

            char response[100];
            snprintf(response, sizeof(response), "{\"status\":\"%s\"}", state ? "on" : "off");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, response, strlen(response));
            return ESP_OK;
        }
    }

    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing state parameter");
    return ESP_FAIL;
}

// Имитация датчика
static esp_err_t sensor_handler(httpd_req_t *req) {
    int volt0 = adc_read_voltage(0);
    int sensor_value = volt0;
    char response[100];
    snprintf(response, sizeof(response),
             "{\"value\":%d,\"message\":\"Случайное значение датчика\"}",
             sensor_value);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

// Статистика системы
static esp_err_t stats_handler(httpd_req_t *req)
{
    char response[200];
    snprintf(response, sizeof(response),
             "{\"free_heap\":%lu,\"uptime\":%d}",
             esp_get_free_heap_size(),
             (int)(xTaskGetTickCount() * portTICK_PERIOD_MS / 1000));

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

// ==================== ИНИЦИАЛИЗАЦИЯ ТОЧКИ ДОСТУПА ====================
void wifi_init_softap(void)
{
    // Инициализация сетевого интерфейса
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    // Инициализация Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Настройка точки доступа
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .password = AP_PASS,
            .max_connection = AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .channel = AP_CHANNEL,
        },
    };

    // Если пароль не задан - открытая сеть
    if (strlen(AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "📡 Точка доступа запущена");
    ESP_LOGI(TAG, "📶 Имя сети (SSID): %s", AP_SSID);
    ESP_LOGI(TAG, "🔑 Пароль: %s", strlen(AP_PASS) ? AP_PASS : "ОТКРЫТАЯ СЕТЬ");
    ESP_LOGI(TAG, "🌐 IP адрес сервера: 192.168.4.1");
    ESP_LOGI(TAG, "=========================================");
}

// void setup_adc() {
//     // Настройка разрешения (9-12 бит)
//     adc1_config_width(ADC_WIDTH_BIT_12);
//
//     // Настройка аттенюации (диапазон входного напряжения)
//     adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
// }
//
// int read_mid_pressure(void) {
//     int N = 5;
//     int pause = 20;
//
//     int sum = 0;
//     for (int i = 0; i < N; i++) {
//         sum += adc1_get_raw(ADC1_CHANNEL_0);
//         vTaskDelay(pdMS_TO_TICKS(pause));
//     }
//
//     return sum /= N;
// }

// static void pressure_control_task(void *pvParameters) {
//     gpio_config_t io_conf = {
//         .pin_bit_mask = (1ULL << 2),
//         .mode = GPIO_MODE_OUTPUT,
//         .intr_type = GPIO_INTR_DISABLE,
//         .pull_down_en = 1,
//         .pull_up_en = 0,
//     };
//     gpio_config(&io_conf);
//
//     while(1) {
//         int low_treshhold_pressure = atomic_load(&low_treshhold);
//         int up_treshhold_pressure = atomic_load(&up_treshhold);
//
//         int pressure = read_mid_pressure();
//
//         if (pressure < low_treshhold_pressure) {
//             gpio_set_level(2, 1);
//         } else if (pressure >= up_treshhold_pressure) {
//             gpio_set_level(2, 0);
//         }
//
//         vTaskDelay(pdMS_TO_TICKS(400));
//     }
// }

// ==================== ЗАДАЧА HTTP СЕРВЕРА ====================
static void http_server_task(void *pvParameters)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
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

        httpd_uri_t gpio = {
            .uri       = "/gpio",
            .method    = HTTP_GET,
            .handler   = gpio_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &gpio);

        httpd_uri_t sensor = {
            .uri       = "/sensor",
            .method    = HTTP_GET,
            .handler   = sensor_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &sensor);

        httpd_uri_t stats = {
            .uri       = "/stats",
            .method    = HTTP_GET,
            .handler   = stats_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &stats);

        ESP_LOGI(TAG, "📝 Зарегистрированы обработчики:");
        ESP_LOGI(TAG, "   - GET /       (главная страница)");
        ESP_LOGI(TAG, "   - GET /gpio   (управление LED)");
        ESP_LOGI(TAG, "   - GET /sensor (чтение датчика)");
        ESP_LOGI(TAG, "   - GET /stats  (статистика)");
    } else {
        ESP_LOGE(TAG, "❌ Ошибка запуска HTTP сервера");
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ==================== ГЛАВНАЯ ФУНКЦИЯ ====================
void app_main(void)
{
    // Инициализация NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    adc_init();

    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "ESP32 Точка Доступа + HTTP Сервер");
    ESP_LOGI(TAG, "=========================================");


    // Инициализация точки доступа
    wifi_init_softap();

    // Небольшая задержка для стабилизации
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Создание задачи HTTP сервера
    xTaskCreate(http_server_task, "http_server", 8192, NULL, 5, NULL);

    // Демонстрационная задача для имитации датчика
    ESP_LOGI(TAG, "✅ Система готова к работе");
    ESP_LOGI(TAG, "📱 Подключитесь к Wi-Fi: %s", AP_SSID);
    ESP_LOGI(TAG, "🌐 Откройте браузер: http://192.168.4.1");

    while (1) {
        // Вывод информации о подключенных клиентах каждые 10 секунд
        vTaskDelay(pdMS_TO_TICKS(10000));

        wifi_sta_list_t sta_list;
        memset(&sta_list, 0, sizeof(sta_list));
        esp_wifi_ap_get_sta_list(&sta_list);

        ESP_LOGI(TAG, "📊 Подключено клиентов: %d", sta_list.num);
        ESP_LOGI(TAG, "💾 Свободно памяти: %d байт", esp_get_free_heap_size());
    }
}

