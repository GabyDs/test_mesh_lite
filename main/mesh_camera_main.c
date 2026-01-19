/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"

#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "cJSON.h"

#include "esp_bridge.h"
#include "esp_mesh_lite.h"

static const char *TAG = "mesh_camera";

// Node role definitions
#ifdef CONFIG_NODE_ROLE_ROOT
#define IS_ROOT_NODE true
#else
#define IS_ROOT_NODE false
#endif

// PIR Sensor GPIO (only for leaf/camera nodes)
#define PIR_SENSOR_GPIO         GPIO_NUM_12
#define ESP_INTR_FLAG_DEFAULT   0
#define PIR_DEBOUNCE_MS         3000  // Debounce time in milliseconds (ignore events within this period)

// Message types for mesh communication
#define MSG_TYPE_PIR_ALERT    "pir_alert"

// Queue for PIR events
static QueueHandle_t pir_evt_queue = NULL;

// Last PIR trigger timestamp for debounce
static uint32_t last_pir_trigger_time = 0;

#if IS_ROOT_NODE
/**
 * @brief Handler for PIR alert messages received from camera nodes (ROOT only)
 */
static cJSON* pir_alert_process(cJSON *payload, uint32_t seq)
{
    cJSON *src_mac = cJSON_GetObjectItem(payload, "src_mac");
    
    if (src_mac && cJSON_IsString(src_mac)) {
        ESP_LOGW(TAG, "ALERT: Motion detected from camera [%s]", src_mac->valuestring);
    } else {
        ESP_LOGW(TAG, "ALERT: Motion detected from unknown camera");
    }
    
    // Return NULL - no response needed
    return NULL;
}

// Message action table for ROOT node
static const esp_mesh_lite_msg_action_t root_msg_action[] = {
    {MSG_TYPE_PIR_ALERT, NULL, pir_alert_process},
    {NULL, NULL, NULL} // End marker
};
#endif

/**
 * @brief PIR sensor interrupt handler
 */
static void IRAM_ATTR pir_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(pir_evt_queue, &gpio_num, NULL);
}

#if !IS_ROOT_NODE
/**
 * @brief Resend function for PIR alert
 */
static esp_err_t pir_alert_resend(const char* payload)
{
    return esp_mesh_lite_send_msg_to_root(payload);
}

/**
 * @brief Send PIR alert to root node
 */
static void send_pir_alert_to_root(void)
{
    uint8_t sta_mac[6] = {0};
    char mac_str[18];
    
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    snprintf(mac_str, sizeof(mac_str), MACSTR, MAC2STR(sta_mac));
    
    // Create JSON payload
    cJSON *req_payload = cJSON_CreateObject();
    cJSON_AddStringToObject(req_payload, "src_mac", mac_str);
    cJSON_AddNumberToObject(req_payload, "timestamp", esp_log_timestamp());
    
    // Configure and send message using mesh_lite API
    esp_mesh_lite_msg_config_t config = {
        .json_msg = {
            .send_msg = MSG_TYPE_PIR_ALERT,
            .expect_msg = NULL,  // No response expected
            .max_retry = 3,
            .retry_interval = 1000,
            .req_payload = req_payload,
            .resend = pir_alert_resend,
            .send_fail = NULL,
        }
    };
    
    esp_err_t ret = esp_mesh_lite_send_msg(ESP_MESH_LITE_JSON_MSG, &config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "PIR alert sent to root [%s]", mac_str);
    } else {
        ESP_LOGE(TAG, "Failed to send PIR alert: %s", esp_err_to_name(ret));
    }
    
    cJSON_Delete(req_payload);
}
#endif

/**
 * @brief Task to handle PIR sensor events with debounce
 */
static void pir_task(void *arg)
{
    uint32_t gpio_num;
    for (;;) {
        if (xQueueReceive(pir_evt_queue, &gpio_num, portMAX_DELAY)) {
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            // Check debounce: ignore if within debounce period
            if ((current_time - last_pir_trigger_time) < PIR_DEBOUNCE_MS) {
                ESP_LOGD(TAG, "PIR event ignored (debounce). Time since last: %"PRIu32" ms", 
                         current_time - last_pir_trigger_time);
                continue;
            }
            
            // Update last trigger time
            last_pir_trigger_time = current_time;
            
            ESP_LOGI(TAG, "PIR SENSOR TRIGGERED! Motion detected on GPIO %"PRIu32, gpio_num);
#if !IS_ROOT_NODE
            send_pir_alert_to_root();
#endif
        }
    }
}

/**
 * @brief Initialize PIR sensor GPIO (only for camera/leaf nodes)
 */
static void pir_sensor_init(void)
{
    ESP_LOGI(TAG, "Initializing PIR sensor on GPIO %d", PIR_SENSOR_GPIO);

    // Configure GPIO
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,     // Interrupt on rising edge
        .mode = GPIO_MODE_INPUT,             // Input mode
        .pin_bit_mask = (1ULL << PIR_SENSOR_GPIO),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);

    // Create queue for PIR events
    pir_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // Create task to handle PIR events
    xTaskCreate(pir_task, "pir_task", 2048, NULL, 10, NULL);

    // Install GPIO ISR service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // Hook ISR handler for specific GPIO
    gpio_isr_handler_add(PIR_SENSOR_GPIO, pir_isr_handler, (void *)PIR_SENSOR_GPIO);

    ESP_LOGI(TAG, "PIR sensor initialized - waiting for motion...");
}

/**
 * @brief Timed printing system information
 */
static void print_system_info_timercb(TimerHandle_t timer)
{
    uint8_t primary                 = 0;
    uint8_t sta_mac[6]              = {0};
    wifi_ap_record_t ap_info        = {0};
    wifi_second_chan_t second       = 0;
    wifi_sta_list_t wifi_sta_list   = {0x0};

    esp_wifi_sta_get_ap_info(&ap_info);
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);

    ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
#if IS_ROOT_NODE
    ESP_LOGI(TAG, "║       MESH CAMERA - ROOT NODE          ║");
#else
    ESP_LOGI(TAG, "║       MESH CAMERA - CAMERA NODE        ║");
#endif
    ESP_LOGI(TAG, "╠════════════════════════════════════════╣");
    ESP_LOGI(TAG, "║ Channel: %-3d | Layer: %-2d               ║", primary, esp_mesh_lite_get_level());
    ESP_LOGI(TAG, "║ MAC: " MACSTR "            ║", MAC2STR(sta_mac));
    
    if (esp_mesh_lite_get_level() > 0) {
        ESP_LOGI(TAG, "║ Parent: " MACSTR " RSSI: %-4d   ║", 
                 MAC2STR(ap_info.bssid), (ap_info.rssi != 0 ? ap_info.rssi : -120));
    } else {
        ESP_LOGI(TAG, "║ Status: Connecting...                  ║");
    }
    
    ESP_LOGI(TAG, "║ Free heap: %-8"PRIu32" bytes            ║", esp_get_free_heap_size());

#if CONFIG_MESH_LITE_NODE_INFO_REPORT
    ESP_LOGI(TAG, "║ Total mesh nodes: %-3"PRIu32"                  ║", esp_mesh_lite_get_mesh_node_number());
#endif

    ESP_LOGI(TAG, "╠════════════════════════════════════════╣");
    if (wifi_sta_list.num > 0) {
        ESP_LOGI(TAG, "║ Child nodes connected: %-2d              ║", wifi_sta_list.num);
        for (int i = 0; i < wifi_sta_list.num; i++) {
            ESP_LOGI(TAG, "║   └─ " MACSTR "           ║", MAC2STR(wifi_sta_list.sta[i].mac));
        }
    } else {
        ESP_LOGI(TAG, "║ No child nodes connected               ║");
    }
    ESP_LOGI(TAG, "╚════════════════════════════════════════╝");
}

static void ip_event_sta_got_ip_handler(void *arg, esp_event_base_t event_base,
                                        int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
}

static esp_err_t esp_storage_init(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    return ret;
}

static void wifi_init(void)
{
    // Station
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ROUTER_SSID,
            .password = CONFIG_ROUTER_PASSWORD,
        },
    };
    esp_bridge_wifi_set_config(WIFI_IF_STA, &wifi_config);

    // Softap
    wifi_config_t wifi_softap_config = {
        .ap = {
            .ssid = CONFIG_BRIDGE_SOFTAP_SSID,
            .password = CONFIG_BRIDGE_SOFTAP_PASSWORD,
        },
    };
    esp_bridge_wifi_set_config(WIFI_IF_AP, &wifi_softap_config);
}

void app_wifi_set_softap_info(void)
{
    char softap_ssid[33];
    char softap_psw[64];
    uint8_t softap_mac[6];
    size_t ssid_size = sizeof(softap_ssid);
    size_t psw_size = sizeof(softap_psw);
    esp_wifi_get_mac(WIFI_IF_AP, softap_mac);
    memset(softap_ssid, 0x0, sizeof(softap_ssid));
    memset(softap_psw, 0x0, sizeof(softap_psw));

    if (esp_mesh_lite_get_softap_ssid_from_nvs(softap_ssid, &ssid_size) == ESP_OK) {
        ESP_LOGI(TAG, "Get ssid from nvs: %s", softap_ssid);
    } else {
#ifdef CONFIG_BRIDGE_SOFTAP_SSID_END_WITH_THE_MAC
        snprintf(softap_ssid, sizeof(softap_ssid), "%.25s_%02x%02x%02x", CONFIG_BRIDGE_SOFTAP_SSID, softap_mac[3], softap_mac[4], softap_mac[5]);
#else
        snprintf(softap_ssid, sizeof(softap_ssid), "%.32s", CONFIG_BRIDGE_SOFTAP_SSID);
#endif
        ESP_LOGI(TAG, "Set ssid: %s", softap_ssid);
    }

    if (esp_mesh_lite_get_softap_psw_from_nvs(softap_psw, &psw_size) == ESP_OK) {
        ESP_LOGI(TAG, "Get password from nvs: [OK]");
    } else {
        strlcpy(softap_psw, CONFIG_BRIDGE_SOFTAP_PASSWORD, sizeof(softap_psw));
        ESP_LOGI(TAG, "Set default password");
    }

    esp_mesh_lite_set_softap_info(softap_ssid, softap_psw);
}

void app_main()
{
    esp_log_level_set("*", ESP_LOG_INFO);

    esp_storage_init();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_bridge_create_all_netif();

    wifi_init();

    esp_mesh_lite_config_t mesh_lite_config = ESP_MESH_LITE_DEFAULT_INIT();
    esp_mesh_lite_init(&mesh_lite_config);

#if IS_ROOT_NODE
    // Register message handler for PIR alerts from camera nodes BEFORE starting mesh
    esp_mesh_lite_msg_action_list_register(root_msg_action);
#endif

    app_wifi_set_softap_info();

    esp_mesh_lite_start();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_sta_got_ip_handler, NULL, NULL));

#if IS_ROOT_NODE
    ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  MESH CAMERA SYSTEM - ROOT NODE        ║");
    ESP_LOGI(TAG, "║  Waiting for camera nodes to connect...║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════╝");
#else
    ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  MESH CAMERA SYSTEM - CAMERA NODE      ║");
    ESP_LOGI(TAG, "║  Connecting to mesh network...         ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════╝");

    // Initialize PIR sensor for camera nodes only
    pir_sensor_init();
#endif

    TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_PERIOD_MS,
                                       true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);
}

