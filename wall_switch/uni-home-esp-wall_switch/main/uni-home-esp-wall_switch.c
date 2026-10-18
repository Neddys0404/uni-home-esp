#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "esp_mac.h"

#include "sdkconfig.h"

// Configuration - Change these to your actual credentials
#define WIFI_SSID      CONFIG_WIFI_SSID
#define WIFI_PASS      CONFIG_WIFI_PASS
#define MQTT_BROKER    "mqtt://192.168.0.252" // Replace with your broker IP

// GPIO Definitions for 74HC595
#define GPIO_SER       8
#define GPIO_SRCLK     21
#define GPIO_RCLK      20

static const char *TAG = "WALL_SWITCH";

// Global state
static uint8_t current_shift_state = 0;
static esp_mqtt_client_handle_t mqtt_client = NULL;
static char mac_str[13];
static char cmd_topic[64];
static char status_topic[64];

// Event group for synchronization
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

/**
 * @brief Converts a byte to an 8-character binary string
 */
void uint8_to_binary_string(uint8_t val, char *buf) {
    for (int i = 0; i < 8; i++) {
        buf[i] = (val & (1 << (7 - i))) ? '1' : '0';
    }
    buf[8] = '\0';
}

/**
 * @brief Converts an 8-character binary string to a byte
 */
uint8_t binary_string_to_uint8(const char *buf) {
    uint8_t val = 0;
    for (int i = 0; i < 8; i++) {
        if (buf[i] == '1') {
            val |= (1 << (7 - i));
        }
    }
    return val;
}

/**
 * @brief Bit-bangs the 74HC595 shift register
 */
void shift_register_write(uint8_t data) {
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(GPIO_SER, (data >> i) & 0x01);
        esp_rom_delay_us(1);
        gpio_set_level(GPIO_SRCLK, 0);
        esp_rom_delay_us(1);
        gpio_set_level(GPIO_SRCLK, 1);
        esp_rom_delay_us(1);
    }
    // Latch pulse to update output pins
    gpio_set_level(GPIO_RCLK, 0);
    esp_rom_delay_us(1);
    gpio_set_level(GPIO_RCLK, 1);
    esp_rom_delay_us(1);
}

/**
 * @brief Updates the shift register and persists state to NVS
 */
void update_state(uint8_t new_state) {
    current_shift_state = new_state;
    shift_register_write(current_shift_state);

    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_u8(my_handle, "shift_state", new_state);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
    ESP_LOGI(TAG, "State updated to: 0x%02X", new_state);
}

/**
 * @brief MQTT Event Handler
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            // Subscribe to command topic
            esp_mqtt_client_subscribe(mqtt_client, cmd_topic, 1);

            // Publish online status: "online, 00000000"
            char bin_str[9];
            char msg[32];
            uint8_to_binary_string(current_shift_state, bin_str);
            snprintf(msg, sizeof(msg), "online, %s", bin_str);
            esp_mqtt_client_publish(mqtt_client, status_topic, msg, 0, 1, 0);
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT Data Received on topic: %.*s", event->topic_len, event->topic);
            // Check if data is the command string (8 bits)
            if (event->data_len >= 8) {
                uint8_t received_val = binary_string_to_uint8((const char *)event->data);
                update_state(received_val);
            }
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT Error");
            break;

        default:
            break;
    }
}

/**
 * @brief Heartbeat task to publish status every 5 seconds
 */
void heartbeat_task(void *pvParameters) {
    char msg[32];
    char bin_str[9];

    while (1) {
        if (xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) {
            uint8_to_binary_string(current_shift_state, bin_str);
            snprintf(msg, sizeof(msg), "online, %s", bin_str);
            esp_mqtt_client_publish(mqtt_client, status_topic, msg, 0, 1, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/**
 * @brief Wi-Fi Event Handler
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                 int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected from Wi-Fi. Retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Connected to Wi-Fi");
    }
}

/**
 * @brief Initializes Wi-Fi station mode
 */
void wifi_init(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t sta_cfg = {0};
    strncpy((char *)sta_cfg.sta.ssid, WIFI_SSID, sizeof(sta_cfg.sta.ssid) - 1);
    strncpy((char *)sta_cfg.sta.password, WIFI_PASS, sizeof(sta_cfg.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void) {
    // 1. Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Get MAC Address and prepare MQTT topics
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    snprintf(mac_str, sizeof(mac_str), "%02X%02X%02X%02X%02X%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    snprintf(cmd_topic, sizeof(cmd_topic), "/indoor/switches/%s", mac_str);
    snprintf(status_topic, sizeof(status_topic), "/indoor/switches/%s/status", mac_str);

    // 3. Initialize GPIOs
    gpio_reset_pin(GPIO_SER);
    gpio_set_direction(GPIO_SER, GPIO_MODE_OUTPUT);
    gpio_reset_pin(GPIO_SRCLK);
    gpio_set_direction(GPIO_SRCLK, GPIO_MODE_OUTPUT);
    gpio_reset_pin(GPIO_RCLK);
    gpio_set_direction(GPIO_RCLK, GPIO_MODE_OUTPUT);

    // 4. Restore state from NVS
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
        nvs_get_u8(my_handle, "shift_state", &current_shift_state);
        nvs_close(my_handle);
    } else {
        current_shift_state = 0;
    }
    shift_register_write(current_shift_state);

    // 5. Initialize Wi-Fi and Event Group
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_event_group = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    wifi_init();

    // 6. Prepare MQTT Configuration with Last Will and Testament (LWT)
    char bin_str[9];
    char lwt_msg[32];
    uint8_to_binary_string(current_shift_state, bin_str);
    snprintf(lwt_msg, sizeof(lwt_msg), "offline, %s", bin_str);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER,
        .session.last_will = {
            .topic = status_topic,
            .msg = lwt_msg,
            .qos = 1,
        },
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    // 7. Start Heartbeat Task
    xTaskCreate(heartbeat_task, "heartbeat_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "System initialized. MAC: %s", mac_str);
}