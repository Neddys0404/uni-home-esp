
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "esp_mac.h"

#include "sdkconfig.h"

// Configuration
#define WIFI_SSID       CONFIG_WIFI_SSID
#define WIFI_PASS       CONFIG_WIFI_PASS
#define MQTT_USER       CONFIG_MQTT_USER
#define MQTT_PASS       CONFIG_MQTT_PASS
#define MQTT_BASE_TOPIC CONFIG_MQTT_BASE_TOPIC
#define MQTT_BROKER     "mqtt://192.168.0.52"

// 74HC595 GPIO definitions
#define GPIO_SER        8
#define GPIO_SRCLK      21
#define GPIO_RCLK       20

// Configurable number of switches
#define SWITCHES_NUM    CONFIG_SWITCHES_NUM

#if SWITCHES_NUM < 1 || SWITCHES_NUM > 4
#error "CONFIG_SWITCHES_NUM must be between 1 and 8"
#endif

// Mask of enabled switch outputs
#define SWITCH_MASK     ((uint8_t)((1U << SWITCHES_NUM) - 1U))

static const char *TAG = "WALL_SWITCH";

// Global state
static uint8_t current_shift_state = 0;
static esp_mqtt_client_handle_t mqtt_client = NULL;

static char mac_str[13];
static char cmd_topic[64];
static char status_topic[64];

// Event group
static EventGroupHandle_t wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define MQTT_CONNECTED_BIT BIT1


/**
 * @brief Convert switch state to a binary string.
 *
 * Only SWITCHES_NUM bits are included.
 * Rightmost bit corresponds to switch 1 (Q0).
 *
 * Example:
 * SWITCHES_NUM = 3
 * val = 0x05
 * result = "101"
 *
 * Buffer must hold at least SWITCHES_NUM + 1 bytes.
 */
static void uint8_to_binary_string(uint8_t val, char *buf)
{
    val &= SWITCH_MASK;

    for (int i = 0; i < SWITCHES_NUM; i++) {
        int bit_index = SWITCHES_NUM - 1 - i;

        buf[i] = (val & (1U << bit_index)) ? '1' : '0';
    }

    buf[SWITCHES_NUM] = '\0';
}


/**
 * @brief Convert an MQTT binary payload to a byte.
 *
 * Returns false if the payload length is incorrect
 * or contains characters other than '0' and '1'. 
 *
 * MQTT payloads are not guaranteed to be null-terminated.
 */
static bool binary_string_to_uint8(
    const char *buf,
    size_t len,
    uint8_t *value
)
{
    if (buf == NULL || value == NULL) {
        return false;
    }

    if (len != SWITCHES_NUM) {
        return false;
    }

    uint8_t result = 0;

    for (size_t i = 0; i < len; i++) {
        if (buf[i] != '0' && buf[i] != '1') {
            return false;
        }

        result = (uint8_t)(
            (result << 1) | (buf[i] == '1')
        );
    }

    *value = result & SWITCH_MASK;

    return true;
}


/**
 * @brief Bit-bang data to the 74HC595.
 *
 * Unused outputs are forced LOW.
 */
static void shift_register_write(uint8_t data)
{
    data &= SWITCH_MASK;

    gpio_set_level(GPIO_SRCLK, 0);
    gpio_set_level(GPIO_RCLK, 0);

    for (int i = 7; i >= 0; i--) {
        gpio_set_level(GPIO_SER, (data >> i) & 0x01);
        esp_rom_delay_us(1);

        gpio_set_level(GPIO_SRCLK, 1);
        esp_rom_delay_us(1);

        gpio_set_level(GPIO_SRCLK, 0);
        esp_rom_delay_us(1);
    }

    // Latch the 8-bit output
    gpio_set_level(GPIO_RCLK, 1);
    esp_rom_delay_us(1);
    gpio_set_level(GPIO_RCLK, 0);
}


/**
 * @brief Publish the current switch status.
 *
 * Example:
 * SWITCHES_NUM = 3
 * "online, 101"
 */
static void publish_status(void)
{
    if (mqtt_client == NULL || wifi_event_group == NULL) {
        return;
    }

    EventBits_t bits = xEventGroupGetBits(wifi_event_group);

    if (!(bits & MQTT_CONNECTED_BIT)) {
        return;
    }

    char bin_str[SWITCHES_NUM + 1];
    char msg[32];

    uint8_to_binary_string(current_shift_state, bin_str);

    snprintf(
        msg,
        sizeof(msg),
        "online, %s",
        bin_str
    );

    esp_mqtt_client_publish(
        mqtt_client,
        status_topic,
        msg,
        0,
        1,
        1
    );
}


/**
 * @brief Update 74HC595 and persist state to NVS.
 */
static void update_state(uint8_t new_state)
{
    new_state &= SWITCH_MASK;

    if (current_shift_state == new_state) {
        publish_status();
        return;
    }

    current_shift_state = new_state;

    shift_register_write(current_shift_state);

    nvs_handle_t handle;

    esp_err_t err = nvs_open(
        "storage",
        NVS_READWRITE,
        &handle
    );

    if (err == ESP_OK) {
        err = nvs_set_u8(
            handle,
            "shift_state",
            current_shift_state
        );

        if (err == ESP_OK) {
            err = nvs_commit(handle);
        }

        nvs_close(handle);
    }

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to save state: %s",
            esp_err_to_name(err)
        );
    }

    char bin_str[SWITCHES_NUM + 1];
    uint8_to_binary_string(current_shift_state, bin_str);

    ESP_LOGI(
        TAG,
        "State updated: %s (0x%02X)",
        bin_str,
        current_shift_state
    );

    // Publish immediately after changing outputs
    publish_status();
}


/**
 * @brief MQTT Event Handler
 */
static void mqtt_event_handler(
    void *handler_args,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data
)
{
    esp_mqtt_event_handle_t event =
        (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {

        case MQTT_EVENT_CONNECTED:
        {
            ESP_LOGI(TAG, "MQTT Connected");

            xEventGroupSetBits(
                wifi_event_group,
                MQTT_CONNECTED_BIT
            );

            esp_mqtt_client_subscribe(
                mqtt_client,
                cmd_topic,
                1
            );

            publish_status();
            break;
        }

        case MQTT_EVENT_DISCONNECTED:
        {
            ESP_LOGW(TAG, "MQTT Disconnected");

            xEventGroupClearBits(
                wifi_event_group,
                MQTT_CONNECTED_BIT
            );

            break;
        }

        case MQTT_EVENT_DATA:
        {
            // Ignore messages to unrelated topics
            size_t expected_topic_len = strlen(cmd_topic);

            if (event->topic == NULL ||
                event->topic_len != expected_topic_len ||
                memcmp(
                    event->topic,
                    cmd_topic,
                    expected_topic_len
                ) != 0) {
                break;
            }

            // Only accept complete command messages.
            // A command is at most 8 bytes.
            if (event->current_data_offset != 0 ||
                event->total_data_len != SWITCHES_NUM ||
                event->data_len != SWITCHES_NUM ||
                event->data == NULL) {

                ESP_LOGW(
                    TAG,
                    "Invalid command length. Expected %d bits",
                    SWITCHES_NUM
                );

                break;
            }

            uint8_t received_val = 0;

            bool valid = binary_string_to_uint8(
                event->data,
                (size_t)event->data_len,
                &received_val
            );

            if (!valid) {
                ESP_LOGW(
                    TAG,
                    "Invalid command payload"
                );
                break;
            }

            ESP_LOGI(
                TAG,
                "Valid command: %.*s",
                event->data_len,
                event->data
            );

            update_state(received_val);

            break;
        }

        case MQTT_EVENT_ERROR:
        {
            ESP_LOGE(TAG, "MQTT Error");
            break;
        }

        default:
            break;
    }
}


/**
 * @brief Publish heartbeat every 5 seconds.
 */
static void heartbeat_task(void *pvParameters)
{
    while (1) {
        publish_status();

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


/**
 * @brief Wi-Fi Event Handler
 */
static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data
)
{
    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_START) {

        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {

        wifi_event_sta_disconnected_t *event =
            (wifi_event_sta_disconnected_t *)event_data;

        uint8_t reason = (event != NULL) ?
            event->reason : 0;

        xEventGroupClearBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT
        );

        ESP_LOGW(
            TAG,
            "Wi-Fi disconnected, reason: %d",
            reason
        );

        esp_wifi_connect();

    } else if (event_base == IP_EVENT &&
               event_id == IP_EVENT_STA_GOT_IP) {

        xEventGroupSetBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT
        );

        ESP_LOGI(TAG, "Wi-Fi Connected");
    }
}


/**
 * @brief Initialize Wi-Fi station mode.
 */
static void wifi_init(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(
        esp_wifi_set_mode(WIFI_MODE_STA)
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_ps(WIFI_PS_NONE)
    );

    wifi_config_t sta_cfg = {0};

    strncpy(
        (char *)sta_cfg.sta.ssid,
        WIFI_SSID,
        sizeof(sta_cfg.sta.ssid) - 1
    );

    strncpy(
        (char *)sta_cfg.sta.password,
        WIFI_PASS,
        sizeof(sta_cfg.sta.password) - 1
    );

    sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_LOGI(
        TAG,
        "Connecting to SSID: %s",
        sta_cfg.sta.ssid
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_config(WIFI_IF_STA, &sta_cfg)
    );

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(
        esp_wifi_set_max_tx_power(34)
    );
}


/**
 * @brief Application entry point.
 */
void app_main(void)
{
    // 1. Initialize NVS

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);


    // 2. Generate MAC-based MQTT topics

    uint8_t mac[6];

    ESP_ERROR_CHECK(
        esp_efuse_mac_get_default(mac)
    );

    snprintf(
        mac_str,
        sizeof(mac_str),
        "%02X%02X%02X%02X%02X%02X",
        mac[0], mac[1], mac[2],
        mac[3], mac[4], mac[5]
    );

    snprintf(
        cmd_topic,
        sizeof(cmd_topic),
        "%s/indoor/switches/%s",
        MQTT_BASE_TOPIC,
        mac_str
    );

    snprintf(
        status_topic,
        sizeof(status_topic),
        "%s/indoor/switches/%s/status",
        MQTT_BASE_TOPIC,
        mac_str
    );


    // 3. Initialize 74HC595 GPIOs

    ESP_ERROR_CHECK(gpio_reset_pin(GPIO_SER));
    ESP_ERROR_CHECK(gpio_reset_pin(GPIO_SRCLK));
    ESP_ERROR_CHECK(gpio_reset_pin(GPIO_RCLK));

    ESP_ERROR_CHECK(
        gpio_set_direction(GPIO_SER, GPIO_MODE_OUTPUT)
    );

    ESP_ERROR_CHECK(
        gpio_set_direction(GPIO_SRCLK, GPIO_MODE_OUTPUT)
    );

    ESP_ERROR_CHECK(
        gpio_set_direction(GPIO_RCLK, GPIO_MODE_OUTPUT)
    );


    // 4. Restore saved switch state

    nvs_handle_t handle;

    current_shift_state = 0;

    if (nvs_open(
            "storage",
            NVS_READONLY,
            &handle
        ) == ESP_OK) {

        nvs_get_u8(
            handle,
            "shift_state",
            &current_shift_state
        );

        nvs_close(handle);
    }

    // Remove bits outside the configured switch count
    current_shift_state &= SWITCH_MASK;

    shift_register_write(current_shift_state);


    // 5. Initialize networking

    wifi_event_group = xEventGroupCreate();

    configASSERT(wifi_event_group != NULL);

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(
        esp_event_loop_create_default()
    );

    ESP_ERROR_CHECK(
        esp_netif_create_default_wifi_sta() != NULL
            ? ESP_OK
            : ESP_FAIL
    );

    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            wifi_event_handler,
            NULL,
            NULL
        )
    );

    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            wifi_event_handler,
            NULL,
            NULL
        )
    );

    wifi_init();


    // 6. MQTT Last Will and Testament

    char bin_str[SWITCHES_NUM + 1];
    char lwt_msg[32];

    uint8_to_binary_string(
        current_shift_state,
        bin_str
    );

    snprintf(
        lwt_msg,
        sizeof(lwt_msg),
        "offline, %s",
        bin_str
    );

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER,

        .credentials.username = MQTT_USER,

        .credentials.authentication.password = MQTT_PASS,

        .session.last_will = {
            .topic = status_topic,
            .msg = lwt_msg,
            .qos = 1,
            .retain = 1
        }
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    ESP_ERROR_CHECK(
        mqtt_client != NULL ? ESP_OK : ESP_FAIL
    );

    ESP_ERROR_CHECK(
        esp_mqtt_client_register_event(
            mqtt_client,
            MQTT_EVENT_ANY,
            mqtt_event_handler,
            NULL
        )
    );

    ESP_ERROR_CHECK(
        esp_mqtt_client_start(mqtt_client)
    );


    // 7. Start heartbeat task

    BaseType_t task_result = xTaskCreate(
        heartbeat_task,
        "heartbeat_task",
        4096,
        NULL,
        5,
        NULL
    );

    configASSERT(task_result == pdPASS);


    ESP_LOGI(
        TAG,
        "System initialized. MAC: %s",
        mac_str
    );

    ESP_LOGI(
        TAG,
        "Configured switches: %d",
        SWITCHES_NUM
    );

    ESP_LOGI(
        TAG,
        "Initial switch state: %s",
        bin_str
    );

    ESP_LOGI(
        TAG,
        "Command topic: %s",
        cmd_topic
    );

    ESP_LOGI(
        TAG,
        "Status topic: %s",
        status_topic
    );
}
