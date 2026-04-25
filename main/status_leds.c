/*
 * status_leds.c — Physical and Blynk LED status indicators
 *
 * 5 Hz task on Core 0. Reads IMU, FC, and setpoint state.
 * Drives 5 GPIO LEDs and publishes to 3 Blynk virtual pins.
 *
 * Physical:
 *   GPIO2  = IMU Green
 *   GPIO7  = IMU Red
 *   GPIO12 = FC Green  (armed)
 *   GPIO13 = FC Red    (emergency)
 *   GPIO14 = UDP Green (fresh setpoint)
 *
 * Blynk:
 *   V10 = IMU status  (color: amber=calibrating, green=OK, red=fail)
 *   V11 = FC state    (color: off=disarmed, green=armed, red=emergency)
 *   V12 = UDP link    (color: green=fresh, off=stale)
 */

#include "status_leds.h"
#include "imu.h"
#include "flight_control.h"
#include "setpoint.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "[LEDS]";

#define SL_TASK_STACK       4096
#define SL_TASK_PRIO        2           // Below Blynk publish (3), above idle
#define SL_TASK_CORE        0
#define SL_PERIOD_MS        200         // 5 Hz

// Blink periods in task ticks (each tick = 200ms)
#define SL_BLINK_SLOW_TICKS  5          // 1 Hz — calibrating, emergency level
#define SL_BLINK_FAST_TICKS  2          // 2.5 Hz — emergency land

static esp_mqtt_client_handle_t s_mqtt = NULL;
static bool                     s_started = false;

// ── GPIO init ─────────────────────────────────────────────────────────────
static void init_gpio_output(int pin)
{
    gpio_config_t cfg = {
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << pin,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(pin, 0);
}

// ── Blynk helpers ─────────────────────────────────────────────────────────
static void blynk_led_color(const char *vpin, const char *hex_color)
{
    if (!s_mqtt) return;
    char topic[48];

    // Set color property
    snprintf(topic, sizeof(topic), "setProperty/%s/color", vpin);
    esp_mqtt_client_publish(s_mqtt, topic, hex_color, 0, 0, 0);

    // Turn widget on (brightness=255) — color alone doesn't make it visible
    char ds_topic[24];
    snprintf(ds_topic, sizeof(ds_topic), "ds/%s", vpin);
    esp_mqtt_client_publish(s_mqtt, ds_topic, "255", 0, 0, 0);
}

static void blynk_led_off(const char *vpin)
{
    if (!s_mqtt) return;
    char topic[24];
    snprintf(topic, sizeof(topic), "ds/%s", vpin);
    esp_mqtt_client_publish(s_mqtt, topic, "0", 0, 0, 0);
}

// ── LED task ──────────────────────────────────────────────────────────────
static void status_led_task(void *arg)
{
    ESP_LOGI(TAG, "Status LED task started — 5 Hz, Core %d", SL_TASK_CORE);

    TickType_t  last_wake   = xTaskGetTickCount();
    uint32_t    tick        = 0;
    // Track last published state to avoid spamming MQTT on every tick
    int         last_imu    = -1;
    int         last_fc     = -1;
    int         last_udp    = -1;

    // IMU LED states
    enum { IMU_CALIBRATING=0, IMU_HEALTHY=1, IMU_FAILED=2 };
    // FC LED states
    enum { FC_DISARMED=0, FC_ARMED=1, FC_EMERG_LEVEL=2, FC_EMERG_LAND=3 };

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SL_PERIOD_MS));
        tick++;

        // ── Read state (non-blocking) ─────────────────────────
        imu_data_t imu;
        imu_get_data(&imu);

        fc_flight_state_t fc_state = flight_control_get_state();
        bool udp_fresh = setpoint_is_fresh();

        // ── Determine LED states ──────────────────────────────

        // IMU
        int imu_led;
        if (!imu.calibrated) {
            imu_led = IMU_CALIBRATING;
        } else if (imu.healthy) {
            imu_led = IMU_HEALTHY;
        } else {
            imu_led = IMU_FAILED;
        }

        // FC
        int fc_led;
        switch (fc_state) {
            case FC_STATE_ARMED_NORMAL:      fc_led = FC_ARMED;        break;
            case FC_STATE_EMERGENCY_LEVEL:   fc_led = FC_EMERG_LEVEL;  break;
            case FC_STATE_EMERGENCY_LAND:    fc_led = FC_EMERG_LAND;   break;
            default:                         fc_led = FC_DISARMED;     break;
        }

        // ── Physical LEDs ─────────────────────────────────────

        // IMU LEDs
        switch (imu_led) {
            case IMU_CALIBRATING:
                // Slow alternate blink — "do not move"
                gpio_set_level(SL_IMU_GREEN_GPIO, (tick % SL_BLINK_SLOW_TICKS) < 2 ? 1 : 0);
                gpio_set_level(SL_IMU_RED_GPIO,   (tick % SL_BLINK_SLOW_TICKS) < 2 ? 0 : 1);
                break;
            case IMU_HEALTHY:
                gpio_set_level(SL_IMU_GREEN_GPIO, 1);
                gpio_set_level(SL_IMU_RED_GPIO,   0);
                break;
            case IMU_FAILED:
                gpio_set_level(SL_IMU_GREEN_GPIO, 0);
                gpio_set_level(SL_IMU_RED_GPIO,   1);
                break;
        }

        // FC LEDs
        switch (fc_led) {
            case FC_DISARMED:
                gpio_set_level(SL_FC_GREEN_GPIO, 0);
                gpio_set_level(SL_FC_RED_GPIO,   0);
                break;
            case FC_ARMED:
                gpio_set_level(SL_FC_GREEN_GPIO, 1);
                gpio_set_level(SL_FC_RED_GPIO,   0);
                break;
            case FC_EMERG_LEVEL:
                // Slow blink red — leveling, not yet landing
                gpio_set_level(SL_FC_GREEN_GPIO, 0);
                gpio_set_level(SL_FC_RED_GPIO, (tick % SL_BLINK_SLOW_TICKS) < 2 ? 1 : 0);
                break;
            case FC_EMERG_LAND:
                // Fast blink red — committed descent
                gpio_set_level(SL_FC_GREEN_GPIO, 0);
                gpio_set_level(SL_FC_RED_GPIO, (tick % SL_BLINK_FAST_TICKS) < 1 ? 1 : 0);
                break;
        }

        // UDP LED
        gpio_set_level(SL_UDP_GREEN_GPIO, udp_fresh ? 1 : 0);

        // ── Blynk (publish only on state change to avoid MQTT spam) ──
        if (imu_led != last_imu) {
            switch (imu_led) {
                case IMU_CALIBRATING:
                    blynk_led_color(SL_BLYNK_IMU_PIN, "#FF8C00"); // amber
                    break;
                case IMU_HEALTHY:
                    blynk_led_color(SL_BLYNK_IMU_PIN, "#00FF00"); // green
                    break;
                case IMU_FAILED:
                    blynk_led_color(SL_BLYNK_IMU_PIN, "#FF0000"); // red
                    break;
            }
            last_imu = imu_led;
        }

        if (fc_led != last_fc) {
            switch (fc_led) {
                case FC_DISARMED:
                    blynk_led_off(SL_BLYNK_FC_PIN);
                    break;
                case FC_ARMED:
                    blynk_led_color(SL_BLYNK_FC_PIN, "#00FF00"); // green
                    break;
                case FC_EMERG_LEVEL:
                    blynk_led_color(SL_BLYNK_FC_PIN, "#FF4500"); // orange-red
                    break;
                case FC_EMERG_LAND:
                    blynk_led_color(SL_BLYNK_FC_PIN, "#FF0000"); // red
                    break;
            }
            last_fc = fc_led;
        }

        int udp_state = udp_fresh ? 1 : 0;
        if (udp_state != last_udp) {
            if (udp_fresh) {
                blynk_led_color(SL_BLYNK_UDP_PIN, "#00FF00");
            } else {
                blynk_led_off(SL_BLYNK_UDP_PIN);
            }
            last_udp = udp_state;
        }
    }
}

// ── Public API ────────────────────────────────────────────────────────────
esp_err_t status_leds_init(esp_mqtt_client_handle_t mqtt_client)
{
    if (s_started) {
        ESP_LOGW(TAG, "Already initialised");
        return ESP_OK;
    }

    s_mqtt = mqtt_client;

    // Init all 5 GPIO pins
    init_gpio_output(SL_IMU_GREEN_GPIO);
    init_gpio_output(SL_IMU_RED_GPIO);
    init_gpio_output(SL_FC_GREEN_GPIO);
    init_gpio_output(SL_FC_RED_GPIO);
    init_gpio_output(SL_UDP_GREEN_GPIO);

    // Boot state: all off
    gpio_set_level(SL_IMU_GREEN_GPIO,  0);
    gpio_set_level(SL_IMU_RED_GPIO,    0);
    gpio_set_level(SL_FC_GREEN_GPIO,   0);
    gpio_set_level(SL_FC_RED_GPIO,     0);
    gpio_set_level(SL_UDP_GREEN_GPIO,  0);

    BaseType_t rc = xTaskCreatePinnedToCore(
        status_led_task, "led_status",
        SL_TASK_STACK, NULL, SL_TASK_PRIO, NULL, SL_TASK_CORE
    );
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Task creation failed");
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    ESP_LOGI(TAG, "Status LEDs ready — IMU G%d/R%d | FC G%d/R%d | UDP G%d",
             SL_IMU_GREEN_GPIO, SL_IMU_RED_GPIO,
             SL_FC_GREEN_GPIO,  SL_FC_RED_GPIO,
             SL_UDP_GREEN_GPIO);
    return ESP_OK;
}