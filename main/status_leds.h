#pragma once

#include "esp_err.h"
#include "mqtt_client.h"

#ifdef __cplusplus
extern "C" {
#endif

// ── GPIO assignments ────────────────────────────────────────
#define SL_IMU_GREEN_GPIO    8
#define SL_IMU_RED_GPIO      9
#define SL_FC_GREEN_GPIO    16
#define SL_FC_RED_GPIO      7
#define SL_UDP_GREEN_GPIO   45

// ── Blynk virtual pins ──────────────────────────────────────
#define SL_BLYNK_IMU_PIN    "V10"
#define SL_BLYNK_FC_PIN     "V11"
#define SL_BLYNK_UDP_PIN    "V12"

/**
 * @brief Initialise GPIO outputs and start 5Hz status LED task on Core 0.
 *
 * Call after flight_control_init() and imu_init() but before or after
 * WiFi — Blynk publishing starts only when MQTT is connected.
 *
 * @param mqtt_client Handle to the active MQTT client.
 * @return ESP_OK on success.
 */
esp_err_t status_leds_init(esp_mqtt_client_handle_t mqtt_client);

#ifdef __cplusplus
}
#endif