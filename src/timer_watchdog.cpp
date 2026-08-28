#include "timer_watchdog.hpp"
#include "safety_types.hpp"
#include "telemetry_task.hpp"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_attr.h"

static const char* TAG = "TIMER_WATCHDOG";

namespace flight {

static gptimer_handle_t  s_gptimer_handle = nullptr;
static TaskHandle_t      s_target_task    = nullptr;
static volatile uint32_t s_timeout_count  = 0;
static uint32_t          s_period_us      = 3000;
static bool              s_is_running     = false;

/**
 * @brief High-priority IRAM alarm callback executed when MPU6050 DRDY pulse is lost
 */
static bool IRAM_ATTR timer_watchdog_on_alarm_cb(gptimer_handle_t timer, 
                                                 const gptimer_alarm_event_data_t* edata, 
                                                 void* user_ctx) {
    (void)timer;
    (void)edata;
    (void)user_ctx;

    BaseType_t high_task_wakeup = pdFALSE;
    s_timeout_count = s_timeout_count + 1;

    // Mark system health flag: DRDY missed, operating on hardware timer fallback
    g_health_flags |= HEALTH_FLAG_TIMER_FALLBACK_ACTIVE;

    if (s_target_task != nullptr) {
        vTaskNotifyGiveFromISR(s_target_task, &high_task_wakeup);
    }

    return (high_task_wakeup == pdTRUE);
}

esp_err_t timer_watchdog_init(FlightProfileId profile, TaskHandle_t target_task) {
    s_target_task = target_task;

    const auto* p_cfg = get_profile_config(profile);
    if (p_cfg == nullptr || p_cfg->rate_hz == 0) {
        ESP_LOGE(TAG, "Invalid flight profile for timer watchdog init");
        return ESP_ERR_INVALID_ARG;
    }

    // Alarm timeout = 1.5 * T_sample (in microseconds)
    uint32_t sample_period_us = 1000000UL / p_cfg->rate_hz;
    s_period_us = (sample_period_us * 3) / 2;

    ESP_LOGI(TAG, "Configuring GPTimer Watchdog for %s (Rate: %u Hz, Sample: %u us, Timeout Alarm: %u us)...",
             p_cfg->name, p_cfg->rate_hz, static_cast<unsigned>(sample_period_us), static_cast<unsigned>(s_period_us));

    // 1. Delete previous timer instance if re-initializing
    if (s_gptimer_handle != nullptr) {
        gptimer_stop(s_gptimer_handle);
        gptimer_disable(s_gptimer_handle);
        gptimer_del_timer(s_gptimer_handle);
        s_gptimer_handle = nullptr;
    }

    // 2. Configure 1 MHz resolution hardware timer
    gptimer_config_t timer_config = {};
    timer_config.clk_src       = GPTIMER_CLK_SRC_DEFAULT;
    timer_config.direction     = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = 1000000; // 1 MHz -> 1 count = 1 microsecond

    esp_err_t err = gptimer_new_timer(&timer_config, &s_gptimer_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create new GPTimer instance: %s", esp_err_to_name(err));
        return err;
    }

    // 3. Register high-speed alarm callback
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_watchdog_on_alarm_cb,
    };
    err = gptimer_register_event_callbacks(s_gptimer_handle, &cbs, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GPTimer alarm callbacks: %s", esp_err_to_name(err));
        return err;
    }

    // 4. Configure auto-reload alarm
    gptimer_alarm_config_t alarm_config = {};
    alarm_config.reload_count                = 0;
    alarm_config.alarm_count                 = s_period_us;
    alarm_config.flags.auto_reload_on_alarm  = true;

    err = gptimer_set_alarm_action(s_gptimer_handle, &alarm_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPTimer alarm action: %s", esp_err_to_name(err));
        return err;
    }

    // 5. Enable timer hardware module
    err = gptimer_enable(s_gptimer_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable GPTimer: %s", esp_err_to_name(err));
        return err;
    }

    s_is_running = false;
    ESP_LOGI(TAG, "GPTimer Watchdog initialized successfully (1.5x T_sample = %u us)", static_cast<unsigned>(s_period_us));
    return ESP_OK;
}

esp_err_t timer_watchdog_start() {
    if (s_gptimer_handle == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_is_running) {
        esp_err_t err = gptimer_start(s_gptimer_handle);
        if (err == ESP_OK) {
            s_is_running = true;
        }
        return err;
    }
    return ESP_OK;
}

esp_err_t timer_watchdog_stop() {
    if (s_gptimer_handle == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_is_running) {
        esp_err_t err = gptimer_stop(s_gptimer_handle);
        if (err == ESP_OK) {
            s_is_running = false;
        }
        return err;
    }
    return ESP_OK;
}

esp_err_t timer_watchdog_feed() {
    if (s_gptimer_handle == nullptr || !s_is_running) {
        return ESP_ERR_INVALID_STATE;
    }
    // Reset counter to 0 on each valid DRDY interrupt pulse
    return gptimer_set_raw_count(s_gptimer_handle, 0);
}

uint32_t timer_watchdog_get_timeout_count() {
    return s_timeout_count;
}

uint32_t timer_watchdog_get_period_us() {
    return s_period_us;
}

} // namespace flight
