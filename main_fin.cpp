#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "lighting_fin.h"
#include "sensors_fin.h"

static const char* TAG = "MAIN";

// ===============================================================
// TASK 1: UI / HMI UART
// ===============================================================
#define UI_UART_PORT   UART_NUM_1
#define UI_UART_TX_PIN GPIO_NUM_16
#define UI_UART_RX_PIN GPIO_NUM_17
#define UI_UART_BAUD   115200

#define UI_UART_RX_BUF_SIZE  1024
#define UI_LINE_BUF_SIZE     128

// ===============================================================
// STATUS / POWER LED
// ===============================================================
#define POWER_LED_PIN GPIO_NUM_2

// ===============================================================
// CAMERA TRIGGER OUTPUT
// Change if your camera trigger wire uses a different GPIO.
// Sends a short HIGH pulse on outside motion rising edge.
// ===============================================================
#define CAMERA_TRIGGER_PIN GPIO_NUM_42

// ===============================================================
// TIMING
// ===============================================================
static const TickType_t CONTROL_PERIOD_TICKS   = pdMS_TO_TICKS(50);
static const TickType_t NETWORK_PERIOD_TICKS   = pdMS_TO_TICKS(500);
static const TickType_t FOYER_HOLD_TICKS       = pdMS_TO_TICKS(10000);
static const TickType_t SECURITY_HOLD_TICKS    = pdMS_TO_TICKS(10000);
static const TickType_t CAMERA_PULSE_TICKS     = pdMS_TO_TICKS(150);
static const TickType_t CAMERA_COOLDOWN_TICKS  = pdMS_TO_TICKS(1500);

// Set to 1 if you want raw UART byte debug
#define UART_RAW_DEBUG 0

// Threshold copied from sensors_fin.cpp so we can avoid double-reading ALS
#define DARK_THRESHOLD 120

// ===========================================================
// Shared system state
// ===========================================================
typedef struct {
    // Manual command state from HMI / app
    bool porch_manual_on;
    bool foyer_manual_on;
    bool security_manual_on;

    // Auto enable/disable flags
    bool porch_auto_enabled;
    bool foyer_auto_enabled;
    bool security_auto_enabled;

    // Sensor snapshot
    int  lux;
    bool dark;
    bool pir_outside;
    bool pir_inside;

    // Motion hold bookkeeping
    TickType_t last_outside_motion_time;
    TickType_t last_inside_motion_time;

    // Edge detection
    bool prev_pir_outside;
    bool prev_pir_inside;

    // Camera pulse bookkeeping
    bool camera_pulse_active;
    TickType_t camera_pulse_end_time;
    TickType_t last_camera_trigger_time;

    // Final outputs
    bool porch_on;
    bool foyer_on;
    bool security_on;
} system_state_t;

static system_state_t g{};
static SemaphoreHandle_t g_mutex = nullptr;

static inline void lock_state()   { xSemaphoreTake(g_mutex, portMAX_DELAY); }
static inline void unlock_state() { xSemaphoreGive(g_mutex); }

// ===========================================================
// Helpers
// ===========================================================
static void trim_inplace(char* s)
{
    char* p = s;
    while (*p && isspace((unsigned char)*p)) p++;
    if (p != s) {
        memmove(s, p, strlen(p) + 1);
    }

    size_t n = strlen(s);
    while (n > 0 && isspace((unsigned char)s[n - 1])) {
        s[--n] = '\0';
    }
}

static void uart_send_line(const char* fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len < 0) return;
    if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;

    uart_write_bytes(UI_UART_PORT, buf, len);
}

static void power_led_init(void)
{
    gpio_config_t io{};
    io.pin_bit_mask = (1ULL << POWER_LED_PIN);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_set_level(POWER_LED_PIN, 1));

    ESP_LOGI(TAG, "Power LED initialized on GPIO %d", (int)POWER_LED_PIN);
}

static void camera_trigger_init(void)
{
    gpio_config_t io{};
    io.pin_bit_mask = (1ULL << CAMERA_TRIGGER_PIN);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_set_level(CAMERA_TRIGGER_PIN, 0));

    ESP_LOGI(TAG, "Camera trigger initialized on GPIO %d", (int)CAMERA_TRIGGER_PIN);
}

static inline void camera_trigger_high(void)
{
    gpio_set_level(CAMERA_TRIGGER_PIN, 1);
}

static inline void camera_trigger_low(void)
{
    gpio_set_level(CAMERA_TRIGGER_PIN, 0);
}

static void ui_uart_init(void)
{
    uart_config_t cfg{};
    cfg.baud_rate = UI_UART_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity    = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(UI_UART_PORT, UI_UART_RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UI_UART_PORT, &cfg));
    ESP_ERROR_CHECK(
        uart_set_pin(
            UI_UART_PORT,
            UI_UART_TX_PIN,
            UI_UART_RX_PIN,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE
        )
    );

    ESP_LOGI(TAG, "UI UART initialized: TX=%d RX=%d baud=%d",
             (int)UI_UART_TX_PIN, (int)UI_UART_RX_PIN, UI_UART_BAUD);
}

// ===========================================================
// TASK 1: Handle HMI command lines
//
// Supported commands:
//   Porch=0|1
//   Foyer=0|1
//   Security=0|1
//   PorchAuto=0|1
//   FoyerAuto=0|1
//   SecurityAuto=0|1
// ===========================================================
static void handle_ui_line(const char* line_in)
{
    char line[UI_LINE_BUF_SIZE];
    snprintf(line, sizeof(line), "%s", line_in);
    trim_inplace(line);

    if (!line[0]) return;

    char* eq = strchr(line, '=');
    if (!eq) {
        ESP_LOGW(TAG, "Ignoring malformed line: %s", line);
        return;
    }

    *eq = '\0';
    char* key = line;
    char* val = eq + 1;

    trim_inplace(key);
    trim_inplace(val);

    for (char* k = key; *k; ++k) {
        *k = (char)toupper((unsigned char)*k);
    }

    int v = atoi(val);
    bool on = (v != 0);

    lock_state();

    if (strcmp(key, "PORCH") == 0) {
        if (g.porch_manual_on != on) {
            ESP_LOGI(TAG, "EVENT: HMI_PORCH_%s at %lu ms",
                     on ? "ON" : "OFF",
                     (unsigned long)esp_log_timestamp());
            g.porch_manual_on = on;
            ESP_LOGI(TAG, "HMI PORCH=%d", (int)on);
        }

    } else if (strcmp(key, "FOYER") == 0) {
        if (g.foyer_manual_on != on) {
            ESP_LOGI(TAG, "EVENT: HMI_FOYER_%s at %lu ms",
                     on ? "ON" : "OFF",
                     (unsigned long)esp_log_timestamp());
            g.foyer_manual_on = on;
            ESP_LOGI(TAG, "HMI FOYER=%d", (int)on);
        }

    } else if (strcmp(key, "SECURITY") == 0) {
        if (g.security_manual_on != on) {
            ESP_LOGI(TAG, "EVENT: HMI_SECURITY_%s at %lu ms",
                     on ? "ON" : "OFF",
                     (unsigned long)esp_log_timestamp());
            g.security_manual_on = on;
            ESP_LOGI(TAG, "HMI SECURITY=%d", (int)on);
        }

    } else if (strcmp(key, "PORCHAUTO") == 0) {
        if (g.porch_auto_enabled != on) {
            g.porch_auto_enabled = on;
            ESP_LOGI(TAG, "HMI PORCHAUTO=%d", (int)on);
        }

    } else if (strcmp(key, "FOYERAUTO") == 0) {
        if (g.foyer_auto_enabled != on) {
            g.foyer_auto_enabled = on;
            ESP_LOGI(TAG, "HMI FOYERAUTO=%d", (int)on);
        }

    } else if (strcmp(key, "SECURITYAUTO") == 0) {
        if (g.security_auto_enabled != on) {
            g.security_auto_enabled = on;
            ESP_LOGI(TAG, "HMI SECURITYAUTO=%d", (int)on);
        }

    } else {
        ESP_LOGW(TAG, "Unknown HMI key: %s", key);
    }

    unlock_state();
}

static void ui_task(void* arg)
{
    (void)arg;

    static char linebuf[UI_LINE_BUF_SIZE];
    int idx = 0;
    uint8_t rx[64];

    while (true)
    {
        int n = uart_read_bytes(UI_UART_PORT, rx, sizeof(rx), pdMS_TO_TICKS(20));

#if UART_RAW_DEBUG
        if (n > 0) {
            ESP_LOGI(TAG, "RX count = %d", n);
            for (int i = 0; i < n; i++) {
                ESP_LOGI(TAG, "RX[%d] = 0x%02X ('%c')",
                         i,
                         rx[i],
                         (rx[i] >= 32 && rx[i] <= 126) ? rx[i] : '.');
            }
        }
#endif

        for (int i = 0; i < n; i++)
        {
            char c = (char)rx[i];

            if (c == '\r' || c == '\n') {
                linebuf[idx] = '\0';
                if (idx > 0) {
                    handle_ui_line(linebuf);
                }
                idx = 0;
            } else {
                if (idx < (UI_LINE_BUF_SIZE - 1)) {
                    linebuf[idx++] = c;
                } else {
                    ESP_LOGW(TAG, "UART line too long, resetting buffer");
                    idx = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ===========================================================
// TASK 2: CONTROL
// Fast loop:
// - read sensors
// - compute final outputs
// - drive relays
// - generate camera trigger pulse on OUTSIDE motion rising edge
//   only when SecurityAuto is enabled
// - event-based logging only
// ===========================================================
static void control_task(void* arg)
{
    (void)arg;

    // HMI echo tracking
    bool last_porch_sent = false;
    bool last_foyer_sent = false;
    bool last_security_sent = false;
    bool last_porch_auto_sent = true;
    bool last_foyer_auto_sent = true;
    bool last_security_auto_sent = true;
    bool first_publish = true;

    // Event logging tracking
    bool last_logged_dark = false;
    bool last_logged_pir_outside = false;
    bool last_logged_pir_inside = false;
    bool last_logged_porch_on = false;
    bool last_logged_foyer_on = false;
    bool last_logged_security_on = false;
    bool last_logged_porch_auto = true;
    bool last_logged_foyer_auto = true;
    bool last_logged_security_auto = true;
    bool first_log = true;

    while (true)
    {
        TickType_t now = xTaskGetTickCount();

        // -----------------------------
        // Read sensors
        // -----------------------------
        int  lux         = sensors_get_ambient();
        bool dark        = (lux >= 0) ? (lux <= DARK_THRESHOLD) : false;
        bool pir_outside = read_pir_outside();
        bool pir_inside  = read_pir_inside();

        // -----------------------------
        // Snapshot current shared state
        // -----------------------------
        bool porch_manual;
        bool foyer_manual;
        bool security_manual;

        bool porch_auto;
        bool foyer_auto;
        bool security_auto;

        bool prev_pir_outside;
        TickType_t last_outside_motion;
        TickType_t last_inside_motion;

        bool camera_pulse_active;
        TickType_t camera_pulse_end_time;
        TickType_t last_camera_trigger_time;

        lock_state();
        g.lux = lux;
        g.dark = dark;
        g.pir_outside = pir_outside;
        g.pir_inside = pir_inside;

        porch_manual = g.porch_manual_on;
        foyer_manual = g.foyer_manual_on;
        security_manual = g.security_manual_on;

        porch_auto = g.porch_auto_enabled;
        foyer_auto = g.foyer_auto_enabled;
        security_auto = g.security_auto_enabled;

        prev_pir_outside = g.prev_pir_outside;

        last_outside_motion = g.last_outside_motion_time;
        last_inside_motion = g.last_inside_motion_time;

        camera_pulse_active = g.camera_pulse_active;
        camera_pulse_end_time = g.camera_pulse_end_time;
        last_camera_trigger_time = g.last_camera_trigger_time;
        unlock_state();

        // -----------------------------
        // Update motion timestamps only if auto is enabled
        // -----------------------------
        if (security_auto && pir_outside) {
            last_outside_motion = now;
        }

        if (foyer_auto && pir_inside) {
            last_inside_motion = now;
        }

        // -----------------------------
        // Camera pulse logic
        // Only when security auto is enabled
        // Rising edge only on OUTSIDE PIR
        // -----------------------------
        bool outside_rising_edge = (!prev_pir_outside && pir_outside);

        if (security_auto &&
            outside_rising_edge &&
            !camera_pulse_active &&
            ((now - last_camera_trigger_time) >= CAMERA_COOLDOWN_TICKS))
        {
            camera_trigger_high();
            camera_pulse_active = true;
            camera_pulse_end_time = now + CAMERA_PULSE_TICKS;
            last_camera_trigger_time = now;

            ESP_LOGI(TAG, "Camera trigger pulse started");
        }

        if (camera_pulse_active && (now >= camera_pulse_end_time)) {
            camera_trigger_low();
            camera_pulse_active = false;
            ESP_LOGI(TAG, "Camera trigger pulse ended");
        }

        if (!security_auto && camera_pulse_active) {
            camera_trigger_low();
            camera_pulse_active = false;
        }

        // -----------------------------
        // Compute automatic behavior
        // -----------------------------
        bool porch_auto_on      = porch_auto && dark;
        bool foyer_motion_on    = foyer_auto && ((now - last_inside_motion) < FOYER_HOLD_TICKS);
        bool security_motion_on = security_auto && ((now - last_outside_motion) < SECURITY_HOLD_TICKS);

        // Final outputs:
        // if auto enabled -> manual OR auto trigger
        // if auto disabled -> manual only
        bool porch_on    = porch_auto    ? (porch_manual    || porch_auto_on)      : porch_manual;
        bool foyer_on    = foyer_auto    ? (foyer_manual    || foyer_motion_on)    : foyer_manual;
        bool security_on = security_auto ? (security_manual || security_motion_on) : security_manual;

        // -----------------------------
        // Apply outputs
        // -----------------------------
        lighting_set_porch(porch_on);
        lighting_set_foyer(foyer_on);
        lighting_set_security(security_on);

        // -----------------------------
        // Save state
        // -----------------------------
        lock_state();
        g.last_outside_motion_time = last_outside_motion;
        g.last_inside_motion_time  = last_inside_motion;

        g.prev_pir_outside = pir_outside;
        g.prev_pir_inside  = pir_inside;

        g.camera_pulse_active = camera_pulse_active;
        g.camera_pulse_end_time = camera_pulse_end_time;
        g.last_camera_trigger_time = last_camera_trigger_time;

        g.porch_on = porch_on;
        g.foyer_on = foyer_on;
        g.security_on = security_on;
        unlock_state();

        // -----------------------------
        // Echo actual state back to HMI only when changed
        // -----------------------------
        if (first_publish || porch_on != last_porch_sent) {
            uart_send_line("Porch=%d\n", porch_on ? 1 : 0);
            last_porch_sent = porch_on;
        }

        if (first_publish || foyer_on != last_foyer_sent) {
            uart_send_line("Foyer=%d\n", foyer_on ? 1 : 0);
            last_foyer_sent = foyer_on;
        }

        if (first_publish || security_on != last_security_sent) {
            uart_send_line("Security=%d\n", security_on ? 1 : 0);
            last_security_sent = security_on;
        }

        if (first_publish || porch_auto != last_porch_auto_sent) {
            uart_send_line("PorchAuto=%d\n", porch_auto ? 1 : 0);
            last_porch_auto_sent = porch_auto;
        }

        if (first_publish || foyer_auto != last_foyer_auto_sent) {
            uart_send_line("FoyerAuto=%d\n", foyer_auto ? 1 : 0);
            last_foyer_auto_sent = foyer_auto;
        }

        if (first_publish || security_auto != last_security_auto_sent) {
            uart_send_line("SecurityAuto=%d\n", security_auto ? 1 : 0);
            last_security_auto_sent = security_auto;
        }

        first_publish = false;

        // -----------------------------
        // Event-based logging only
        // -----------------------------

        // Dark threshold crossing
        if (first_log || dark != last_logged_dark) {
            ESP_LOGI(TAG, "EVENT: DARK_THRESHOLD_%s at %lu ms (lux=%d)",
                     dark ? "DARK" : "BRIGHT",
                     (unsigned long)esp_log_timestamp(),
                     lux);
            last_logged_dark = dark;
        }

        // Outside PIR edge logging
        if (first_log || pir_outside != last_logged_pir_outside) {
            ESP_LOGI(TAG, "Outside PIR changed: %d", (int)pir_outside);

            if (pir_outside) {
                ESP_LOGI(TAG, "EVENT: PIR_OUTSIDE_RISE at %lu ms",
                         (unsigned long)esp_log_timestamp());
            } else {
                ESP_LOGI(TAG, "EVENT: PIR_OUTSIDE_FALL at %lu ms",
                         (unsigned long)esp_log_timestamp());
            }

            last_logged_pir_outside = pir_outside;
        }

        // Inside PIR edge logging
        if (first_log || pir_inside != last_logged_pir_inside) {
            ESP_LOGI(TAG, "Inside PIR changed: %d", (int)pir_inside);

            if (pir_inside) {
                ESP_LOGI(TAG, "EVENT: PIR_INSIDE_RISE at %lu ms",
                         (unsigned long)esp_log_timestamp());
            } else {
                ESP_LOGI(TAG, "EVENT: PIR_INSIDE_FALL at %lu ms",
                         (unsigned long)esp_log_timestamp());
            }

            last_logged_pir_inside = pir_inside;
        }

        // Auto mode logging
        if (first_log || porch_auto != last_logged_porch_auto) {
            ESP_LOGI(TAG, "PorchAuto changed: %d", (int)porch_auto);
            last_logged_porch_auto = porch_auto;
        }

        if (first_log || foyer_auto != last_logged_foyer_auto) {
            ESP_LOGI(TAG, "FoyerAuto changed: %d", (int)foyer_auto);
            last_logged_foyer_auto = foyer_auto;
        }

        if (first_log || security_auto != last_logged_security_auto) {
            ESP_LOGI(TAG, "SecurityAuto changed: %d", (int)security_auto);
            last_logged_security_auto = security_auto;
        }

        // Output change logging for all 3 lights
        if (first_log || porch_on != last_logged_porch_on) {
            ESP_LOGI(TAG, "EVENT: PORCH_OUTPUT_%s at %lu ms",
                     porch_on ? "ON" : "OFF",
                     (unsigned long)esp_log_timestamp());
            last_logged_porch_on = porch_on;
        }

        if (first_log || foyer_on != last_logged_foyer_on) {
            ESP_LOGI(TAG, "EVENT: FOYER_OUTPUT_%s at %lu ms",
                     foyer_on ? "ON" : "OFF",
                     (unsigned long)esp_log_timestamp());
            last_logged_foyer_on = foyer_on;
        }

        if (first_log || security_on != last_logged_security_on) {
            ESP_LOGI(TAG, "EVENT: SECURITY_OUTPUT_%s at %lu ms",
                     security_on ? "ON" : "OFF",
                     (unsigned long)esp_log_timestamp());
            last_logged_security_on = security_on;
        }

        first_log = false;

        vTaskDelay(CONTROL_PERIOD_TICKS);
    }
}

// ===========================================================
// TASK 3: NETWORK / DATABASE
// Placeholder shell for WiFi/Firebase/app sync.
// This is where slow/blocking work belongs later.
// ===========================================================
static void network_task(void* arg)
{
    (void)arg;

    while (true)
    {
        // TODO:
        // - Firebase reads/writes
        // - app command handling
        // - telemetry upload
        // - charge controller / battery / database sync
        vTaskDelay(NETWORK_PERIOD_TICKS);
    }
}

// ===========================================================
// MAIN
// ===========================================================
extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "Starting 3-task system: UI + Control + Network");

    g_mutex = xSemaphoreCreateMutex();
    if (g_mutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    power_led_init();
    camera_trigger_init();
    ui_uart_init();
    lighting_init();
    sensors_init();

    lock_state();

    // Startup defaults you approved
    g.porch_manual_on = false;
    g.foyer_manual_on = false;
    g.security_manual_on = false;

    g.porch_auto_enabled = true;
    g.foyer_auto_enabled = true;
    g.security_auto_enabled = true;

    g.lux = 0;
    g.dark = false;
    g.pir_outside = false;
    g.pir_inside = false;

    g.last_outside_motion_time = 0;
    g.last_inside_motion_time = 0;

    g.prev_pir_outside = false;
    g.prev_pir_inside = false;

    g.camera_pulse_active = false;
    g.camera_pulse_end_time = 0;
    g.last_camera_trigger_time = 0;

    g.porch_on = false;
    g.foyer_on = false;
    g.security_on = false;

    unlock_state();

    lighting_set_porch(false);
    lighting_set_foyer(false);
    lighting_set_security(false);
    camera_trigger_low();

    xTaskCreate(ui_task,      "ui_task",      4096, nullptr, 8,  nullptr);
    xTaskCreate(control_task, "control_task", 6144, nullptr, 10, nullptr);
    xTaskCreate(network_task, "network_task", 4096, nullptr, 5,  nullptr);

    ESP_LOGI(TAG, "UI task started.");
    ESP_LOGI(TAG, "Control task started.");
    ESP_LOGI(TAG, "Network task started.");
}
