#include "network_fin.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"

#include "wifi_fin.h"
#include "firebase_config_fin.h"

// ===========================================================
// CONFIG
// ===========================================================

#ifndef FIREBASE_AUTH
#define FIREBASE_AUTH ""
#endif

#ifdef FIREBASE_ROOT_CA_PEM
static const char* s_cert_pem = FIREBASE_ROOT_CA_PEM;
#else
static const char* s_cert_pem = nullptr;
#endif

static const char* TAG = "NETWORK";

// Faster, more responsive final-product timing
static const TickType_t UPLOAD_LOOP_PERIOD_TICKS   = pdMS_TO_TICKS(200);
static const TickType_t COMMAND_LOOP_PERIOD_TICKS  = pdMS_TO_TICKS(200);

static const TickType_t STATE_UPLOAD_PERIOD_TICKS  = pdMS_TO_TICKS(2000);
static const TickType_t CHARGE_UPLOAD_PERIOD_TICKS = pdMS_TO_TICKS(1000);
static const TickType_t COMMAND_POLL_PERIOD_TICKS  = pdMS_TO_TICKS(250);
static const TickType_t META_UPLOAD_PERIOD_TICKS   = pdMS_TO_TICKS(5000);

// ===========================================================
// INTERNAL STATE
// ===========================================================
static SemaphoreHandle_t s_network_mutex = nullptr;

static network_state_snapshot_t s_latest_snapshot{};
static bool s_snapshot_valid = false;
static bool s_state_dirty = false;

static charge_telem_t s_latest_charge_telem{};
static bool s_charge_telem_valid = false;
static bool s_charge_dirty = false;

static network_command_t s_pending_command{};
static bool s_has_pending_command = false;
static int s_last_seen_seq = -1;
static int s_last_acked_seq = -1;

// ===========================================================
// HELPERS
// ===========================================================
static inline void lock_network(void)
{
    xSemaphoreTake(s_network_mutex, portMAX_DELAY);
}

static inline void unlock_network(void)
{
    xSemaphoreGive(s_network_mutex);
}

static void build_firebase_url(char* out, size_t out_size, const char* path)
{
    if (!out || out_size == 0 || !path) return;

    if (strlen(FIREBASE_AUTH) > 0) {
        snprintf(out, out_size, "%s%s.json?auth=%s", FIREBASE_URL, path, FIREBASE_AUTH);
    } else {
        snprintf(out, out_size, "%s%s.json", FIREBASE_URL, path);
    }
}

static esp_http_client_handle_t make_http_client(const char* url, esp_http_client_method_t method)
{
    esp_http_client_config_t config = {};
    config.url = url;
    config.method = method;

    // Reduced from 10000 ms to 2000 ms for responsiveness
    config.timeout_ms = 2000;

    if (strncmp(url, "https://", 8) == 0) {
        config.transport_type = HTTP_TRANSPORT_OVER_SSL;

        if (s_cert_pem != nullptr && strlen(s_cert_pem) > 0) {
            config.cert_pem = s_cert_pem;
        } else {
            config.crt_bundle_attach = esp_crt_bundle_attach;
        }
    } else {
        config.transport_type = HTTP_TRANSPORT_OVER_TCP;
    }

    return esp_http_client_init(&config);
}

static esp_err_t http_put_json(const char* path, const char* json_body)
{
    if (!path || !json_body) return ESP_ERR_INVALID_ARG;

    char url[512];
    build_firebase_url(url, sizeof(url), path);

    esp_http_client_handle_t client = make_http_client(url, HTTP_METHOD_PUT);
    if (!client) return ESP_FAIL;

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_body, strlen(json_body));

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);

    if (err == ESP_OK && status >= 200 && status < 300) {
        ESP_LOGI(TAG, "PUT OK: %s", path);
    } else {
        ESP_LOGE(TAG, "PUT FAILED: %s err=%s status=%d",
                 path, esp_err_to_name(err), status);
        if (err == ESP_OK) err = ESP_FAIL;
    }

    esp_http_client_cleanup(client);
    return err;
}

static esp_err_t http_get(const char* path, char* buffer, size_t buffer_size)
{
    if (!path || !buffer || buffer_size == 0) return ESP_ERR_INVALID_ARG;

    char url[512];
    build_firebase_url(url, sizeof(url), path);

    esp_http_client_handle_t client = make_http_client(url, HTTP_METHOD_GET);
    if (!client) return ESP_FAIL;

    buffer[0] = '\0';

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GET OPEN FAILED: %s err=%s", path, esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    int status = esp_http_client_fetch_headers(client);
    if (status < 0) {
        ESP_LOGE(TAG, "GET HEADERS FAILED: %s", path);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    int total = esp_http_client_read_response(client, buffer, buffer_size - 1);
    if (total < 0) {
        ESP_LOGE(TAG, "GET READ FAILED: %s", path);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    buffer[total] = '\0';

    int http_status = esp_http_client_get_status_code(client);
    if (!(http_status >= 200 && http_status < 300)) {
        ESP_LOGE(TAG, "GET FAILED: %s status=%d body=%s", path, http_status, buffer);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return ESP_OK;
}

static bool json_find_key(const char* json, const char* key, const char** out_value_start)
{
    if (!json || !key || !out_value_start) return false;

    char pattern[96];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    const char* p = strstr(json, pattern);
    if (!p) return false;

    p = strchr(p, ':');
    if (!p) return false;
    p++;

    while (*p && isspace((unsigned char)*p)) p++;

    *out_value_start = p;
    return true;
}

static bool json_parse_int_field(const char* json, const char* key, int* out_value)
{
    if (!out_value) return false;

    const char* p = nullptr;
    if (!json_find_key(json, key, &p)) return false;

    if (strncmp(p, "null", 4) == 0) {
        *out_value = 0;
        return true;
    }

    if (*p == '\"') {
        p++;
        *out_value = atoi(p);
        return true;
    }

    *out_value = atoi(p);
    return true;
}

static bool json_parse_bool_field(const char* json, const char* key, bool* out_value)
{
    if (!out_value) return false;

    const char* p = nullptr;
    if (!json_find_key(json, key, &p)) return false;

    if (strncmp(p, "true", 4) == 0 || strncmp(p, "\"true\"", 6) == 0 || *p == '1') {
        *out_value = true;
        return true;
    }

    if (strncmp(p, "false", 5) == 0 || strncmp(p, "\"false\"", 7) == 0 || *p == '0') {
        *out_value = false;
        return true;
    }

    return false;
}

// ===========================================================
// JSON BUILDERS
// ===========================================================
static void build_controls_json(const network_state_snapshot_t* s, char* out, size_t out_size)
{
    snprintf(
        out,
        out_size,
        "{"
        "\"porch_manual_on\":%s,"
        "\"foyer_manual_on\":%s,"
        "\"security_manual_on\":%s,"
        "\"porch_auto_enabled\":%s,"
        "\"foyer_auto_enabled\":%s,"
        "\"security_auto_enabled\":%s,"
        "\"security_mode\":\"auto\""
        "}",
        s->porch_manual_on ? "true" : "false",
        s->foyer_manual_on ? "true" : "false",
        s->security_manual_on ? "true" : "false",
        s->porch_auto_enabled ? "true" : "false",
        s->foyer_auto_enabled ? "true" : "false",
        s->security_auto_enabled ? "true" : "false"
    );
}

static void build_sensors_json(const network_state_snapshot_t* s, char* out, size_t out_size)
{
    snprintf(
        out,
        out_size,
        "{"
        "\"lux\":%d,"
        "\"dark\":%s,"
        "\"pir_outside\":%s,"
        "\"pir_inside\":%s,"
        "\"sensor_health\":\"ok\","
        "\"last_sensor_update_ts\":%lu"
        "}",
        s->lux,
        s->dark ? "true" : "false",
        s->pir_outside ? "true" : "false",
        s->pir_inside ? "true" : "false",
        (unsigned long)s->timestamp_ms
    );
}

static void build_lights_json(const network_state_snapshot_t* s, char* out, size_t out_size)
{
    snprintf(
        out,
        out_size,
        "{"
        "\"porch_on\":%s,"
        "\"foyer_on\":%s,"
        "\"security_on\":%s,"
        "\"last_output_update_ts\":%lu"
        "}",
        s->porch_on ? "true" : "false",
        s->foyer_on ? "true" : "false",
        s->security_on ? "true" : "false",
        (unsigned long)s->timestamp_ms
    );
}

static void build_network_json(const network_state_snapshot_t* s, char* out, size_t out_size,
                               uint32_t last_upload_ts, uint32_t last_poll_ts)
{
    snprintf(
        out,
        out_size,
        "{"
        "\"wifi_connected\":%s,"
        "\"internet_reachable\":%s,"
        "\"last_successful_upload_ts\":%lu,"
        "\"last_successful_command_poll_ts\":%lu,"
        "\"timestamp_ms\":%lu"
        "}",
        s->wifi_connected ? "true" : "false",
        s->wifi_connected ? "true" : "false",
        (unsigned long)last_upload_ts,
        (unsigned long)last_poll_ts,
        (unsigned long)s->timestamp_ms
    );
}

static void build_meta_json(uint32_t now_ms, bool online, char* out, size_t out_size)
{
    snprintf(
        out,
        out_size,
        "{"
        "\"device_id\":\"%s\","
        "\"name\":\"Solar Home Lighting System\","
        "\"team\":\"Photon Phools\","
        "\"device_type\":\"esp32\","
        "\"firmware_version\":\"1.0.0\","
        "\"hardware_version\":\"rev_a\","
        "\"location_label\":\"prototype\","
        "\"created_at\":0,"
        "\"last_seen\":%lu,"
        "\"online\":%s,"
        "\"last_boot_ts\":0,"
        "\"last_boot_reason\":\"unknown\","
        "\"ip_address\":\"\","
        "\"wifi_ssid\":\"\""
        "}",
        DEVICE_ID,
        (unsigned long)now_ms,
        online ? "true" : "false"
    );
}

static void build_ack_json(int seq, const char* status, const char* message, uint32_t now_ms,
                           char* out, size_t out_size)
{
    snprintf(
        out,
        out_size,
        "{"
        "\"last_applied_seq\":%d,"
        "\"status\":\"%s\","
        "\"message\":\"%s\","
        "\"ts\":%lu"
        "}",
        seq,
        status ? status : "idle",
        message ? message : "",
        (unsigned long)now_ms
    );
}

static void build_battery_system_json(const charge_telem_t* t, char* out, size_t out_size)
{
    snprintf(
        out,
        out_size,
        "{"
        "\"active_pack\":\"%c\","
        "\"charging_pack\":\"%c\","
        "\"discharging_pack\":\"%c\","
        "\"route\":%d,"
        "\"switch_state\":\"%s\","
        "\"charging\":%d,"
        "\"fan\":%d,"
        "\"fault\":%d,"
        "\"system_health\":\"%s\","
        "\"solar_voltage\":%.2f,"
        "\"packA_voltage\":%.2f,"
        "\"packB_voltage\":%.2f,"
        "\"packA_percent\":%.2f,"
        "\"packB_percent\":%.2f"
        "}",
        t->active_pack,
        t->charging_pack,
        t->discharging_pack,
        t->route,
        t->switch_state,
        t->charging,
        t->fan,
        t->fault,
        t->system_health,
        t->solar_voltage,
        t->packA_voltage,
        t->packB_voltage,
        t->packA_percent,
        t->packB_percent
    );
}

// ===========================================================
// FIREBASE WRITERS
// ===========================================================
static esp_err_t upload_controls(const network_state_snapshot_t* snapshot)
{
    char path[128];
    char json[256];

    snprintf(path, sizeof(path), "/devices/%s/state/controls", DEVICE_ID);
    build_controls_json(snapshot, json, sizeof(json));

    return http_put_json(path, json);
}

static esp_err_t upload_sensors(const network_state_snapshot_t* snapshot)
{
    char path[128];
    char json[256];

    snprintf(path, sizeof(path), "/devices/%s/state/sensors", DEVICE_ID);
    build_sensors_json(snapshot, json, sizeof(json));

    return http_put_json(path, json);
}

static esp_err_t upload_lights(const network_state_snapshot_t* snapshot)
{
    char path[128];
    char json[256];

    snprintf(path, sizeof(path), "/devices/%s/state/lights", DEVICE_ID);
    build_lights_json(snapshot, json, sizeof(json));

    return http_put_json(path, json);
}

static esp_err_t upload_network_branch(const network_state_snapshot_t* snapshot,
                                       uint32_t last_upload_ts, uint32_t last_poll_ts)
{
    char path[128];
    char json[256];

    snprintf(path, sizeof(path), "/devices/%s/state/network", DEVICE_ID);
    build_network_json(snapshot, json, sizeof(json), last_upload_ts, last_poll_ts);

    return http_put_json(path, json);
}

static esp_err_t upload_meta(bool online, uint32_t now_ms)
{
    char path[128];
    char json[512];

    snprintf(path, sizeof(path), "/devices/%s/meta", DEVICE_ID);
    build_meta_json(now_ms, online, json, sizeof(json));

    return http_put_json(path, json);
}

static esp_err_t upload_ack(int seq, const char* status, const char* message, uint32_t now_ms)
{
    char path[128];
    char json[256];

    snprintf(path, sizeof(path), "/devices/%s/commands/ack", DEVICE_ID);
    build_ack_json(seq, status, message, now_ms, json, sizeof(json));

    return http_put_json(path, json);
}

static esp_err_t upload_battery_system(const charge_telem_t* telem)
{
    char path[128];
    char json[512];

    snprintf(path, sizeof(path), "/devices/%s/state/battery_system", DEVICE_ID);
    build_battery_system_json(telem, json, sizeof(json));

    return http_put_json(path, json);
}

static esp_err_t upload_state_snapshot(const network_state_snapshot_t* snapshot,
                                       uint32_t last_upload_ts, uint32_t last_poll_ts)
{
    esp_err_t err;

    err = upload_controls(snapshot);
    if (err != ESP_OK) return err;

    err = upload_sensors(snapshot);
    if (err != ESP_OK) return err;

    err = upload_lights(snapshot);
    if (err != ESP_OK) return err;

    err = upload_network_branch(snapshot, last_upload_ts, last_poll_ts);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

// ===========================================================
// COMMAND POLLING
// ===========================================================
static bool poll_remote_command(network_command_t* out_cmd)
{
    if (!out_cmd) return false;

    char path[128];
    char buffer[512];

    snprintf(path, sizeof(path), "/devices/%s/commands/active", DEVICE_ID);

    esp_err_t err = http_get(path, buffer, sizeof(buffer));
    if (err != ESP_OK) {
        return false;
    }

    if (strcmp(buffer, "null") == 0 || buffer[0] == '\0') {
        return false;
    }

    network_command_t cmd{};
    int seq = 0;

    if (!json_parse_int_field(buffer, "seq", &seq)) {
        return false;
    }

    if (seq <= s_last_seen_seq) {
        return false;
    }

    cmd.seq = seq;

    json_parse_bool_field(buffer, "porch_manual_on", &cmd.porch_manual_on);
    json_parse_bool_field(buffer, "foyer_manual_on", &cmd.foyer_manual_on);
    json_parse_bool_field(buffer, "security_manual_on", &cmd.security_manual_on);

    json_parse_bool_field(buffer, "porch_auto_enabled", &cmd.porch_auto_enabled);
    json_parse_bool_field(buffer, "foyer_auto_enabled", &cmd.foyer_auto_enabled);
    json_parse_bool_field(buffer, "security_auto_enabled", &cmd.security_auto_enabled);

    *out_cmd = cmd;
    return true;
}

// ===========================================================
// PUBLIC API
// ===========================================================
void network_init(void)
{
    if (s_network_mutex != nullptr) {
        ESP_LOGW(TAG, "network_init() already called");
        return;
    }

    s_network_mutex = xSemaphoreCreateMutex();
    if (s_network_mutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create network mutex");
        return;
    }

    s_snapshot_valid = false;
    s_state_dirty = false;

    s_charge_telem_valid = false;
    s_charge_dirty = false;

    s_has_pending_command = false;
    s_last_seen_seq = -1;
    s_last_acked_seq = -1;

    ESP_LOGI(TAG, "Network module initialized");
}

bool network_has_wifi(void)
{
    return wifi_is_connected();
}

void network_update_state_snapshot(const network_state_snapshot_t* snapshot)
{
    if (!snapshot || s_network_mutex == nullptr) return;

    lock_network();
    s_latest_snapshot = *snapshot;
    s_snapshot_valid = true;
    s_state_dirty = true;
    unlock_network();
}

void network_update_charge_telem(const charge_telem_t* telem)
{
    if (!telem || s_network_mutex == nullptr) return;

    lock_network();
    s_latest_charge_telem = *telem;
    s_charge_telem_valid = true;
    s_charge_dirty = true;
    unlock_network();
}

bool network_try_get_pending_command(network_command_t* out_cmd)
{
    if (!out_cmd || s_network_mutex == nullptr) return false;

    bool has_cmd = false;

    lock_network();
    if (s_has_pending_command) {
        *out_cmd = s_pending_command;
        s_has_pending_command = false;
        has_cmd = true;
    }
    unlock_network();

    return has_cmd;
}

// ===========================================================
// COMMAND TASK
// Polls app commands + uploads ACK
// ===========================================================
void network_command_task(void* arg)
{
    (void)arg;

    TickType_t last_command_poll_tick = 0;
    uint32_t last_successful_command_poll_ts = 0;

    while (true)
    {
        bool wifi_connected = wifi_is_connected();
        uint32_t now_ms = (uint32_t)esp_log_timestamp();

        if (wifi_connected)
        {
            TickType_t now = xTaskGetTickCount();

            if ((now - last_command_poll_tick) >= COMMAND_POLL_PERIOD_TICKS)
            {
                network_command_t cmd{};
                if (poll_remote_command(&cmd))
                {
                    lock_network();
                    s_pending_command = cmd;
                    s_has_pending_command = true;
                    s_last_seen_seq = cmd.seq;
                    unlock_network();

                    last_successful_command_poll_ts = now_ms;
                    ESP_LOGI(TAG, "New remote command received (seq=%d)", cmd.seq);
                }

                last_command_poll_tick = now;
            }

            (void)last_successful_command_poll_ts;

            if (s_last_seen_seq > s_last_acked_seq)
            {
                esp_err_t err = upload_ack(s_last_seen_seq, "applied", "OK", now_ms);
                if (err == ESP_OK) {
                    s_last_acked_seq = s_last_seen_seq;
                    ESP_LOGI(TAG, "Command ACK uploaded for seq=%d", s_last_acked_seq);
                }
            }
        }

        vTaskDelay(COMMAND_LOOP_PERIOD_TICKS);
    }
}

// ===========================================================
// UPLOAD TASK
// Uploads state snapshot, charge telemetry, and meta
// ===========================================================
void network_upload_task(void* arg)
{
    (void)arg;

    bool last_wifi_connected = false;
    bool first_wifi_log = true;

    TickType_t last_state_upload_tick = 0;
    TickType_t last_charge_upload_tick = 0;
    TickType_t last_meta_upload_tick = 0;

    uint32_t last_successful_upload_ts = 0;
    uint32_t last_successful_command_poll_ts = 0;

    while (true)
    {
        bool wifi_connected = wifi_is_connected();
        uint32_t now_ms = (uint32_t)esp_log_timestamp();

        if (first_wifi_log || wifi_connected != last_wifi_connected) {
            ESP_LOGI(TAG, "WiFi %s", wifi_connected ? "CONNECTED" : "DISCONNECTED");
            last_wifi_connected = wifi_connected;
            first_wifi_log = false;
        }

        if (wifi_connected)
        {
            TickType_t now = xTaskGetTickCount();

            if ((now - last_state_upload_tick) >= STATE_UPLOAD_PERIOD_TICKS)
            {
                network_state_snapshot_t snapshot{};
                bool have_snapshot = false;
                bool should_upload = false;

                lock_network();
                if (s_snapshot_valid) {
                    snapshot = s_latest_snapshot;
                    have_snapshot = true;
                    should_upload = s_state_dirty;
                }
                unlock_network();

                if (have_snapshot && should_upload)
                {
                    esp_err_t err = upload_state_snapshot(&snapshot,
                                                          last_successful_upload_ts,
                                                          last_successful_command_poll_ts);
                    if (err == ESP_OK) {
                        last_successful_upload_ts = now_ms;

                        lock_network();
                        s_state_dirty = false;
                        unlock_network();
                    }
                }

                last_state_upload_tick = now;
            }

            if ((now - last_charge_upload_tick) >= CHARGE_UPLOAD_PERIOD_TICKS)
            {
                charge_telem_t telem{};
                bool have_telem = false;
                bool should_upload = false;

                lock_network();
                if (s_charge_telem_valid) {
                    telem = s_latest_charge_telem;
                    have_telem = true;
                    should_upload = s_charge_dirty;
                }
                unlock_network();

                if (have_telem && should_upload)
                {
                    esp_err_t err = upload_battery_system(&telem);
                    if (err == ESP_OK) {
                        lock_network();
                        s_charge_dirty = false;
                        unlock_network();
                    }
                }

                last_charge_upload_tick = now;
            }

            if ((now - last_meta_upload_tick) >= META_UPLOAD_PERIOD_TICKS)
            {
                esp_err_t err = upload_meta(true, now_ms);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Meta heartbeat updated");
                }
                last_meta_upload_tick = now;
            }
        }

        vTaskDelay(UPLOAD_LOOP_PERIOD_TICKS);
    }
}

// ===========================================================
// Compatibility wrapper
// If old code still creates network_task, it spawns both tasks
// then exits.
// ===========================================================
void network_task(void* arg)
{
    (void)arg;

    xTaskCreate(network_command_task, "network_cmd_task", 8192, nullptr, 7, nullptr);
    xTaskCreate(network_upload_task,  "network_up_task",  8192, nullptr, 5, nullptr);

    ESP_LOGI(TAG, "Spawned network command/upload tasks");
    vTaskDelete(NULL);
}