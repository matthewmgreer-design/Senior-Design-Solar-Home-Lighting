#include "charge_controller_uart.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define UART_PORT       UART_NUM_2
#define TX_PIN          GPIO_NUM_11
#define RX_PIN          GPIO_NUM_36
#define BUF_SIZE        512
#define LINE_BUF_SIZE   512

// Set to 1 if you want to dump every received byte
#define CC_RAW_BYTE_DEBUG 0

static const char *TAG = "CC_UART";

static uint8_t rx_buffer[BUF_SIZE];
static char line_buffer[LINE_BUF_SIZE];
static size_t line_index = 0;

// =====================
// INTERNAL HELPERS
// =====================
static void cc_clear_telem(charge_telem_t *t)
{
    memset(t, 0, sizeof(charge_telem_t));

    t->active_pack = 'N';
    t->charging_pack = 'N';
    t->discharging_pack = 'N';

    strcpy(t->switch_state, "unknown");
    strcpy(t->system_health, "ok");

    t->route = 0;
    t->charging = 0;
    t->fan = 0;
    t->fault = 0;
}

static void cc_derive_system_state(charge_telem_t *t)
{
    switch (t->route)
    {
        case 0:
            t->active_pack = 'N';
            t->charging_pack = 'N';
            t->discharging_pack = 'N';
            strcpy(t->switch_state, "all_off");
            break;

        case 1:
            t->active_pack = 'B';
            t->charging_pack = 'A';
            t->discharging_pack = 'B';
            strcpy(t->switch_state, "charge_A_load_B");
            break;

        case 2:
            t->active_pack = 'A';
            t->charging_pack = 'B';
            t->discharging_pack = 'A';
            strcpy(t->switch_state, "charge_B_load_A");
            break;

        default:
            t->active_pack = 'N';
            t->charging_pack = 'N';
            t->discharging_pack = 'N';
            strcpy(t->switch_state, "unknown");
            break;
    }

    if (t->fault)
    {
        strcpy(t->system_health, "fault");
    }
    else if (t->charging)
    {
        strcpy(t->system_health, "charging");
    }
    else
    {
        strcpy(t->system_health, "normal");
    }
}

static bool cc_parse_token(char *token, charge_telem_t *t)
{
    char *equals = strchr(token, '=');
    if (equals == NULL)
    {
        ESP_LOGW(TAG, "Skipping malformed token: %s", token);
        return false;
    }

    *equals = '\0';
    char *key = token;
    char *value = equals + 1;

    if (strcmp(key, "solar") == 0)
    {
        t->solar_voltage = strtof(value, NULL);
        return true;
    }
    else if (strcmp(key, "bat1") == 0)
    {
        t->packA_voltage = strtof(value, NULL);
        return true;
    }
    else if (strcmp(key, "bat2") == 0)
    {
        t->packB_voltage = strtof(value, NULL);
        return true;
    }
    else if (strcmp(key, "soc1") == 0)
    {
        t->packA_percent = strtof(value, NULL);
        return true;
    }
    else if (strcmp(key, "soc2") == 0)
    {
        t->packB_percent = strtof(value, NULL);
        return true;
    }
    else if (strcmp(key, "route") == 0)
    {
        t->route = atoi(value);
        return true;
    }
    else if (strcmp(key, "chg") == 0)
    {
        t->charging = atoi(value);
        return true;
    }
    else if (strcmp(key, "fan") == 0)
    {
        t->fan = atoi(value);
        return true;
    }
    else if (strcmp(key, "fault") == 0)
    {
        t->fault = atoi(value);
        return true;
    }

    ESP_LOGW(TAG, "Unknown token key: %s=%s", key, value);
    return false;
}

static bool cc_parse_line(const char *line, charge_telem_t *out_telem)
{
    if (line == NULL || out_telem == NULL)
    {
        ESP_LOGE(TAG, "cc_parse_line received null pointer");
        return false;
    }

    if (strlen(line) == 0)
    {
        ESP_LOGW(TAG, "Empty telemetry line received");
        return false;
    }

    charge_telem_t temp;
    cc_clear_telem(&temp);

    char working_copy[LINE_BUF_SIZE];
    strncpy(working_copy, line, sizeof(working_copy) - 1);
    working_copy[sizeof(working_copy) - 1] = '\0';

    bool found_valid_field = false;

    char *token = strtok(working_copy, ",");
    while (token != NULL)
    {
        if (cc_parse_token(token, &temp))
        {
            found_valid_field = true;
        }
        token = strtok(NULL, ",");
    }

    if (!found_valid_field)
    {
        ESP_LOGW(TAG, "[CC DROP] No valid telemetry fields found");
        return false;
    }

    cc_derive_system_state(&temp);
    *out_telem = temp;

    ESP_LOGI(TAG,
             "[CC PARSED] solar=%.2fV bat1=%.2fV bat2=%.2fV soc1=%.1f%% soc2=%.1f%% route=%d chg=%d fan=%d fault=%d active=%c charging=%c discharging=%c switch=%s health=%s",
             temp.solar_voltage,
             temp.packA_voltage,
             temp.packB_voltage,
             temp.packA_percent,
             temp.packB_percent,
             temp.route,
             temp.charging,
             temp.fan,
             temp.fault,
             temp.active_pack,
             temp.charging_pack,
             temp.discharging_pack,
             temp.switch_state,
             temp.system_health);

    return true;
}

// =====================
// INIT
// =====================
void cc_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "Charge controller UART initialized on UART%d TX=%d RX=%d baud=%d",
             UART_PORT, TX_PIN, RX_PIN, uart_config.baud_rate);
}

// =====================
// READ + PARSE STREAMED TELEMETRY
// =====================
bool cc_read_telemetry(charge_telem_t *out_telem)
{
    if (out_telem == NULL)
    {
        ESP_LOGE(TAG, "cc_read_telemetry called with null output pointer");
        return false;
    }

    int len = uart_read_bytes(UART_PORT, rx_buffer, BUF_SIZE - 1, pdMS_TO_TICKS(20));
    if (len <= 0)
    {
        return false;
    }

#if CC_RAW_BYTE_DEBUG
    ESP_LOGI(TAG, "[CC RX LEN] %d bytes", len);
    for (int i = 0; i < len; i++)
    {
        char c = (char)rx_buffer[i];
        ESP_LOGI(TAG, "[CC RX BYTE %d] 0x%02X '%c'",
                 i,
                 (unsigned char)c,
                 (c >= 32 && c <= 126) ? c : '.');
    }
#endif

    for (int i = 0; i < len; i++)
    {
        char c = (char)rx_buffer[i];

        // Treat either CR or LF as end-of-line
        if (c == '\r' || c == '\n')
        {
            if (line_index == 0)
            {
                continue;
            }

            line_buffer[line_index] = '\0';

            ESP_LOGI(TAG, "[CC RAW] %s", line_buffer);

            bool parsed = cc_parse_line(line_buffer, out_telem);

            line_index = 0;
            memset(line_buffer, 0, sizeof(line_buffer));

            if (parsed)
            {
                ESP_LOGI(TAG, "[CC ACCEPTED] Telemetry packet accepted by MCU");
                return true;
            }
            else
            {
                ESP_LOGW(TAG, "[CC DROP] Failed to parse telemetry line");
            }

            continue;
        }
        else
        {
            if (line_index < (LINE_BUF_SIZE - 1))
            {
                line_buffer[line_index++] = c;
            }
            else
            {
                line_buffer[line_index] = '\0';
                ESP_LOGW(TAG, "[CC OVERFLOW] partial packet: %s", line_buffer);
                ESP_LOGW(TAG, "Line buffer overflow, clearing partial packet");
                line_index = 0;
                memset(line_buffer, 0, sizeof(line_buffer));
            }
        }
    }

    return false;
}