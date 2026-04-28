#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "charge_controller_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// ===========================================================
// Network-facing snapshot of current MCU/system truth
// ===========================================================
typedef struct
{
    bool porch_manual_on;
    bool foyer_manual_on;
    bool security_manual_on;

    bool porch_auto_enabled;
    bool foyer_auto_enabled;
    bool security_auto_enabled;

    int  lux;
    bool dark;
    bool pir_outside;
    bool pir_inside;

    bool porch_on;
    bool foyer_on;
    bool security_on;

    // Optional / future-facing fields
    int  battery_percent;      // -1 if unavailable
    bool battery_charging;     // false if unknown / not charging
    bool wifi_connected;       // local WiFi status snapshot
    uint32_t timestamp_ms;     // esp_log_timestamp() or similar
} network_state_snapshot_t;

// ===========================================================
// Remote command payload from app -> Firebase -> MCU
// ===========================================================
typedef struct
{
    int  seq;
    bool porch_manual_on;
    bool foyer_manual_on;
    bool security_manual_on;

    bool porch_auto_enabled;
    bool foyer_auto_enabled;
    bool security_auto_enabled;
} network_command_t;

// ===========================================================
// Init / tasks
// ===========================================================
void network_init(void);
void network_task(void* arg);   // wrapper, can be left unused if desired

// Final-product split tasks
void network_command_task(void* arg);
void network_upload_task(void* arg);

// ===========================================================
// State upload interface
// ===========================================================
void network_update_state_snapshot(const network_state_snapshot_t* snapshot);
void network_update_charge_telem(const charge_telem_t* telem);

// ===========================================================
// Command interface
// ===========================================================
bool network_try_get_pending_command(network_command_t* out_cmd);

// ===========================================================
// Optional helpers
// ===========================================================
bool network_has_wifi(void);

#ifdef __cplusplus
}
#endif