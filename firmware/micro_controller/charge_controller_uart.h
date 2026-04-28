#ifndef CHARGE_CONTROLLER_UART_H
#define CHARGE_CONTROLLER_UART_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =====================
// TELEMETRY STRUCT
// =====================
typedef struct
{
    // ---------------------
    // Battery Pack A
    // ---------------------
    float packA_voltage;
    float packA_current;
    float packA_percent;

    // ---------------------
    // Battery Pack B
    // ---------------------
    float packB_voltage;
    float packB_current;
    float packB_percent;

    // ---------------------
    // Solar
    // ---------------------
    float solar_voltage;
    float solar_current;
    float solar_power;
    float charge_current;

    // ---------------------
    // Load
    // ---------------------
    float load_voltage;
    float load_current;
    float load_power;

    // ---------------------
    // Battery system state
    // ---------------------
    char active_pack;         // 'A', 'B', or 'N'
    char charging_pack;       // 'A', 'B', or 'N'
    char discharging_pack;    // 'A', 'B', or 'N'

    char switch_state[24];    // "all_off", "charge_A_load_B", etc.
    char system_health[16];   

    // ---------------------
    // Raw status/debug fields
    // ---------------------
    int route;                // 0, 1, 2
    int charging;             // 0 or 1
    int fan;                  // 0 or 1
    int fault;                // 0 or 1 (or future fault code)

    // ---------------------
    // Telemetry validity
    // ---------------------
    bool valid;

} charge_telem_t;


// =====================
// FUNCTION PROTOTYPES
// =====================

// Initialize UART used to receive charge controller telemetry
void cc_uart_init(void);

// Read and parse one streamed telemetry line if available
// Returns true only when a full valid telemetry packet was parsed
bool cc_read_telemetry(charge_telem_t *out_telem);

#ifdef __cplusplus
}
#endif

#endif // CHARGE_CONTROLLER_UART_H