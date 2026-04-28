#include <stdio.h>
#include <math.h>
#include <string.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "nvs_flash.h"
}

#define TAG "DUAL_BATT_CTRL"

// ---------------- ADC / GPIO ----------------
#define PIN_SOLAR     ADC1_CHANNEL_6   // GPIO34
#define PIN_BAT1      ADC1_CHANNEL_7   // GPIO35
#define PIN_BAT2      ADC1_CHANNEL_4   // GPIO32
#define PIN_CURRENT   ADC1_CHANNEL_3   // GPIO39
#define PIN_VREF      ADC1_CHANNEL_0   // GPIO36

#define GPIO_CHARGE_A_LOAD_B GPIO_NUM_26
#define GPIO_CHARGE_B_LOAD_A GPIO_NUM_33
#define GPIO_FAN_RELAY       GPIO_NUM_14

// ---------------- UART ----------------
#define UART_PORT_NUM        UART_NUM_2
#define UART_TX_GPIO         GPIO_NUM_16
#define UART_RX_GPIO         GPIO_NUM_17
#define UART_BAUD_RATE       115200
#define UART_TX_BUF_SIZE     1024
#define UART_RX_BUF_SIZE     1024
#define UART_TASK_PERIOD_MS  1000

// ---------------- ADC constants ----------------
#define ADC_MAX             4095.0f
#define ADC_REF_VOLTAGE     3.3f

#define SOLAR_RATIO         ((68.0f + 10.0f) / 10.0f)
#define BAT_RATIO           ((34.8f + 10.0f) / 10.0f)

#define CURRENT_SENSITIVITY 0.044f   // 44 mV/A

// ---------------- I2C / LTC4015 ----------------
#define I2C_PORT            I2C_NUM_0
#define I2C_SDA_GPIO        GPIO_NUM_23
#define I2C_SCL_GPIO        GPIO_NUM_22
#define I2C_FREQ_HZ         100000
#define LTC4015_ADDR        0x68

#define REG_CONFIG_BITS       0x14
#define REG_ICHARGE_TARGET    0x1A
#define REG_CHARGER_STATE     0x34
#define REG_CHARGE_STATUS     0x35
#define REG_SYSTEM_STATUS     0x39
#define REG_BATTERY_VOLTAGE   0x3A
#define REG_IBAT              0x3D
#define REG_ICHARGE_DAC       0x44

#define LTC4015_CFG_SUSPEND_CHARGER (1u << 8)
#define RSNSB_OHMS 0.006f

// ---------------- Timing ----------------
#define ADC_TASK_PERIOD_MS       100
#define SOC_TASK_PERIOD_MS       1000
#define LTC_TASK_PERIOD_MS       500
#define CONTROL_TASK_PERIOD_MS   200

#define SWITCH_OFF_DELAY_MS      200
#define SWITCH_SETTLE_DELAY_MS   300
#define MIN_ROUTE_DWELL_TIME_MS  60000
#define SOC_HOLD_AFTER_SWITCH_MS 5000

// ---------------- Thresholds ----------------
#define SOLAR_ON_THRESHOLD_V     8.0f
#define SOLAR_OFF_THRESHOLD_V    6.0f

#define SOC_LOW_THRESHOLD              30.0f
#define SOC_RECOVERY_THRESHOLD         50.0f
#define SOC_REST_CURRENT_THRESHOLD_A   0.10f
#define SOC_SWITCH_DEADBAND            5.0f

#define LOAD_ONLY_SWITCH_THRESHOLD     10.0f
#define LOAD_ONLY_RECOVERY_DEADBAND    3.0f

#define LOAD_SAG_COMP_V_PER_A          0.08f
#define CHARGE_SURFACE_COMP_V_PER_A    0.15f

#define BAT_VALID_MIN_V          10.0f
#define BAT_VALID_MAX_V          15.5f
#define SOLAR_VALID_MAX_V        25.0f

#define FAN_CHARGING_CURRENT_THRESHOLD_A 0.03f

// ---------------- Battery config ----------------
#define LEAD_ACID_CELLS          6.0f

// ---------------- Enums ----------------
enum PowerRoute
{
    ROUTE_ALL_OFF = 0,
    ROUTE_CHARGE_A_LOAD_B,
    ROUTE_CHARGE_B_LOAD_A
};

enum SystemState
{
    STATE_STARTUP = 0,
    STATE_NORMAL,
    STATE_LOW_SOLAR,
    STATE_SWITCHING,
    STATE_RECOVERY_LOCK,
    STATE_FAULT
};

// ---------------- Shared data ----------------
static float solar_voltage = 0.0f;
static float bat1_voltage = 0.0f;
static float bat2_voltage = 0.0f;
static float current_amps = 0.0f;

static float solar_voltage_filt = 0.0f;
static float bat1_voltage_filt = 12.0f;
static float bat2_voltage_filt = 12.0f;
static float current_amps_filt = 0.0f;

static float soc_bat1 = 100.0f;
static float soc_bat2 = 100.0f;
static float soc_bat1_raw = 100.0f;
static float soc_bat2_raw = 100.0f;

static float ltc_battery_voltage = 0.0f;
static float ltc_charge_current = 0.0f;

static uint16_t ltc_raw_vbat = 0;
static uint16_t ltc_raw_ibat = 0;
static uint16_t ltc_config_bits = 0;
static uint16_t ltc_system_status = 0;
static uint16_t ltc_charger_state = 0;
static uint16_t ltc_charge_status = 0;
static uint16_t ltc_icharge_dac = 0;

static bool solar_available = false;
static bool charging_enabled = false;
static bool fault_present = false;
static bool fan_relay_on = false;

static PowerRoute current_route = ROUTE_ALL_OFF;
static SystemState system_state = STATE_STARTUP;

// recovery lock: prevents ping-pong when both batteries are low
static bool recovery_lock_active = false;
static PowerRoute locked_route = ROUTE_ALL_OFF;

static TickType_t last_route_switch_tick = 0;
static TickType_t soc_hold_until_tick = 0;

static SemaphoreHandle_t data_mutex = NULL;

// ---------------- Utility ----------------
const char* route_to_string(PowerRoute route)
{
    switch (route)
    {
        case ROUTE_ALL_OFF:          return "ALL_OFF";
        case ROUTE_CHARGE_A_LOAD_B:  return "CHARGE_A_LOAD_B";
        case ROUTE_CHARGE_B_LOAD_A:  return "CHARGE_B_LOAD_A";
        default:                     return "UNKNOWN_ROUTE";
    }
}

const char* state_to_string(SystemState state)
{
    switch (state)
    {
        case STATE_STARTUP:       return "STARTUP";
        case STATE_NORMAL:        return "NORMAL";
        case STATE_LOW_SOLAR:     return "LOW_SOLAR";
        case STATE_SWITCHING:     return "SWITCHING";
        case STATE_RECOVERY_LOCK: return "RECOVERY_LOCK";
        case STATE_FAULT:         return "FAULT";
        default:                  return "UNKNOWN_STATE";
    }
}

bool route_dwell_time_elapsed()
{
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed = now - last_route_switch_tick;
    return elapsed >= pdMS_TO_TICKS(MIN_ROUTE_DWELL_TIME_MS);
}

bool ltc_is_in_active_charge_state(uint16_t charger_state)
{
    return ((charger_state & (1 << 10)) ||   // equalize
            (charger_state & (1 << 9))  ||   // absorb
            (charger_state & (1 << 7))  ||   // precharge
            (charger_state & (1 << 6)));     // cc/cv
}

bool charger_is_actively_charging()
{
    bool not_suspended = ((ltc_config_bits & LTC4015_CFG_SUSPEND_CHARGER) == 0);
    bool active_state = ltc_is_in_active_charge_state(ltc_charger_state);
    bool positive_current = (ltc_charge_current > FAN_CHARGING_CURRENT_THRESHOLD_A);
    bool charger_ok = !!(ltc_system_status & (1 << 13));

    return not_suspended && charger_ok && active_state && positive_current;
}

void update_fan_output()
{
    bool should_turn_on = charger_is_actively_charging();

    if (should_turn_on != fan_relay_on)
    {
        fan_relay_on = should_turn_on;
        gpio_set_level(GPIO_FAN_RELAY, fan_relay_on ? 1 : 0);
        ESP_LOGI(TAG, "Fan relay -> %s", fan_relay_on ? "ON" : "OFF");
    }
}

// ---------------- ADC helpers ----------------
float read_adc_voltage(adc1_channel_t channel)
{
    int raw_sum = 0;
    const int samples = 16;

    for (int i = 0; i < samples; i++)
    {
        raw_sum += adc1_get_raw(channel);
    }

    float raw_avg = (float)raw_sum / samples;
    return (raw_avg / ADC_MAX) * ADC_REF_VOLTAGE;
}

// ---------------- UART ----------------
void uart_init_custom()
{
    uart_config_t uart_config = {};
    uart_config.baud_rate = UART_BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// ---------------- I2C ----------------
void i2c_master_init()
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_GPIO;
    conf.scl_io_num = I2C_SCL_GPIO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
}

esp_err_t ltc4015_write_reg(uint8_t reg, uint16_t value)
{
    uint8_t data[3] = {
        reg,
        (uint8_t)(value & 0xFF),
        (uint8_t)((value >> 8) & 0xFF)
    };

    return i2c_master_write_to_device(I2C_PORT, LTC4015_ADDR, data, 3, pdMS_TO_TICKS(100));
}

esp_err_t ltc4015_read_reg(uint8_t reg, uint16_t *value)
{
    uint8_t data[2] = {0};

    esp_err_t err = i2c_master_write_read_device(
        I2C_PORT,
        LTC4015_ADDR,
        &reg,
        1,
        data,
        2,
        pdMS_TO_TICKS(100)
    );

    if (err == ESP_OK)
    {
        *value = (uint16_t)(data[0] | (data[1] << 8));
    }

    return err;
}

// ---------------- LTC4015 helpers ----------------
bool ltc4015_enable_charging(bool enable)
{
    uint16_t cfg = 0;

    if (ltc4015_read_reg(REG_CONFIG_BITS, &cfg) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read CONFIG_BITS");
        return false;
    }

    if (enable)
        cfg &= ~LTC4015_CFG_SUSPEND_CHARGER;
    else
        cfg |= LTC4015_CFG_SUSPEND_CHARGER;

    if (ltc4015_write_reg(REG_CONFIG_BITS, cfg) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write CONFIG_BITS");
        return false;
    }

    uint16_t verify = 0;
    if (ltc4015_read_reg(REG_CONFIG_BITS, &verify) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to verify CONFIG_BITS");
        return false;
    }

    xSemaphoreTake(data_mutex, portMAX_DELAY);
    ltc_config_bits = verify;
    charging_enabled = enable;
    xSemaphoreGive(data_mutex);

    ESP_LOGI(TAG, "Charging %s | CONFIG_BITS=0x%04X",
             enable ? "ENABLED" : "SUSPENDED", verify);

    update_fan_output();
    return true;
}

float ltc4015_get_battery_voltage()
{
    uint16_t raw = 0;
    if (ltc4015_read_reg(REG_BATTERY_VOLTAGE, &raw) == ESP_OK)
    {
        ltc_raw_vbat = raw;
        return raw * 128.176e-6f * LEAD_ACID_CELLS;
    }
    return -1.0f;
}

float ltc4015_get_battery_current()
{
    uint16_t raw_u16 = 0;
    if (ltc4015_read_reg(REG_IBAT, &raw_u16) != ESP_OK)
        return -1000.0f;

    ltc_raw_ibat = raw_u16;

    int16_t raw = (int16_t)raw_u16;
    return ((float)raw * 1.46487e-6f) / RSNSB_OHMS;
}

float ltc4015_icharge_target_to_amps(uint16_t reg_value)
{
    return ((float)((reg_value & 0x1F) + 1) * 1.0e-3f) / RSNSB_OHMS;
}

float ltc4015_icharge_dac_to_amps(uint16_t reg_value)
{
    return ((float)((reg_value & 0x1F) + 1) * 1.0e-3f) / RSNSB_OHMS;
}

void ltc4015_read_config_bits()
{
    uint16_t cfg = 0;
    if (ltc4015_read_reg(REG_CONFIG_BITS, &cfg) == ESP_OK)
    {
        ltc_config_bits = cfg;
    }
}

void ltc4015_read_status()
{
    uint16_t status = 0;
    if (ltc4015_read_reg(REG_SYSTEM_STATUS, &status) == ESP_OK)
    {
        ltc_system_status = status;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read LTC4015 system status");
        fault_present = true;
    }
}

const char* ltc4015_charge_status_to_string(uint16_t status)
{
    if (status & (1 << 3)) return "VIN_UVCL";
    if (status & (1 << 2)) return "IIN_LIMIT";
    if (status & (1 << 1)) return "CONSTANT_CURRENT";
    if (status & (1 << 0)) return "CONSTANT_VOLTAGE";
    return "IDLE_OR_UNKNOWN";
}

void ltc4015_log_charger_state_bits(uint16_t state)
{
    if (state & (1 << 10)) ESP_LOGI(TAG, "STATE: EQUALIZE_CHARGE");
    if (state & (1 << 9))  ESP_LOGI(TAG, "STATE: ABSORB_CHARGE");
    if (state & (1 << 8))  ESP_LOGW(TAG, "STATE: CHARGER_SUSPENDED");
    if (state & (1 << 7))  ESP_LOGI(TAG, "STATE: PRECHARGE");
    if (state & (1 << 6))  ESP_LOGI(TAG, "STATE: CC_CV_CHARGE");
    if (state & (1 << 5))  ESP_LOGW(TAG, "STATE: NTC_PAUSE");
    if (state & (1 << 4))  ESP_LOGI(TAG, "STATE: TIMER_TERM");
    if (state & (1 << 3))  ESP_LOGI(TAG, "STATE: C_OVER_X_TERM");
    if (state & (1 << 2))  ESP_LOGE(TAG, "STATE: MAX_CHARGE_TIME_FAULT");
    if (state & (1 << 1))  ESP_LOGE(TAG, "STATE: BAT_MISSING_FAULT");
    if (state & (1 << 0))  ESP_LOGE(TAG, "STATE: BAT_SHORT_FAULT");
}

void ltc4015_log_system_status_bits(uint16_t sys)
{
    ESP_LOGI(TAG,
        "SYS: charger_enabled=%d mppt_en_pin=%d equalize_req=%d drvcc_good=%d "
        "cell_count_error=%d ok_to_charge=%d no_rt=%d thermal_shutdown=%d "
        "vin_ovlo=%d vin_gt_vbat=%d intvcc_gt_4p3v=%d intvcc_gt_2p8v=%d",
        !!(sys & (1 << 13)),
        !!(sys & (1 << 11)),
        !!(sys & (1 << 10)),
        !!(sys & (1 << 9)),
        !!(sys & (1 << 8)),
        !!(sys & (1 << 6)),
        !!(sys & (1 << 5)),
        !!(sys & (1 << 4)),
        !!(sys & (1 << 3)),
        !!(sys & (1 << 2)),
        !!(sys & (1 << 1)),
        !!(sys & (1 << 0)));
}

void ltc4015_debug_full()
{
    uint16_t charger_state = 0;
    uint16_t charge_status = 0;
    uint16_t system_status = 0;
    uint16_t icharge_dac = 0;
    uint16_t icharge_target = 0;

    if (ltc4015_read_reg(REG_CHARGER_STATE, &charger_state) != ESP_OK ||
        ltc4015_read_reg(REG_CHARGE_STATUS, &charge_status) != ESP_OK ||
        ltc4015_read_reg(REG_SYSTEM_STATUS, &system_status) != ESP_OK ||
        ltc4015_read_reg(REG_ICHARGE_DAC, &icharge_dac) != ESP_OK ||
        ltc4015_read_reg(REG_ICHARGE_TARGET, &icharge_target) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read LTC4015 extended debug registers");
        fault_present = true;
        return;
    }

    xSemaphoreTake(data_mutex, portMAX_DELAY);
    ltc_charger_state = charger_state;
    ltc_charge_status = charge_status;
    ltc_system_status = system_status;
    ltc_icharge_dac = icharge_dac;
    xSemaphoreGive(data_mutex);

    ESP_LOGI(TAG,
             "LTC DBG | CHARGER_STATE=0x%04X | CHARGE_STATUS=0x%04X (%s) | SYSTEM_STATUS=0x%04X | ICHARGE_DAC=0x%04X | ICHARGE_TARGET=0x%04X",
             charger_state,
             charge_status,
             ltc4015_charge_status_to_string(charge_status),
             system_status,
             icharge_dac,
             icharge_target);

    ESP_LOGI(TAG,
             "LTC CURR | IBAT=%.3f A | ICHARGE_DAC=%.3f A | ICHARGE_TARGET=%.3f A",
             ltc_charge_current,
             ltc4015_icharge_dac_to_amps(icharge_dac),
             ltc4015_icharge_target_to_amps(icharge_target));

    ltc4015_log_charger_state_bits(charger_state);
    ltc4015_log_system_status_bits(system_status);

    update_fan_output();
}

// ---------------- SOC helpers ----------------
float compute_soc_lead_acid_resting(float voltage)
{
    if (voltage >= 12.73f) return 100.0f;
    if (voltage >= 12.62f) return 90.0f + (voltage - 12.62f) * (10.0f / (12.73f - 12.62f));
    if (voltage >= 12.50f) return 80.0f + (voltage - 12.50f) * (10.0f / (12.62f - 12.50f));
    if (voltage >= 12.37f) return 70.0f + (voltage - 12.37f) * (10.0f / (12.50f - 12.37f));
    if (voltage >= 12.24f) return 60.0f + (voltage - 12.24f) * (10.0f / (12.37f - 12.24f));
    if (voltage >= 12.10f) return 50.0f + (voltage - 12.10f) * (10.0f / (12.24f - 12.10f));
    if (voltage >= 11.96f) return 40.0f + (voltage - 11.96f) * (10.0f / (12.10f - 11.96f));
    if (voltage >= 11.81f) return 30.0f + (voltage - 11.81f) * (10.0f / (11.96f - 11.81f));
    if (voltage >= 11.66f) return 20.0f + (voltage - 11.66f) * (10.0f / (11.81f - 11.66f));
    if (voltage >= 11.51f) return 10.0f + (voltage - 11.51f) * (10.0f / (11.66f - 11.51f));
    if (voltage >= 11.31f) return (voltage - 11.31f) * (10.0f / (11.51f - 11.31f));
    return 0.0f;
}

float estimate_rest_voltage(float measured_v, float battery_current_a, bool is_charging_battery, bool is_load_battery)
{
    float corrected_v = measured_v;

    if (is_load_battery)
    {
        corrected_v += fabsf(battery_current_a) * LOAD_SAG_COMP_V_PER_A;
    }

    if (is_charging_battery)
    {
        corrected_v -= fabsf(battery_current_a) * CHARGE_SURFACE_COMP_V_PER_A;
    }

    return corrected_v;
}

// ---------------- Validation ----------------
bool voltages_are_valid(float solar_v, float b1_v, float b2_v)
{
    if (solar_v < 0.0f || solar_v > SOLAR_VALID_MAX_V) return false;
    if (b1_v < BAT_VALID_MIN_V || b1_v > BAT_VALID_MAX_V) return false;
    if (b2_v < BAT_VALID_MIN_V || b2_v > BAT_VALID_MAX_V) return false;
    return true;
}

void update_solar_availability(float solar_v)
{
    if (solar_v > SOLAR_ON_THRESHOLD_V)
        solar_available = true;
    else if (solar_v < SOLAR_OFF_THRESHOLD_V)
        solar_available = false;
}

// ---------------- Route meaning helpers ----------------
float get_load_battery_soc_for_route(PowerRoute route)
{
    switch (route)
    {
        case ROUTE_CHARGE_A_LOAD_B: return soc_bat2;
        case ROUTE_CHARGE_B_LOAD_A: return soc_bat1;
        default: return 0.0f;
    }
}

float get_charge_battery_soc_for_route(PowerRoute route)
{
    switch (route)
    {
        case ROUTE_CHARGE_A_LOAD_B: return soc_bat1;
        case ROUTE_CHARGE_B_LOAD_A: return soc_bat2;
        default: return 0.0f;
    }
}

// ---------------- GPIO route control ----------------
void apply_route_gpio(PowerRoute route)
{
    if (route == current_route)
        return;

    switch (route)
    {
        case ROUTE_ALL_OFF:
            gpio_set_level(GPIO_CHARGE_A_LOAD_B, 0);
            gpio_set_level(GPIO_CHARGE_B_LOAD_A, 0);
            break;

        case ROUTE_CHARGE_A_LOAD_B:
            gpio_set_level(GPIO_CHARGE_A_LOAD_B, 1);
            gpio_set_level(GPIO_CHARGE_B_LOAD_A, 0);
            break;

        case ROUTE_CHARGE_B_LOAD_A:
            gpio_set_level(GPIO_CHARGE_A_LOAD_B, 0);
            gpio_set_level(GPIO_CHARGE_B_LOAD_A, 1);
            break;
    }

    current_route = route;
    last_route_switch_tick = xTaskGetTickCount();
    soc_hold_until_tick = last_route_switch_tick + pdMS_TO_TICKS(SOC_HOLD_AFTER_SWITCH_MS);

    // Reset filters so they do not lag badly after rerouting
    bat1_voltage_filt = bat1_voltage;
    bat2_voltage_filt = bat2_voltage;
    solar_voltage_filt = solar_voltage;
    current_amps_filt = current_amps;

    ESP_LOGI(TAG, "Route -> %s", route_to_string(route));
}

bool set_route_safely(PowerRoute new_route)
{
    if (new_route == current_route)
        return true;

    system_state = STATE_SWITCHING;
    ESP_LOGW(TAG, "Switching route: %s -> %s",
             route_to_string(current_route),
             route_to_string(new_route));

    ltc4015_enable_charging(false);

    apply_route_gpio(ROUTE_ALL_OFF);
    vTaskDelay(pdMS_TO_TICKS(SWITCH_OFF_DELAY_MS));

    apply_route_gpio(new_route);
    vTaskDelay(pdMS_TO_TICKS(SWITCH_SETTLE_DELAY_MS));

    if (solar_available && !fault_present && new_route != ROUTE_ALL_OFF)
    {
        ltc4015_enable_charging(true);
    }

    return true;
}

// ---------------- Route decision logic ----------------
PowerRoute choose_startup_route(float soc1, float soc2)
{
    if (soc1 <= soc2)
        return ROUTE_CHARGE_A_LOAD_B;
    else
        return ROUTE_CHARGE_B_LOAD_A;
}

PowerRoute choose_route_from_soc(float soc1, float soc2)
{
    if (soc1 < SOC_LOW_THRESHOLD && soc2 >= SOC_LOW_THRESHOLD)
        return ROUTE_CHARGE_A_LOAD_B;

    if (soc2 < SOC_LOW_THRESHOLD && soc1 >= SOC_LOW_THRESHOLD)
        return ROUTE_CHARGE_B_LOAD_A;

    if (fabsf(soc1 - soc2) < SOC_SWITCH_DEADBAND)
    {
        if (current_route == ROUTE_ALL_OFF)
            return choose_startup_route(soc1, soc2);

        return current_route;
    }

    if (current_route != ROUTE_ALL_OFF && !route_dwell_time_elapsed())
    {
        return current_route;
    }

    if (soc1 <= soc2)
        return ROUTE_CHARGE_A_LOAD_B;
    else
        return ROUTE_CHARGE_B_LOAD_A;
}

PowerRoute choose_load_only_route(float soc1, float soc2)
{
    // In low/no solar, keep one battery always feeding the load.
    // ROUTE_CHARGE_A_LOAD_B -> load on B
    // ROUTE_CHARGE_B_LOAD_A -> load on A

    if (current_route == ROUTE_CHARGE_A_LOAD_B)
    {
        // Load on B
        if (soc2 > LOAD_ONLY_SWITCH_THRESHOLD)
            return current_route;

        if (soc1 > soc2 + LOAD_ONLY_RECOVERY_DEADBAND)
            return ROUTE_CHARGE_B_LOAD_A;

        return current_route;
    }

    if (current_route == ROUTE_CHARGE_B_LOAD_A)
    {
        // Load on A
        if (soc1 > LOAD_ONLY_SWITCH_THRESHOLD)
            return current_route;

        if (soc2 > soc1 + LOAD_ONLY_RECOVERY_DEADBAND)
            return ROUTE_CHARGE_A_LOAD_B;

        return current_route;
    }

    // If currently ALL_OFF, pick higher SOC battery for load
    if (soc1 >= soc2)
        return ROUTE_CHARGE_B_LOAD_A;  // load A
    else
        return ROUTE_CHARGE_A_LOAD_B;  // load B
}

bool should_enter_recovery_lock(float soc1, float soc2)
{
    return (soc1 < SOC_LOW_THRESHOLD && soc2 < SOC_LOW_THRESHOLD);
}

bool recovery_lock_cleared(PowerRoute route, float soc1, float soc2)
{
    float charging_soc = 0.0f;

    if (route == ROUTE_CHARGE_A_LOAD_B)
        charging_soc = soc1;
    else if (route == ROUTE_CHARGE_B_LOAD_A)
        charging_soc = soc2;
    else
        return false;

    return (charging_soc >= SOC_RECOVERY_THRESHOLD);
}

// ---------------- Tasks ----------------
void adc_task(void *pvParameters)
{
    const float alpha_v = 0.15f;
    const float alpha_i = 0.10f;

    while (1)
    {
        float local_solar = read_adc_voltage(PIN_SOLAR) * SOLAR_RATIO;
        float local_bat1  = read_adc_voltage(PIN_BAT1) * BAT_RATIO;
        float local_bat2  = read_adc_voltage(PIN_BAT2) * BAT_RATIO;

        float vout = read_adc_voltage(PIN_CURRENT);
        float vref = read_adc_voltage(PIN_VREF);
        float local_current = (vout - vref) / CURRENT_SENSITIVITY;

        xSemaphoreTake(data_mutex, portMAX_DELAY);

        solar_voltage = local_solar;
        bat1_voltage = local_bat1;
        bat2_voltage = local_bat2;
        current_amps = local_current;

        solar_voltage_filt += alpha_v * (local_solar - solar_voltage_filt);
        bat1_voltage_filt += alpha_v * (local_bat1 - bat1_voltage_filt);
        bat2_voltage_filt += alpha_v * (local_bat2 - bat2_voltage_filt);
        current_amps_filt += alpha_i * (local_current - current_amps_filt);

        update_solar_availability(local_solar);

        xSemaphoreGive(data_mutex);

        ESP_LOGI(TAG,
                 "ADC | Solar: %.2f V | BatA: %.2f V | BatB: %.2f V | I: %.2f A | FiltA: %.2f | FiltB: %.2f | FiltI: %.2f",
                 local_solar, local_bat1, local_bat2, local_current,
                 bat1_voltage_filt, bat2_voltage_filt, current_amps_filt);

        vTaskDelay(pdMS_TO_TICKS(ADC_TASK_PERIOD_MS));
    }
}

void soc_task(void *pvParameters)
{
    while (1)
    {
        float local_bat1_filt, local_bat2_filt;
        float local_current_filt, local_ltc_ibat;
        float local_soc1_prev, local_soc2_prev;
        PowerRoute local_route;

        xSemaphoreTake(data_mutex, portMAX_DELAY);
        local_bat1_filt = bat1_voltage_filt;
        local_bat2_filt = bat2_voltage_filt;
        local_current_filt = current_amps_filt;
        local_ltc_ibat = ltc_charge_current;
        local_soc1_prev = soc_bat1;
        local_soc2_prev = soc_bat2;
        local_route = current_route;
        xSemaphoreGive(data_mutex);

        TickType_t now = xTaskGetTickCount();
        bool soc_hold_active = (now < soc_hold_until_tick);

        bool bat1_is_charging = (local_route == ROUTE_CHARGE_A_LOAD_B);
        bool bat2_is_charging = (local_route == ROUTE_CHARGE_B_LOAD_A);
        bool bat1_is_load     = (local_route == ROUTE_CHARGE_B_LOAD_A);
        bool bat2_is_load     = (local_route == ROUTE_CHARGE_A_LOAD_B);

        float bat1_current_for_comp = 0.0f;
        float bat2_current_for_comp = 0.0f;

        if (bat1_is_charging) bat1_current_for_comp = fmaxf(local_ltc_ibat, 0.0f);
        if (bat2_is_charging) bat2_current_for_comp = fmaxf(local_ltc_ibat, 0.0f);

        if (bat1_is_load) bat1_current_for_comp = fabsf(local_current_filt);
        if (bat2_is_load) bat2_current_for_comp = fabsf(local_current_filt);

        float bat1_rest_est = estimate_rest_voltage(local_bat1_filt, bat1_current_for_comp, bat1_is_charging, bat1_is_load);
        float bat2_rest_est = estimate_rest_voltage(local_bat2_filt, bat2_current_for_comp, bat2_is_charging, bat2_is_load);

        bool bat1_near_rest = (!bat1_is_charging && !bat1_is_load) || (bat1_current_for_comp < SOC_REST_CURRENT_THRESHOLD_A);
        bool bat2_near_rest = (!bat2_is_charging && !bat2_is_load) || (bat2_current_for_comp < SOC_REST_CURRENT_THRESHOLD_A);

        float new_soc1 = local_soc1_prev;
        float new_soc2 = local_soc2_prev;

        const float alpha_soc = 0.20f;

        if (!soc_hold_active)
        {
            if (bat1_near_rest || bat1_is_charging || bat1_is_load)
            {
                float est1 = compute_soc_lead_acid_resting(bat1_rest_est);
                soc_bat1_raw += alpha_soc * (est1 - soc_bat1_raw);
                new_soc1 = soc_bat1_raw;
            }

            if (bat2_near_rest || bat2_is_charging || bat2_is_load)
            {
                float est2 = compute_soc_lead_acid_resting(bat2_rest_est);
                soc_bat2_raw += alpha_soc * (est2 - soc_bat2_raw);
                new_soc2 = soc_bat2_raw;
            }
        }

        if (new_soc1 > 100.0f) new_soc1 = 100.0f;
        if (new_soc1 < 0.0f)   new_soc1 = 0.0f;
        if (new_soc2 > 100.0f) new_soc2 = 100.0f;
        if (new_soc2 < 0.0f)   new_soc2 = 0.0f;

        xSemaphoreTake(data_mutex, portMAX_DELAY);
        soc_bat1 = new_soc1;
        soc_bat2 = new_soc2;
        xSemaphoreGive(data_mutex);

        ESP_LOGI(TAG,
                 "SOC | BatA: %.1f %% | BatB: %.1f %% | V1f=%.2f | V2f=%.2f | V1rest=%.2f | V2rest=%.2f | Iload=%.2f | Ichg=%.2f | Route=%s | hold=%d",
                 new_soc1, new_soc2,
                 local_bat1_filt, local_bat2_filt,
                 bat1_rest_est, bat2_rest_est,
                 local_current_filt, local_ltc_ibat,
                 route_to_string(local_route),
                 soc_hold_active ? 1 : 0);

        vTaskDelay(pdMS_TO_TICKS(SOC_TASK_PERIOD_MS));
    }
}

void ltc_task(void *pvParameters)
{
    while (1)
    {
        float vb = ltc4015_get_battery_voltage();
        float ibat = ltc4015_get_battery_current();
        ltc4015_read_config_bits();
        ltc4015_read_status();

        if (vb < 0.0f || ibat < -1000.0f)
        {
            ESP_LOGE(TAG, "LTC read error");
            fault_present = true;
        }
        else
        {
            xSemaphoreTake(data_mutex, portMAX_DELAY);
            ltc_battery_voltage = vb;
            ltc_charge_current = ibat;
            xSemaphoreGive(data_mutex);

            ESP_LOGI(TAG,
                "LTC | VBAT: %.2f V | RAW_VBAT: %u | IBAT_RAW: 0x%04X | IBAT: %.3f A | CONFIG_BITS: 0x%04X",
                vb, ltc_raw_vbat, ltc_raw_ibat, ibat, ltc_config_bits);
        }

        ltc4015_debug_full();
        vTaskDelay(pdMS_TO_TICKS(LTC_TASK_PERIOD_MS));
    }
}

void control_task(void *pvParameters)
{
    static int valid_recovery_count = 0;

    while (1)
    {
        float local_solar;
        float local_bat1, local_bat2;
        float local_soc1, local_soc2;
        bool local_solar_available;
        bool local_fault;

        xSemaphoreTake(data_mutex, portMAX_DELAY);
        local_solar = solar_voltage;
        local_bat1 = bat1_voltage;
        local_bat2 = bat2_voltage;
        local_soc1 = soc_bat1;
        local_soc2 = soc_bat2;
        local_solar_available = solar_available;
        local_fault = fault_present;
        xSemaphoreGive(data_mutex);

        bool measurements_ok = voltages_are_valid(local_solar, local_bat1, local_bat2);

        if (!measurements_ok)
        {
            ESP_LOGE(TAG,
                     "Invalid measured voltages | Solar=%.2f V | BatA=%.2f V | BatB=%.2f V",
                     local_solar, local_bat1, local_bat2);
            fault_present = true;
            valid_recovery_count = 0;
        }

        if (local_fault || fault_present)
        {
            if (measurements_ok)
            {
                valid_recovery_count++;

                if (valid_recovery_count >= 5)
                {
                    ESP_LOGW(TAG, "Fault cleared after %d valid cycles -> STARTUP",
                             valid_recovery_count);

                    fault_present = false;
                    valid_recovery_count = 0;
                    system_state = STATE_STARTUP;
                }
                else
                {
                    system_state = STATE_FAULT;
                }
            }
            else
            {
                valid_recovery_count = 0;
                system_state = STATE_FAULT;
            }
        }

        switch (system_state)
        {
            case STATE_STARTUP:
            {
                PowerRoute startup_route;

                if (local_solar_available)
                {
                    startup_route = choose_startup_route(local_soc1, local_soc2);
                    set_route_safely(startup_route);
                    system_state = STATE_NORMAL;
                }
                else
                {
                    startup_route = choose_load_only_route(local_soc1, local_soc2);
                    set_route_safely(startup_route);
                    ltc4015_enable_charging(false);
                    system_state = STATE_LOW_SOLAR;
                }

                ESP_LOGI(TAG, "Startup -> %s | %s",
                         state_to_string(system_state),
                         route_to_string(current_route));
                break;
            }

            case STATE_NORMAL:
            {
                if (!local_solar_available)
                {
                    ltc4015_enable_charging(false);

                    PowerRoute desired_load_route = choose_load_only_route(local_soc1, local_soc2);
                    if (desired_load_route != current_route || current_route == ROUTE_ALL_OFF)
                    {
                        set_route_safely(desired_load_route);
                    }

                    system_state = STATE_LOW_SOLAR;
                    ESP_LOGW(TAG, "Solar lost -> LOW_SOLAR");
                    break;
                }

                if (!charging_enabled && current_route != ROUTE_ALL_OFF)
                {
                    ltc4015_enable_charging(true);
                }

                if (should_enter_recovery_lock(local_soc1, local_soc2))
                {
                    locked_route = choose_route_from_soc(local_soc1, local_soc2);
                    recovery_lock_active = true;
                    set_route_safely(locked_route);
                    system_state = STATE_RECOVERY_LOCK;

                    ESP_LOGW(TAG, "Both batteries low -> RECOVERY_LOCK on %s",
                             route_to_string(locked_route));
                    break;
                }

                PowerRoute desired_route = choose_route_from_soc(local_soc1, local_soc2);

                if (desired_route != current_route)
                {
                    set_route_safely(desired_route);
                }

                ESP_LOGI(TAG,
                         "NORMAL | Route: %s | LoadSOC: %.1f %% | ChargeSOC: %.1f %%",
                         route_to_string(current_route),
                         get_load_battery_soc_for_route(current_route),
                         get_charge_battery_soc_for_route(current_route));
                break;
            }

            case STATE_LOW_SOLAR:
            {
                if (charging_enabled)
                    ltc4015_enable_charging(false);

                PowerRoute desired_load_route = choose_load_only_route(local_soc1, local_soc2);

                if (desired_load_route != current_route)
                {
                    set_route_safely(desired_load_route);
                }
                else if (current_route == ROUTE_ALL_OFF)
                {
                    set_route_safely(desired_load_route);
                }

                ESP_LOGW(TAG,
                         "LOW_SOLAR | Route: %s | LoadSOC: %.1f %% | OtherSOC: %.1f %%",
                         route_to_string(current_route),
                         get_load_battery_soc_for_route(current_route),
                         get_charge_battery_soc_for_route(current_route));

                if (local_solar_available)
                {
                    if (recovery_lock_active)
                        system_state = STATE_RECOVERY_LOCK;
                    else
                        system_state = STATE_NORMAL;

                    if (current_route != ROUTE_ALL_OFF)
                        ltc4015_enable_charging(true);

                    ESP_LOGI(TAG, "Solar restored -> %s", state_to_string(system_state));
                }
                break;
            }

            case STATE_RECOVERY_LOCK:
            {
                if (!recovery_lock_active)
                {
                    system_state = STATE_NORMAL;
                    break;
                }

                if (!local_solar_available)
                {
                    ltc4015_enable_charging(false);

                    PowerRoute desired_load_route = choose_load_only_route(local_soc1, local_soc2);
                    if (desired_load_route != current_route || current_route == ROUTE_ALL_OFF)
                    {
                        set_route_safely(desired_load_route);
                    }

                    system_state = STATE_LOW_SOLAR;
                    break;
                }

                if (current_route != locked_route)
                {
                    set_route_safely(locked_route);
                }

                if (!charging_enabled && current_route != ROUTE_ALL_OFF)
                {
                    ltc4015_enable_charging(true);
                }

                if (recovery_lock_cleared(locked_route, local_soc1, local_soc2))
                {
                    recovery_lock_active = false;
                    locked_route = ROUTE_ALL_OFF;
                    system_state = STATE_NORMAL;

                    ESP_LOGI(TAG, "Recovery lock cleared -> NORMAL");
                }
                else
                {
                    ESP_LOGW(TAG,
                             "RECOVERY_LOCK | Route: %s | BatA: %.1f %% | BatB: %.1f %%",
                             route_to_string(current_route), local_soc1, local_soc2);
                }

                break;
            }

            case STATE_SWITCHING:
            {
                if (recovery_lock_active)
                    system_state = STATE_RECOVERY_LOCK;
                else if (local_solar_available)
                    system_state = STATE_NORMAL;
                else
                    system_state = STATE_LOW_SOLAR;
                break;
            }

            case STATE_FAULT:
            default:
            {
                if (charging_enabled)
                    ltc4015_enable_charging(false);

                apply_route_gpio(ROUTE_ALL_OFF);
                ESP_LOGE(TAG,
                         "FAULT | Charging disabled, all routes off | Solar=%.2f V | BatA=%.2f V | BatB=%.2f V | recovery_count=%d",
                         local_solar, local_bat1, local_bat2, valid_recovery_count);
                break;
            }
        }

        update_fan_output();
        vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
    }
}

void uart_status_task(void *pvParameters)
{
    char line[768];

    while (1)
    {
        float local_solar, local_bat1, local_bat2, local_soc1, local_soc2;
        float local_solar_f, local_bat1_f, local_bat2_f, local_current_f;
        float local_current, local_ltc_vbat, local_ltc_ibat;
        bool local_fault, local_solar_ok, local_charging, local_fan;
        PowerRoute local_route;
        SystemState local_state;
        uint16_t local_raw_vbat, local_raw_ibat, local_sys, local_cfgbits;
        uint16_t local_chg_state, local_chg_status, local_ichg_dac;

        xSemaphoreTake(data_mutex, portMAX_DELAY);
        local_solar = solar_voltage;
        local_bat1 = bat1_voltage;
        local_bat2 = bat2_voltage;
        local_solar_f = solar_voltage_filt;
        local_bat1_f = bat1_voltage_filt;
        local_bat2_f = bat2_voltage_filt;
        local_current = current_amps;
        local_current_f = current_amps_filt;
        local_soc1 = soc_bat1;
        local_soc2 = soc_bat2;
        local_ltc_vbat = ltc_battery_voltage;
        local_ltc_ibat = ltc_charge_current;
        local_fault = fault_present;
        local_solar_ok = solar_available;
        local_charging = charging_enabled;
        local_fan = fan_relay_on;
        local_route = current_route;
        local_state = system_state;
        local_raw_vbat = ltc_raw_vbat;
        local_raw_ibat = ltc_raw_ibat;
        local_sys = ltc_system_status;
        local_cfgbits = ltc_config_bits;
        local_chg_state = ltc_charger_state;
        local_chg_status = ltc_charge_status;
        local_ichg_dac = ltc_icharge_dac;
        xSemaphoreGive(data_mutex);

        int len = snprintf(line, sizeof(line),
            "solar=%.2f,bat1=%.2f,bat2=%.2f,solarf=%.2f,bat1f=%.2f,bat2f=%.2f,soc1=%.1f,soc2=%.1f,i=%.2f,if=%.2f,ltcv=%.2f,ibat=%.3f,rawvbat=%u,rawibat=0x%04X,cfg=0x%04X,sys=0x%04X,chgstate=0x%04X,chgstatus=0x%04X,ichgdac=0x%04X,route=%d,state=%d,solarok=%d,chg=%d,fan=%d,fault=%d\r\n",
            local_solar, local_bat1, local_bat2,
            local_solar_f, local_bat1_f, local_bat2_f,
            local_soc1, local_soc2,
            local_current, local_current_f,
            local_ltc_vbat, local_ltc_ibat,
            local_raw_vbat, local_raw_ibat,
            local_cfgbits, local_sys,
            local_chg_state, local_chg_status, local_ichg_dac,
            (int)local_route, (int)local_state,
            (int)local_solar_ok, (int)local_charging, (int)local_fan, (int)local_fault);

        if (len > 0)
        {
            uart_write_bytes(UART_PORT_NUM, line, len);
        }

        vTaskDelay(pdMS_TO_TICKS(UART_TASK_PERIOD_MS));
    }
}

// ---------------- Main ----------------
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "System starting...");

    ESP_ERROR_CHECK(nvs_flash_init());

    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    last_route_switch_tick = xTaskGetTickCount();
    soc_hold_until_tick = 0;

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(PIN_SOLAR,   ADC_ATTEN_DB_11);
    adc1_config_channel_atten(PIN_BAT1,    ADC_ATTEN_DB_11);
    adc1_config_channel_atten(PIN_BAT2,    ADC_ATTEN_DB_11);
    adc1_config_channel_atten(PIN_CURRENT, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(PIN_VREF,    ADC_ATTEN_DB_11);

    gpio_reset_pin(GPIO_CHARGE_A_LOAD_B);
    gpio_reset_pin(GPIO_CHARGE_B_LOAD_A);
    gpio_reset_pin(GPIO_FAN_RELAY);

    gpio_set_direction(GPIO_CHARGE_A_LOAD_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_CHARGE_B_LOAD_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_FAN_RELAY, GPIO_MODE_OUTPUT);

    gpio_set_level(GPIO_CHARGE_A_LOAD_B, 0);
    gpio_set_level(GPIO_CHARGE_B_LOAD_A, 0);
    gpio_set_level(GPIO_FAN_RELAY, 0);

    i2c_master_init();
    uart_init_custom();

    xTaskCreate(adc_task,         "adc_task",         4096, NULL, 6, NULL);
    xTaskCreate(soc_task,         "soc_task",         4096, NULL, 3, NULL);
    xTaskCreate(ltc_task,         "ltc_task",         4096, NULL, 5, NULL);
    xTaskCreate(control_task,     "control_task",     4096, NULL, 4, NULL);
    xTaskCreate(uart_status_task, "uart_status_task", 4096, NULL, 2, NULL);
}