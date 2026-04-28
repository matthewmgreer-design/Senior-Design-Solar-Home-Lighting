#include "sensors_fin.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "SENSORS";

// =========================
// ======== I2C SETUP ======
// =========================
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     GPIO_NUM_39 // change to gpio 39
#define I2C_SCL_PIN     GPIO_NUM_40 // change to gpio 40
#define I2C_FREQ_HZ     100000

#define APDS_ADDR       0x39    // 7-bit APDS9960 address

// APDS9960 register map
#define APDS_ENABLE     0x80
#define APDS_ATIME      0x81
#define APDS_CONTROL    0x8F
#define APDS_ID         0x92
#define APDS_CDATAL     0x94    // ALS low byte
#define APDS_CDATAH     0x95    // ALS high byte

// ============================
// ===== Ambient Threshold ====
// ============================
#define DARK_THRESHOLD  120     // Increase if too sensitive
static bool s_dark = false;


// ======================================================
// ======== I2C Helper functions ========================
// ======================================================
static esp_err_t apds_write(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = {reg, val};
    return i2c_master_write_to_device(
        I2C_PORT, APDS_ADDR, data, 2, pdMS_TO_TICKS(20));
}

static esp_err_t apds_read_word(uint8_t reg, uint16_t* out)
{
    uint8_t buf[2];

    esp_err_t err = i2c_master_write_read_device(
        I2C_PORT, APDS_ADDR,
        &reg, 1,
        buf, 2,
        pdMS_TO_TICKS(20)
    );

    if (err == ESP_OK) {
        *out = ((uint16_t)buf[1] << 8) | buf[0];
    }

    return err;
}


// ==========================================
// ========== APDS9960 Ambient Init =========
// ==========================================
static void apds_init(void)
{
    ESP_LOGI(TAG, "Initializing APDS9960 ALS...");

    i2c_config_t cfg = {};
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = I2C_SDA_PIN;
    cfg.scl_io_num = I2C_SCL_PIN;
    cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.clk_flags = 0;
    cfg.master.clk_speed = I2C_FREQ_HZ;

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    // Read chip ID
    uint8_t id_reg = 0;
    uint8_t id_addr = APDS_ID;

    i2c_master_write_read_device(
        I2C_PORT, APDS_ADDR,
        &id_addr, 1,
        &id_reg, 1,
        pdMS_TO_TICKS(20)
    );

    ESP_LOGI(TAG, "APDS9960 ID = 0x%02X (should be 0xAB)", id_reg);

    // Enable ALS
    apds_write(APDS_ENABLE, 0x03);   // Bit0 = PON, Bit1 = AEN (ALS enable)

    // Integration time & gain tuning
    apds_write(APDS_ATIME,  0xDB);   // ~100 ms integration
    apds_write(APDS_CONTROL, 0x06);  // AGAIN = 64x
}


// ==========================================
// ============= Read Ambient Light =========
// ==========================================
int sensors_get_ambient(void)
{
    uint16_t als = 0;
    esp_err_t err = apds_read_word(APDS_CDATAL, &als);

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "ALS read failed");
        return -1;
    }

    return als;
}


// ==========================================
// =========== Dark / Bright Logic ==========
// ==========================================
bool sensors_is_dark(void)
{
    int als = sensors_get_ambient();
    if (als < 0) return false;   // read fail → assume bright

    s_dark = (als <= DARK_THRESHOLD);
    return s_dark;
}


// ==========================================
// ============ PIR MOTION SENSORS ===========
// ==========================================
void init_pir(void)
{
    gpio_config_t io = {};
    io.intr_type = GPIO_INTR_DISABLE;
    io.mode = GPIO_MODE_INPUT;

    // Configure BOTH PIR pins from sensors.h
    io.pin_bit_mask = (1ULL << PIR_OUTSIDE_PIN) | (1ULL << PIR_INSIDE_PIN);

    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en = GPIO_PULLUP_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io));

    ESP_LOGI(TAG, "PIR initialized: outside GPIO %d, inside GPIO %d",
             (int)PIR_OUTSIDE_PIN, (int)PIR_INSIDE_PIN);
}

// Active-high PIRs: level=1 means motion
bool read_pir_outside(void)
{
    return gpio_get_level(PIR_OUTSIDE_PIN) == 1;
}

bool read_pir_inside(void)
{
    return gpio_get_level(PIR_INSIDE_PIN) == 1;
}


// ==========================================
// ============= COMBINED INIT ==============
// ==========================================
void sensors_init(void)
{
    apds_init();
    init_pir();
    ESP_LOGI(TAG, "All sensors initialized");
}