#include "lighting_fin.h"
#include "driver/gpio.h"

#define PORCH_PIN     GPIO_NUM_12
#define FOYER_PIN     GPIO_NUM_19
#define SECURITY_PIN  GPIO_NUM_9

static const bool RELAY_ACTIVE_LOW = false;

static void config_output(gpio_num_t pin)
{
    gpio_config_t io{};
    io.mode = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = (1ULL << pin);
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);
}

static void set_pin(gpio_num_t pin, bool on)
{
    int level = (!RELAY_ACTIVE_LOW) ? (on ? 1 : 0) : (on ? 0 : 1);
    gpio_set_level(pin, level);
}

void lighting_init(void)
{
    config_output(PORCH_PIN);
    config_output(FOYER_PIN);
    config_output(SECURITY_PIN);

    set_pin(PORCH_PIN, false);
    set_pin(FOYER_PIN, false);
    set_pin(SECURITY_PIN, false);
}

void lighting_set_porch(bool on)    { set_pin(PORCH_PIN, on); }
void lighting_set_foyer(bool on)    { set_pin(FOYER_PIN, on); }
void lighting_set_security(bool on) { set_pin(SECURITY_PIN, on); }