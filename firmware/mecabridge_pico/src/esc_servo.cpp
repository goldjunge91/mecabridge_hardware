#include "esc_servo.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
enum
{
  ESC1 = 14,
  ESC2 = 15,
  GEAR = 16
};

static uint slice_esc, slice_gear, ch_esc1, ch_esc2, ch_gear;

static void pwm50_config_pin(uint pin, uint * slice, uint * ch)
{
  gpio_set_function(pin, GPIO_FUNC_PWM);
  *slice = pwm_gpio_to_slice_num(pin);
  *ch = pwm_gpio_to_channel(pin);
  pwm_set_clkdiv(*slice, 125.0f);    // 125 MHz / 125 = 1 MHz
  pwm_set_wrap(*slice, 20000 - 1);   // 1 MHz / 20000 = 50 Hz
  pwm_set_enabled(*slice, true);
}

void esc_servo_init(void)
{
  uint s;
  uint c;
  pwm50_config_pin(ESC1, &s, &c);
  slice_esc = s;
  ch_esc1 = c;
  pwm50_config_pin(ESC2, &s, &c);   /* same slice */
  ch_esc2 = c;
  pwm50_config_pin(GEAR, &slice_gear, &ch_gear);
  // neutral
  pwm_set_chan_level(slice_esc, ch_esc1, 1000);
  pwm_set_chan_level(slice_esc, ch_esc2, 1000);
  pwm_set_chan_level(slice_gear, ch_gear, 1500);
}

void esc_write_pct(int id, int pct)
{
  pct = pct < 0 ? 0 : (pct > 100 ? 100 : pct);
  uint lvl = 1000 + (uint)(pct * 10);   // 1000..2000 us
  if (id == 1) {
    pwm_set_chan_level(slice_esc, ch_esc1, lvl);
  } else {
    pwm_set_chan_level(slice_esc, ch_esc2, lvl);
  }
}

void gear_speed_pct(int pct)
{
  if (pct < -100) {
    pct = -100;
  }
  if (pct > 100) {
    pct = 100;
  }
  int lvl = 1500 + (pct * 500) / 100;   // 1000..2000
  pwm_set_chan_level(slice_gear, ch_gear, (uint)lvl);
}
