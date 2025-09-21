#include "motors.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "config.h"

static uint slice_m[4], ch_m[4];
static const uint PWM_PINS[4] = {FL_PWM, FR_PWM, RL_PWM, RR_PWM};
static const uint8_t IN1[4] = {FL_IN1, FR_IN1, RL_IN1, RR_IN1};
static const uint8_t IN2[4] = {FL_IN2, FR_IN2, RL_IN2, RR_IN2};
static int8_t offset[4] = {OFFSET_FL, OFFSET_FR, OFFSET_RL, OFFSET_RR};

static inline int clamp(int v, int lo, int hi) {return v < lo ? lo : (v > hi ? hi : v);}

void motors_set_freq_hz(uint32_t hz)
{
  for (int i = 0; i < 4; i++) {
    uint s = slice_m[i];
    pwm_set_clkdiv(s, 1.0f);
    uint32_t top = (125000000u / hz) - 1u;
    pwm_set_wrap(s, top);
  }
}

void motors_init(void)
{
  gpio_init(STBY);
  gpio_set_dir(STBY, GPIO_OUT);
  gpio_put(STBY, 1);
  for (int i = 0; i < 4; i++) {
    gpio_init(IN1[i]);
    gpio_set_dir(IN1[i], GPIO_OUT);
    gpio_put(IN1[i], 0);
    gpio_init(IN2[i]);
    gpio_set_dir(IN2[i], GPIO_OUT);
    gpio_put(IN2[i], 0);
    gpio_set_function(PWM_PINS[i], GPIO_FUNC_PWM);
    slice_m[i] = pwm_gpio_to_slice_num(PWM_PINS[i]);
    ch_m[i] = pwm_gpio_to_channel(PWM_PINS[i]);
    pwm_set_enabled(slice_m[i], true);
  }
  motors_set_freq_hz(20000); // 20 kHz default
}

void motor_drive_pct(int i, int pct)
{
  pct = clamp(pct, -100, 100) * offset[i];
  uint s = slice_m[i], ch = ch_m[i];
  uint16_t top = pwm_hw->slice[s].top;
  // sichere Richtungsumschaltung
  if (pct == 0) {
    pwm_set_chan_level(s, ch, 0);
    gpio_put(IN1[i], 0);
    gpio_put(IN2[i], 0);
    return;
  }
  static int dir_last[4] = {0, 0, 0, 0};
  int dir = (pct > 0) ? +1 : -1;
  if (dir_last[i] && dir_last[i] != dir) {
    pwm_set_chan_level(s, ch, 0);
    sleep_us(200);
  }
  gpio_put(IN1[i], dir > 0);
  gpio_put(IN2[i], dir < 0);
  uint16_t lvl = (uint16_t)((uint32_t)(pct > 0 ? pct : -pct) * top / 100);
  pwm_set_chan_level(s, ch, lvl);
  dir_last[i] = dir;
}

void motors_brake_all(void)
{
  for (int i = 0; i < 4; i++) {
    gpio_put(IN1[i], 1);
    gpio_put(IN2[i], 1);
    pwm_set_chan_level(slice_m[i], ch_m[i], 0);
  }
}
