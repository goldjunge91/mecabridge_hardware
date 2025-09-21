#ifndef DRIVE_ARDUINO__FIRMWARE__MECABRIDGE_PICO__MOTORS_H_
#define DRIVE_ARDUINO__FIRMWARE__MECABRIDGE_PICO__MOTORS_H_


#pragma once
#include <stdint.h>
void motors_init(void);               // init pins + PWM @ default kHz
void motors_set_freq_hz(uint32_t hz); // 20000..50000
void motor_drive_pct(int i, int pct); // i: 0=FL,1=FR,2=RL,3=RR ; pct -100..100
void motors_brake_all(void);
#endif  // DRIVE_ARDUINO__FIRMWARE__MECABRIDGE_PICO__MOTORS_H_
