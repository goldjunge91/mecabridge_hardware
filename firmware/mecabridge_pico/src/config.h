#ifndef DRIVE_ARDUINO__FIRMWARE__MECABRIDGE_PICO__CONFIG_H_
#define DRIVE_ARDUINO__FIRMWARE__MECABRIDGE_PICO__CONFIG_H_


// ===== Laufzeit-Defaults =====
#define PWM_MAX            255   // UNO PWM 8-bit
#define WATCHDOG_MS        300   // Stop, wenn solange kein Kommando
#define LOOP_MS            10    // Regeltakt (ms)
#define DEADZONE_DEFAULT   30    // 0..80 (Anlauf-Offset)
#define SLEW_DEFAULT       8     // 1..30 (max. PWM-Änderung pro LOOP_MS)

// ===== Offset Motoren(UNO) =====
// Richtungskorrektur: 1 oder -1 (wenn "vorwärts" falsch herum ist)
// Richtungskorrektur
#define OFFSET_FL (+1)
#define OFFSET_FR (+1)
#define OFFSET_RL (+1)
#define OFFSET_RR (+1)

// ===== Verkabelung TB6612 (UNO) =====
// Linkes Board (2 Motoren)

// Gemeinsamer Standby-Pin für beide TB6612
#define STBY 28

// Front-Left  (TB1 A)
#define FL_IN1 18
#define FL_IN2 19
#define FL_PWM 2
#define ENC_FL_A   8
#define ENC_FL_B   9
// Front-Right (TB1 B)
#define FR_IN1 17
#define FR_IN2 20
#define FR_PWM 3
#define ENC_FR_A   10
#define ENC_FR_B   11

// Rear-Left   (TB2 A)
#define RL_IN1 21
#define RL_IN2 22
#define RL_PWM 4
#define ENC_RL_A   12
#define ENC_RL_B   13
// Rear-Right  (TB2 B)
#define RR_IN1 26
#define RR_IN2 27
#define RR_PWM 5
#define ENC_RR_A   6
#define ENC_RR_B   7

// Feintrimm je Motor (PWM-Offset in Schritten, zum Geradeauslauf)
#define TRIM_FL     0
#define TRIM_FR     0
#define TRIM_RL     0
#define TRIM_RR     0

// // ##
#define ESC1_PIN   14
#define ESC2_PIN   15
#define GEAR_PIN   16


#endif  // DRIVE_ARDUINO__FIRMWARE__MECABRIDGE_PICO__CONFIG_H_
