// term.cpp
#include "term.h"
#include "pico/stdlib.h"
#include "motors.h"
#include "esc_servo.h"
#include "enc.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MAX_CMD_LEN 128
#define MAX_ARGS 5

// --- Static Function Declarations ---
static void process_command(char * line);
static void handle_help();
static void handle_stop();
static void handle_freq(int argc, char * argv[]);
static void handle_esc(int argc, char * argv[]);
static void handle_gear(int argc, char * argv[]);
static void handle_enc_read();
static void handle_enc_reset();
static void handle_drive(const char * cmd, int argc, char * argv[]);
static void handle_motor(int argc, char * argv[]);

// --- Public Functions ---

void term_init(void)
{
  // Wait for user to connect and send a character
  while (getchar_timeout_us(10000) == PICO_ERROR_TIMEOUT) {}

  puts("\n=== Pico Robot Terminal ===");
  puts("Type 'help' for a list of commands.");
  printf("> ");
  fflush(stdout);
}

void term_poll(void)
{
  static char line_buf[MAX_CMD_LEN];
  static uint len = 0;
  int c;

  if ((c = getchar_timeout_us(0)) == PICO_ERROR_TIMEOUT) {
    return;     // No character received
  }

  // Echo character back to the terminal
  putchar(c);
  fflush(stdout);

  if (c == '\r' || c == '\n') {
    printf("\n");     // New line after command entry
    if (len > 0) {
      line_buf[len] = '\0';       // Null-terminate the string
      process_command(line_buf);
      len = 0;       // Reset buffer
    }
    printf("> ");     // Show prompt for next command
    fflush(stdout);
  } else if (c >= 32 && len < MAX_CMD_LEN - 1) {
    // Store printable characters
    line_buf[len++] = (char)c;
  } else if ((c == 8 || c == 127) && len > 0) {
    // Handle backspace
    len--;
    printf("\b \b");     // Erase character on screen
    fflush(stdout);
  }
}

// --- Command Processing Logic ---

static void process_command(char * line)
{
  char * argv[MAX_ARGS + 1] = {0};
  int argc = 0;
  char * p = strtok(line, " ");
  while (p != NULL && argc < MAX_ARGS) {
    argv[argc++] = p;
    p = strtok(NULL, " ");
  }

  if (argc == 0) {return;}

  const char * cmd = argv[0];

  if (!strcmp(cmd, "help")) {handle_help();} else if (!strcmp(cmd, "x")) {
    handle_stop();
  } else if (!strcmp(cmd, "freq")) {
    handle_freq(argc, argv);
  } else if (!strcmp(cmd, "esc1") || !strcmp(cmd, "esc2")) {
    handle_esc(argc, argv);
  } else if (!strcmp(cmd, "gear")) {handle_gear(argc, argv);} else if (!strcmp(cmd, "encr")) {
    handle_enc_read();
  } else if (!strcmp(cmd, "encz")) {
    handle_enc_reset();
  } else if (!strcmp(
      cmd,
      "w") ||
    !strcmp(
      cmd,
      "s") || !strcmp(cmd, "a") || !strcmp(cmd, "d") || !strcmp(cmd, "q") || !strcmp(cmd, "e"))
  {
    handle_drive(cmd, argc, argv);
  } else if (!strcmp(cmd, "m")) {handle_motor(argc, argv);} else {
    printf("[ERR] Unknown command: %s\n", cmd);
  }
}

// --- Command Handlers ---

static inline int clamp(int v, int lo, int hi) {return v < lo ? lo : (v > hi ? hi : v);}

static void handle_help()
{
  puts("Commands:");
  puts("  w/s/a/d/q/e [pct] - Drive robot");
  puts("  m <id> <pct> [ms] - Control a single motor");
  puts("  x                 - Stop all motors");
  puts("  esc1/esc2 <pct>   - Control ESCs");
  puts("  gear <pct>        - Control servo gear");
  puts("  encr              - Read encoder values");
  puts("  encz              - Reset encoder values");
  puts("  freq <hz>         - Set PWM frequency (20k-50k)");
  puts("  help              - Show this message");
}

static void handle_stop()
{
  motors_brake_all();
  puts("[OK] All motors stopped.");
}

static void handle_freq(int argc, char * argv[])
{
  if (argc < 2) {
    puts("[ERR] Usage: freq <hz>");
    return;
  }
  int f = atoi(argv[1]);
  motors_set_freq_hz(clamp(f, 20000, 50000));
  printf("[OK] PWM frequency set to %d Hz\n", f);
}

static void handle_esc(int argc, char * argv[])
{
  if (argc < 2) {
    printf("[ERR] Usage: %s <percentage>\n", argv[0]);
    return;
  }
  int id = (argv[0][3] == '1') ? 1 : 2;
  int pct = clamp(atoi(argv[1]), 0, 100);
  esc_write_pct(id, pct);
  printf("[OK] ESC %d set to %d%%\n", id, pct);
}

static void handle_gear(int argc, char * argv[])
{
  if (argc < 2) {
    puts("[ERR] Usage: gear <percentage>");
    return;
  }
  int pct = clamp(atoi(argv[1]), -100, 100);
  gear_speed_pct(pct);
  printf("[OK] Gear set to %d%%\n", pct);
}

static void handle_enc_read()
{
  int32_t v[4];
  enc_read(v);
  printf("[ENC] %ld, %ld, %ld, %ld\n", v[0], v[1], v[2], v[3]);
}

static void handle_enc_reset()
{
  enc_reset();
  puts("[OK] Encoders reset.");
}

static void handle_drive(const char * cmd, int argc, char * argv[])
{
  static int def_spd = 50;
  int s = (argc >= 2) ? clamp(atoi(argv[1]), 0, 100) : def_spd;
  def_spd = s;

  if (!strcmp(cmd, "w")) {
    motor_drive_pct(0, +s); motor_drive_pct(1, +s); motor_drive_pct(2, +s); motor_drive_pct(3, +s);
  } else if (!strcmp(cmd, "s")) {
    motor_drive_pct(0, -s); motor_drive_pct(1, -s); motor_drive_pct(2, -s); motor_drive_pct(3, -s);
  } else if (!strcmp(cmd, "a")) {
    motor_drive_pct(0, -s); motor_drive_pct(1, +s); motor_drive_pct(2, -s); motor_drive_pct(3, +s);
  } else if (!strcmp(cmd, "d")) {
    motor_drive_pct(0, +s); motor_drive_pct(1, -s); motor_drive_pct(2, +s); motor_drive_pct(3, -s);
  } else if (!strcmp(cmd, "q")) {
    motor_drive_pct(0, -s); motor_drive_pct(1, +s); motor_drive_pct(2, +s); motor_drive_pct(3, -s);
  } else if (!strcmp(cmd, "e")) {
    motor_drive_pct(0, +s); motor_drive_pct(1, -s); motor_drive_pct(2, -s); motor_drive_pct(3, +s);
  }

  printf("[OK] Drive command '%s' with speed %d%%\n", cmd, s);
}

static void handle_motor(int argc, char * argv[])
{
  if (argc < 3) {
    puts("[ERR] Usage: m <id 0-3> <pct -100-100>");
    return;
  }
  int i = clamp(atoi(argv[1]), 0, 3);
  int p = clamp(atoi(argv[2]), -100, 100);
  int ms = (argc >= 4) ? clamp(atoi(argv[3]), 0, 10000) : 0;

  motor_drive_pct(i, p);
  printf("[OK] Motor %d set to %d%%\n", i, p);

  if (ms > 0) {
    sleep_ms(ms);
    motor_drive_pct(i, 0);
    printf("[OK] Motor %d stopped after %d ms\n", i, ms);
  }
}
