#include "enc.h"
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "config.h"

// enum
// {
//     ENC_FL_A = 8,
//     ENC_FL_B = 9,
//     ENC_FR_A = 10,
//     ENC_FR_B = 11,
//     ENC_RL_A = 12,
//     ENC_RL_B = 13,
//     ENC_RR_A = 6,
//     ENC_RR_B = 7
// };

typedef struct
{
  uint8_t a, b;
  volatile int32_t cnt;
  uint8_t last;
} enc_t;
static enc_t E[4] =
{{ENC_FL_A, ENC_FL_B, 0, 0}, {ENC_FR_A, ENC_FR_B, 0, 0}, {ENC_RL_A, ENC_RL_B, 0, 0},
  {ENC_RR_A, ENC_RR_B, 0, 0}};
static const int8_t LUT[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

static void handle(uint idx)
{
  enc_t * e = &E[idx];
  uint a = gpio_get(e->a), b = gpio_get(e->b);
  uint cur = (a << 1) | b, idxlut = ((e->last & 3) << 2) | (cur & 3);
  e->cnt += LUT[idxlut];
  e->last = cur;
}

static void irq(uint gpio, uint32_t events)
{
  (void)events;
  for (uint i = 0; i < 4; i++) {
    if (gpio == E[i].a || gpio == E[i].b) {
      handle(i);
      break;
    }
  }
}

void enc_init(void)
{
  for (int i = 0; i < 4; i++) {
    gpio_init(E[i].a);
    gpio_pull_up(E[i].a);
    gpio_set_dir(E[i].a, false);
    gpio_init(E[i].b);
    gpio_pull_up(E[i].b);
    gpio_set_dir(E[i].b, false);
    E[i].last = (gpio_get(E[i].a) << 1) | gpio_get(E[i].b);
  }
  gpio_set_irq_enabled_with_callback(ENC_FL_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &irq);
  gpio_set_irq_enabled(ENC_FL_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC_FR_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC_FR_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC_RL_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC_RL_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC_RR_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC_RR_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

void enc_read(int32_t out[4])
{
  uint32_t irq = save_and_disable_interrupts();
  for (int i = 0; i < 4; i++) {
    out[i] = E[i].cnt;
  }
  restore_interrupts(irq);
}

void enc_reset(void)
{
  uint32_t irq = save_and_disable_interrupts();
  for (int i = 0; i < 4; i++) {
    E[i].cnt = 0;
  }
  restore_interrupts(irq);
}
