// Wspólny licznik 1 MHz na TIMER4 (32 bit, przepełnienie co ~71,6 min –
// zawsze odejmować jako uint32_t).
//
// Rejestry CC:
//   CC[0..2] – capture zboczy halla A/B/C (GPIOTE → PPI → CAPTURE);
//              w trybie HALL_LINES == 1 CC[1] (PPI 1) = zbocza skrętu (pin D7)
//   CC[3]    – capture zbocza gazu
//   CC[4]    – compare: takt próbkowania SAMPLE_RATE_HZ
//   CC[5]    – capture „teraz” (timebase_now)
#pragma once

#include <stdint.h>

#define TB_CC_HALL_A    0
#define TB_CC_HALL_B    1
#define TB_CC_HALL_C    2
#define TB_CC_STEER     1   // tylko HALL_LINES == 1 (wspólny z halla B)
#define TB_CC_THROTTLE  3
#define TB_CC_TICK      4
#define TB_CC_NOW       5

typedef void (*tick_cb_t)(uint32_t t_us);

void     timebase_begin(tick_cb_t tick_cb);
// Wymuszenie HFXO (kwarc 32 MHz) – bez tego TIMER chodzi z HFINT (±1–2 %).
// Wywołać po włączeniu SoftDevice.
bool     timebase_request_hfxo(void);
uint32_t timebase_now(void);
// Łączy kanał GPIOTE przydzielony pinowi przez attachInterrupt() z
// TASKS_CAPTURE[cc] przez kanał PPI ppi_ch. Zwraca false, gdy brak kanału.
bool     timebase_connect_capture(uint32_t arduino_pin, uint8_t cc, uint8_t ppi_ch);
static inline uint32_t timebase_captured(uint8_t cc);

#include <nrf.h>
static inline uint32_t timebase_captured(uint8_t cc) { return NRF_TIMER4->CC[cc]; }
