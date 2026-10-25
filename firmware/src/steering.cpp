#include "steering.h"

#include <Arduino.h>
#include "config.h"
#include "timebase.h"

#if STEERING_ENABLED
static_assert(HALL_LINES == 1, "Skręt używa kanału CC/PPI halla B – tylko HALL_LINES == 1");

static volatile uint32_t s_rise_t;
static volatile bool     s_have_rise;
static volatile uint16_t s_width;
static volatile uint32_t s_valid_t;
static volatile bool     s_any_valid;
static volatile uint32_t s_pulses, s_rejects;

static uint16_t s_neutral;
static bool     s_learned;
static uint32_t s_win_start_t, s_win_sum, s_win_n, s_win_last_pulse;
static uint16_t s_win_min, s_win_max;

static void isr_steering(void) {
  uint32_t t = timebase_captured(TB_CC_STEER);
  if (nrf_gpio_pin_read(g_ADigitalPinMap[PIN_STEERING])) {
    s_rise_t = t;
    s_have_rise = true;
    return;
  }
  if (!s_have_rise) return;
  s_have_rise = false;
  uint32_t w = t - s_rise_t;
  if (w < THROTTLE_MIN_US || w > THROTTLE_MAX_US) {
    s_rejects++;
    return;
  }
  s_width = (uint16_t)w;
  s_valid_t = t;
  s_any_valid = true;
  s_pulses++;
}

bool steering_begin(void) {
  pinMode(PIN_STEERING, INPUT);
  attachInterrupt(PIN_STEERING, isr_steering, CHANGE);
  return timebase_connect_capture(PIN_STEERING, TB_CC_STEER, 1);
}

uint16_t steering_tick_width(uint32_t t_tick) {
  if (!s_any_valid) return 0;
  if ((int32_t)(t_tick - s_valid_t) > (int32_t)THROTTLE_TIMEOUT_MS * 1000) return 0;
  return s_width;
}

// Neutrum jak w throttle.cpp: THROTTLE_NEUTRAL_LEARN_MS stabilnego sygnału (rozrzut < SPREAD)
void steering_poll(uint32_t now_us) {
  if (s_learned) return;
  noInterrupts();
  uint32_t pulses = s_pulses;
  uint16_t w = s_width;
  interrupts();
  if (!steering_tick_width(now_us)) {
    s_win_n = 0;
    return;
  }
  if (pulses == s_win_last_pulse) return;
  s_win_last_pulse = pulses;
  if (s_win_n == 0) {
    s_win_start_t = now_us;
    s_win_sum = 0;
    s_win_min = 0xFFFF;
    s_win_max = 0;
  }
  s_win_sum += w;
  s_win_n++;
  if (w < s_win_min) s_win_min = w;
  if (w > s_win_max) s_win_max = w;
  if (s_win_max - s_win_min >= THROTTLE_NEUTRAL_SPREAD_US) {
    s_win_n = 0;
    return;
  }
  if ((uint32_t)(now_us - s_win_start_t) >= (uint32_t)THROTTLE_NEUTRAL_LEARN_MS * 1000) {
    s_neutral = (uint16_t)((s_win_sum + s_win_n / 2) / s_win_n);
    s_learned = true;
  }
}

bool steering_neutral_learned(void) { return s_learned; }
uint16_t steering_neutral_us(void) { return s_neutral; }
uint32_t steering_pulse_count(void) { return s_pulses; }
uint32_t steering_reject_count(void) { return s_rejects; }

#else
bool steering_begin(void) { return true; }
uint16_t steering_tick_width(uint32_t) { return 0; }
void steering_poll(uint32_t) {}
bool steering_neutral_learned(void) { return false; }
uint16_t steering_neutral_us(void) { return 0; }
uint32_t steering_pulse_count(void) { return 0; }
uint32_t steering_reject_count(void) { return 0; }
#endif
