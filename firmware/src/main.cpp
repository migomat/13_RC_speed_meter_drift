// X4F Launch Logger – faza 1 (tylko pomiar). Urządzenie wyłącznie
// podsłuchuje sygnały halla i gazu; nie ma żadnego wyjścia do ESC.
#include <Arduino.h>
#include "tusb.h"
#include <math.h>

#include "ble_link.h"
#include "config.h"
#include "drift.h"
#include "estimator.h"
#include "flash_store.h"
#include "hall.h"
#include "imu.h"
#include "led.h"
#include "logger.h"
#include "race.h"
#include "run_format.h"
#include "sim.h"
#include "throttle.h"
#include "timebase.h"

// ------------------------------------------------------------ takty 500 Hz
// ISR taktu zapisuje stan halla i gazu; pętla główna dokłada IMU.
struct Tick {
  uint32_t t_us, edges, period;
  uint16_t throttle;
  uint8_t  flags;
};
#define TICK_RING 256   // 512 ms zapasu na zablokowaną pętlę
static Tick s_ticks[TICK_RING];
static volatile uint32_t s_tick_w, s_tick_r;
volatile uint32_t g_tick_overflows;
uint32_t g_stack_free_min = 0xFFFFFFFF;

static void on_tick(uint32_t t) {
  HallSnapshot h;
  hall_tick_snapshot(t, &h);
  uint16_t thr = throttle_tick_width(t);
  if (s_tick_w - s_tick_r >= TICK_RING) {
    g_tick_overflows++;
    return;
  }
  Tick& k = s_ticks[s_tick_w % TICK_RING];
  k.t_us = t;
  k.edges = h.edges;
  k.period = h.period_rev_us;
  k.throttle = thr;
  k.flags = h.flags | (thr ? SF_THROTTLE_VALID : 0);
  s_tick_w = s_tick_w + 1;
}

// ------------------------------------------------------------ estymator
static EstParams s_ep;
static EstState  s_es;
static uint32_t  s_prev_t, s_prev_edges;
static bool      s_have_prev;
static bool      s_imu_ok, s_flash_ok, s_ble_ok, s_hall_ok, s_thr_ok;

// Montaż płytki: osie auta (forward, left, up) = osie płytki idx · znak, potem obrót poziomy
// o yaw. Domyślnie z config.h, zmiana w działaniu komendą MOUNT (wynik kalibracji z aplikacji).
Mount g_mount = {{IMU_AXIS_FORWARD, IMU_AXIS_LEFT, IMU_AXIS_UP},
                 {IMU_SIGN_FORWARD, IMU_SIGN_LEFT, IMU_SIGN_UP}, IMU_YAW_OFFSET_DEG};

static inline float car_axis(const float v[3], int k) { return v[g_mount.idx[k]] * g_mount.sign[k]; }

// ------------------------------------------------------------ drift
static DriftParams s_dp;
static DriftState  s_ds;

// Każda próbka wyścigu (20 Hz) – także gdy pamięć wyścigów jest pełna.
static void on_race_sample(const RaceSample& r) {
  float ka = imu_accel_scale_g(), kg = imu_gyro_scale_dps();
  float a[3] = {r.ax * ka, r.ay * ka, r.az * ka};
  float g[3] = {r.gx * kg, r.gy * kg, r.gz * kg};
  // gaz jak w analizie nagrania: neutrum z nagłówka (0 = nienauczone → 0 %)
  uint16_t neutral = throttle_neutral_learned() ? throttle_neutral_us() : 0;
  float pct = 0.0f;
  if (r.throttle_us && neutral) {
    pct = ((int)r.throttle_us - (int)neutral) * 100.0f / THROTTLE_FULL_SCALE_US;
    if (!THROTTLE_FORWARD_IS_HIGHER) pct = -pct;
  }
  int rest = r.edges == 0 && fabsf(pct) < ARM_NEUTRAL_PCT;
  s_dp.yaw_offset_deg = g_mount.yaw_deg;
  drift_step(&s_ds, &s_dp, (float)r.n_ticks / SAMPLE_RATE_HZ, car_axis(a, 0), car_axis(a, 1),
             car_axis(g, 2), rest);
  g_live.d_pts = s_ds.pts;
  g_live.d_mult = s_ds.mult;
  g_live.d_total = s_ds.total;
  g_live.d_last = s_ds.last;
  g_live.d_best = s_ds.best;
  g_live.d_angle = s_ds.corner ? s_ds.angle : 0.0f;
  g_live.d_on = s_ds.in_drift;
  g_live.d_chain = s_ds.chain;
  g_live.d_banks = s_ds.n_banks;
  g_live.d_lost = s_ds.n_lost;
  g_live.d_trans = s_ds.n_trans;
}

static float v_front_from_period(uint32_t period_us) {
  if (!period_us) return 0.0f;
  float motor_rps = 1.0e6f / (float)period_us;
  return motor_rps / FINAL_DRIVE_RATIO * (float)M_PI * FRONT_TIRE_DIAMETER_MM * 1e-3f;
}

static void process_sample(const Sample& s) {
  float a[3] = {s.ax * imu_accel_scale_g(), s.ay * imu_accel_scale_g(), s.az * imu_accel_scale_g()};
  float g[3] = {s.gx * imu_gyro_scale_dps(), s.gy * imu_gyro_scale_dps(), s.gz * imu_gyro_scale_dps()};
  float pct = throttle_pct(s.throttle_us);
  float vf = v_front_from_period(s.period_rev_us);
  if (s_have_prev) {
    EstInput in;
    in.dt_s = (float)(uint32_t)(s.t_us - s_prev_t) * 1e-6f;
    in.a_fwd_g = car_axis(a, 0);
    in.a_left_g = car_axis(a, 1);
    in.a_up_g = car_axis(a, 2);
    in.g_fwd_dps = car_axis(g, 0);
    in.g_left_dps = car_axis(g, 1);
    in.g_up_dps = car_axis(g, 2);
    s_ep.yaw_offset_deg = g_mount.yaw_deg;
    in.v_front_mps = vf;
    in.hall_stopped = (s.period_rev_us == 0 && s.hall_edges == s_prev_edges);
    in.throttle_pct = pct;
    est_step(&s_es, &s_ep, &in);
  }
  s_prev_t = s.t_us;
  s_prev_edges = s.hall_edges;
  s_have_prev = true;

  g_live.v = s_es.v;
  g_live.v_max = s_es.v_max;
  g_live.v_front = vf;
  g_live.spinning = s_es.spinning;
  g_live.rpm = s.period_rev_us ? 60.0e6f / s.period_rev_us : 0.0f;
  g_live.throttle_us = s.throttle_us;
  g_live.throttle_pct = pct;
  g_live.ax_g = car_axis(a, 0);
  g_live.zupt = s_es.zupt;

  // Wyścig niezależnie od loggera startów (ten w SAVING nie przyjmuje próbek).
  race_push(s, pct, s.throttle_us != 0, throttle_neutral_learned());
  logger_push(s, pct, s.throttle_us != 0);
}

static void drain_ticks(uint32_t now) {
  while (s_tick_r != s_tick_w) {
    const Tick& k = s_ticks[s_tick_r % TICK_RING];
    // Czekamy, aż IMU będzie miało próbkę z chwili taktu (FIFO odczytywane
    // z opóźnieniem do 1 okresu ODR).
    if ((int32_t)(now - k.t_us) < (int32_t)SAMPLE_PERIOD_US) break;
    Sample s;
    s.t_us = k.t_us;
    s.hall_edges = k.edges;
    s.period_rev_us = k.period;
    s.throttle_us = k.throttle;
    s.flags = k.flags;
    s.reserved = 0;
    ImuRaw r;
    bool fresh = imu_get_at(k.t_us, &r);
    if (s_imu_ok) {
      s.ax = r.ax; s.ay = r.ay; s.az = r.az;
      s.gx = r.gx; s.gy = r.gy; s.gz = r.gz;
      if (!fresh) s.flags |= SF_IMU_STALE;
      if (imu_take_overrun_flag()) s.flags |= SF_IMU_OVERRUN;
    } else {
      s.ax = s.ay = s.az = s.gx = s.gy = s.gz = 0;
      s.flags |= SF_IMU_STALE;
    }
    s_tick_r = s_tick_r + 1;
    process_sample(s);
  }
}

// Obsługa pomiaru w trakcie czekania na bufor BLE (ble_send_typed): IMU FIFO
// i kolejka taktów nie mogą się przepełnić przy słabym zasięgu.
static void measure_service(void) {
  uint32_t now = timebase_now();
  imu_poll(now);
  drain_ticks(timebase_now());
  race_poll();
}

// Błąd krytyczny (np. przepełnienie stosu) → reset zamiast zawieszenia.
// Przyczynę widać po restarcie w STATUS: reset_reason (RESETREAS), boot_hf = 1,
// gdy poprzedni reset był z HardFault (znacznik w GPREGRET2 – przeżywa reset
// programowy, bootloader go nie używa).
#define DIAG_HARDFAULT_MARK   0xA5
#define DFU_MAGIC_SERIAL_ONLY 0x4E
#define DFU_MAGIC_UF2         0x57
uint32_t g_boot_gpregret, g_boot_hf;

extern "C" void HardFault_Handler(void) {
  NRF_POWER->GPREGRET2 = DIAG_HARDFAULT_MARK;
  NVIC_SystemReset();
}

// Wejście do bootloadera. Przy włączonym SoftDevice rejestr GPREGRET trzeba
// ustawić przez sd_power_gpregret_set – bezpośredni zapis (enterSerialDfu()
// w rdzeniu, używany przez „touch 1200”) nie dociera do bootloadera i płytka
// startowała z powrotem w aplikacji (wgrywanie wymagało podwójnego RESET).
void enter_bootloader(uint8_t magic) {
  uint8_t sd_en = 0;
  sd_softdevice_is_enabled(&sd_en);
  if (sd_en) {
    sd_power_gpregret_clr(0, 0xFF);
    sd_power_gpregret_set(0, magic);
  } else {
    NRF_POWER->GPREGRET = magic;
  }
  NVIC_SystemReset();
}

// „Touch 1200” (arduino-cli upload): reagujemy już na zmianę prędkości portu,
// zanim biblioteka TinyUSB przy zrzuceniu DTR wywoła enterSerialDfu().
extern "C" void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* coding) {
  if (itf == 0 && coding->bit_rate == 1200) enter_bootloader(DFU_MAGIC_SERIAL_ONLY);
}

void setup() {
  // przed włączeniem SoftDevice – rejestry POWER dostępne bezpośrednio
  g_boot_gpregret = NRF_POWER->GPREGRET;
  g_boot_hf = NRF_POWER->GPREGRET2 == DIAG_HARDFAULT_MARK;
  NRF_POWER->GPREGRET2 = 0;
  Serial.begin(115200);
  led_begin();
  s_ble_ok = ble_begin();          // SoftDevice musi działać przed sd_ppi_*
  ble_set_wait_hook(measure_service);
  timebase_begin(on_tick);
  timebase_request_hfxo();
  s_hall_ok = hall_begin();
  s_thr_ok = throttle_begin();
  s_imu_ok = imu_begin();
  s_flash_ok = flash_begin();
  race_begin();                    // po skanie slotów: wspólna numeracja seq / boot
  drift_default_params(&s_dp);
  drift_init(&s_ds);
  race_set_sample_cb(on_race_sample);
  logger_begin();
  if (!s_flash_ok) logger_set_error();
  est_default_params(&s_ep);
  est_init(&s_es);
}

void loop() {
  uint32_t now = timebase_now();
  imu_poll(now);
  sim_poll(now);
  now = timebase_now();
  drain_ticks(now);
  throttle_poll(now);
  logger_poll();
  race_poll();
  if (logger_state() != LS_SAVING) flash_poll();
  if (vmax_reset_requested()) s_es.v_max = 0.0f;
  if (drift_reset_requested()) drift_reset_score(&s_ds);
  if (mount_changed()) {
    // bias i odniesienia liczone w starych osiach – od nowa
    float vmax = s_es.v_max;
    est_init(&s_es);
    s_es.v_max = vmax;
    s_ds.have_bias = 0;
    s_ds.first = 1;
    drift_reset_score(&s_ds);   // punkty liczone w złych osiach nic nie znaczą
  }

  uint32_t ms = millis();
  links_poll(ms);

  LedInputs li;
  li.recording = logger_state() == LS_RECORDING;
  li.saving = logger_state() == LS_SAVING;
  li.error = logger_state() == LS_ERROR || !s_imu_ok || !s_flash_ok || !s_hall_ok || !s_thr_ok;
  li.no_throttle = !throttle_signal_ok(now);
  li.learning = !throttle_neutral_learned();
  li.connected = ble_connected();
  li.unread_runs = flash_count_runs() > 0 || logger_ram_run_available() || race_count_unread() > 0;
  li.race_recording = race_recording();
  li.race_full = race_state() == RS_FULL || race_state() == RS_DIRTY;
  if (race_state() == RS_ERROR) li.error = true;
  led_poll(li, ms);

  uint32_t free_b = uxTaskGetStackHighWaterMark(nullptr) * 4;
  if (free_b < g_stack_free_min) g_stack_free_min = free_b;
}
