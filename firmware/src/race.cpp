#include "race.h"

#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "crc32.h"
#include "flash_store.h"
#include "imu.h"
#include "logger.h"
#include "throttle.h"

#define PREFIX_MAGIC   0x50463458u   // "X4FP"
#define PREFIX_SIZE    32
#define PAGE           256
// Zapas na trailer i przeskok strony przy domykaniu po zaniku zasilania [B]
#define RESERVE_BYTES  1024
// Najmniej miejsca na rozpoczęcie wyścigu: nagłówki + 5 s próbek [B]
#define MIN_START_BYTES (PREFIX_SIZE + RACE_HEADER_SIZE + RESERVE_BYTES + \
                         5 * RACE_RATE_HZ * sizeof(RaceSample))
#define PRE_N          (RACE_PRETRIGGER_MS * RACE_RATE_HZ / 1000)
#define END_IDLE_N     ((uint32_t)RACE_END_IDLE_S * RACE_RATE_HZ)
#define AREA_END       (RACE_AREA_ADDR + RACE_AREA_SIZE)

static_assert(SAMPLE_RATE_HZ % RACE_RATE_HZ == 0, "RACE_RATE_HZ musi dzielić SAMPLE_RATE_HZ");
static_assert(RACE_DECIM >= 1 && RACE_DECIM <= 254, "RACE_DECIM 1..254");
static_assert(PRE_N >= 1, "RACE_PRETRIGGER_MS za krótki");
static_assert(RACE_BUF_BYTES >= 4 * PAGE, "RACE_BUF_BYTES za mały");

static RaceRec   s_recs[RACE_MAX_RECORDS];
static int       s_n;
static RaceState s_state = RS_ERROR;
static uint32_t  s_write_addr = RACE_AREA_ADDR;   // początek następnego rekordu (strona)
static uint32_t  s_limit = AREA_END;
static uint32_t  s_scan_ms, s_overflows, s_reset_at;
static bool      s_sim;
static void    (*s_sample_cb)(const RaceSample&);

// Uśrednianie taktów do jednej próbki
static struct {
  int32_t  a[6];
  uint32_t thr_sum;
  uint16_t thr_n;
  uint8_t  n;
  uint8_t  flags;       // OR: SF_THROTTLE_VALID | SF_IMU_*
  bool     start_hit;   // gaz > RACE_START_PCT
  bool     active;      // zbocze albo gaz poza neutrum
} s_acc;
static uint32_t s_tick_edges;            // licznik zboczy z poprzedniego taktu
static uint32_t s_prev_edges, s_prev_t;  // z poprzedniej próbki
static bool     s_have_prev;
static uint32_t s_gap_ticks;             // czas próbek zgubionych przez pełny bufor

// Pre-trigger (próbki sprzed gazu)
static RaceSample s_pre[PRE_N];
static uint32_t   s_pre_t[PRE_N];
static uint32_t   s_pre_w, s_pre_n;

// Bieżący rekord
static uint32_t   s_rec_addr, s_data_addr;
static RaceHeader s_hdr;
static uint32_t   s_n_samples, s_idle_n;
static uint32_t   s_crc;
static uint8_t    s_end_reason;
static bool       s_trailer_pending;
static uint8_t    s_buf[RACE_BUF_BYTES];
static uint32_t   s_appended, s_flushed;  // bajty pliku: dopisane do bufora / zapisane do flash

static inline uint32_t page_up(uint32_t a) { return (a + PAGE - 1) & ~(uint32_t)(PAGE - 1); }

static bool all_ff(const uint8_t* p, uint32_t n) {
  for (uint32_t i = 0; i < n; i++)
    if (p[i] != 0xFF) return false;
  return true;
}

static bool write_word(uint32_t addr, uint32_t v) { return flash_raw_write(addr, &v, 4); }

static void update_full(void) {
  if (s_state != RS_IDLE) return;
  if (s_n >= RACE_MAX_RECORDS || s_write_addr >= s_limit || s_limit - s_write_addr < MIN_START_BYTES)
    s_state = RS_FULL;
}

// ------------------------------------------------------------ skan przy starcie
static bool sample_valid(const RaceSample& r) {
  return (r.flags & RSF_MARK_ZERO) == 0 && r.n_ticks >= 1 && r.n_ticks <= 254;
}

// Rekord przerwany zanikiem zasilania (stan FE): liczy poprawne próbki,
// dopisuje trailer na następnej czystej stronie, ustawia długość, CRC i stan FC.
// Zwraca false, gdy nie da się go domknąć (obszar trzeba skasować).
static bool recover(uint32_t addr, uint32_t* len_out) {
  uint32_t data = addr + PREFIX_SIZE;
  RaceHeader h;
  if (!flash_raw_read(data, &h, sizeof(h))) return false;
  if (h.magic != RACE_MAGIC || h.sample_size != sizeof(RaceSample)) {
    // brak nagłówka (zanik tuż po prefiksie) – rekord pusty, pomijany
    *len_out = 0;
    return write_word(addr + 12, 0) && write_word(addr + 8, 0xFFFFFF00u);
  }
  static RaceSample chunk[50];
  uint32_t off = data + RACE_HEADER_SIZE, n = 0;
  bool end = false;
  while (!end && off + sizeof(RaceSample) <= AREA_END - sizeof(RaceTrailer)) {
    uint32_t cnt = min((uint32_t)50, (AREA_END - sizeof(RaceTrailer) - off) / sizeof(RaceSample));
    if (!flash_raw_read(off, chunk, cnt * sizeof(RaceSample))) return false;
    for (uint32_t i = 0; i < cnt; i++) {
      if (!sample_valid(chunk[i])) { end = true; break; }
      n++;
      off += sizeof(RaceSample);
    }
  }
  // Trailer na stronie po tej, na której urwał się zapis (ta mogła zostać
  // zaprogramowana częściowo) – i tylko na czystym miejscu.
  uint32_t tr = (off / PAGE + 1) * PAGE;
  uint8_t probe[sizeof(RaceTrailer)];
  for (;; tr += PAGE) {
    if (tr + sizeof(RaceTrailer) > AREA_END) return false;
    if (!flash_raw_read(tr, probe, sizeof(probe))) return false;
    if (all_ff(probe, sizeof(probe))) break;
  }
  static uint8_t buf[1024] __attribute__((aligned(4)));
  uint32_t crc = crc32_begin();
  for (uint32_t a = data; a < tr; a += sizeof(buf)) {
    uint32_t k = min((uint32_t)sizeof(buf), tr - a);
    if (!flash_raw_read(a, buf, k)) return false;
    crc = crc32_update(crc, buf, k);
  }
  RaceTrailer t;
  memset(&t, 0, sizeof(t));
  t.magic = RACE_TRAILER_MAGIC;
  t.n_samples = n;
  t.end_reason = RACE_END_RECOVERED;
  crc = crc32_end(crc32_update(crc, &t, 12));
  t.crc32 = crc;
  uint32_t len = tr + sizeof(RaceTrailer) - data;
  bool ok = flash_raw_write(tr, &t, sizeof(t)) && write_word(addr + 12, len) &&
            write_word(addr + 16, crc) && write_word(addr + 8, 0xFFFFFF00u | SLOT_COMPLETE);
  *len_out = len;
  return ok;
}

// Weryfikacja CRC rekordu i odczyt metadanych do listy.
static bool verify(uint32_t addr, uint32_t len, uint32_t crc_pfx, RaceRec* r) {
  uint32_t data = addr + PREFIX_SIZE;
  if (len < RACE_HEADER_SIZE + sizeof(RaceTrailer)) return false;
  static uint8_t buf[1024] __attribute__((aligned(4)));
  uint32_t crc = crc32_begin();
  uint32_t body = len - 4;
  for (uint32_t off = 0; off < body; off += sizeof(buf)) {
    uint32_t k = min((uint32_t)sizeof(buf), body - off);
    if (!flash_raw_read(data + off, buf, k)) return false;
    crc = crc32_update(crc, buf, k);
  }
  crc = crc32_end(crc);
  RaceHeader h;
  RaceTrailer t;
  if (!flash_raw_read(data, &h, sizeof(h)) ||
      !flash_raw_read(data + len - sizeof(t), &t, sizeof(t)))
    return false;
  if (crc != crc_pfx || crc != t.crc32 || h.magic != RACE_MAGIC || t.magic != RACE_TRAILER_MAGIC)
    return false;
  r->addr = addr;
  r->seq = h.seq;
  r->len = len;
  r->crc = crc;
  r->unix_s = h.unix_s;
  r->boot_count = h.boot_count;
  r->uptime_ms = h.uptime_ms;
  r->n_samples = t.n_samples;
  r->end_reason = t.end_reason;
  flash_note_boot(h.boot_count);
  return true;
}

void race_begin(void) {
  uint32_t t0 = millis();
  s_n = 0;
  s_state = RS_IDLE;
  if (!flash_ok()) {
    s_state = RS_ERROR;
    return;
  }
  uint32_t addr = RACE_AREA_ADDR;
  while (addr + PREFIX_SIZE <= AREA_END) {
    uint32_t w[5];
    if (!flash_raw_read(addr, w, sizeof(w))) { s_state = RS_ERROR; return; }
    if (all_ff((const uint8_t*)w, sizeof(w))) break;
    if (w[0] != PREFIX_MAGIC || (w[2] | 0xFF) != 0xFFFFFFFFu || s_n >= RACE_MAX_RECORDS) {
      s_state = RS_DIRTY;   // obce dane (np. sloty starego układu flash)
      break;
    }
    uint8_t st = (uint8_t)(w[2] & 0xFF);
    flash_note_seq(w[1]);
    uint32_t len = w[3];
    if (st == SLOT_WRITING) {
      if (!recover(addr, &len)) { s_state = RS_DIRTY; break; }
      st = len ? SLOT_COMPLETE : 0x00;
      if (len && !flash_raw_read(addr + 16, &w[4], 4)) { s_state = RS_ERROR; return; }
    } else if (st != SLOT_COMPLETE && st != SLOT_DOWNLOADED && st != 0x00) {
      s_state = RS_DIRTY;
      break;
    }
    if (len > AREA_END - addr - PREFIX_SIZE) { s_state = RS_DIRTY; break; }
    if (st != 0x00 && len) {
      RaceRec r;
      memset(&r, 0, sizeof(r));
      if (verify(addr, len, w[4], &r)) {   // niespójny rekord pomijany
        r.state = st;
        s_recs[s_n++] = r;
      }
    }
    addr = page_up(addr + PREFIX_SIZE + len);
  }
  s_write_addr = addr;
  // Reszta obszaru musi być skasowana – inaczej zapis po cichu by się nie udał.
  if (s_state == RS_IDLE) {
    static uint8_t buf[1024] __attribute__((aligned(4)));
    for (uint32_t a = s_write_addr; a < AREA_END; a += sizeof(buf)) {
      uint32_t k = min((uint32_t)sizeof(buf), AREA_END - a);
      if (!flash_raw_read(a, buf, k) || !all_ff(buf, k)) {
        s_state = RS_DIRTY;
        break;
      }
    }
  }
  update_full();
  s_scan_ms = millis() - t0;
}

// ------------------------------------------------------------ zapis
static uint32_t buf_free(void) { return RACE_BUF_BYTES - (s_appended - s_flushed); }

static void append(const void* data, uint32_t n, bool with_crc) {
  const uint8_t* p = (const uint8_t*)data;
  if (with_crc) s_crc = crc32_update(s_crc, p, n);
  while (n) {
    uint32_t pos = s_appended % RACE_BUF_BYTES;
    uint32_t k = min(n, (uint32_t)RACE_BUF_BYTES - pos);
    memcpy(s_buf + pos, p, k);
    s_appended += k;
    p += k;
    n -= k;
  }
}

static void finish(uint8_t reason) {
  if (s_state != RS_RECORDING) return;
  s_end_reason = reason;
  s_trailer_pending = true;
  s_state = RS_FINISHING;
}

static void append_sample(RaceSample& r) {
  if (s_data_addr + s_appended + sizeof(RaceSample) + RESERVE_BYTES > s_limit) {
    finish(RACE_END_FULL);
    return;
  }
  if (buf_free() < sizeof(RaceSample) + sizeof(RaceTrailer)) {
    // flash nie nadąża (np. długie czekanie na BLE) – próbka gubiona, czas
    // doliczany do następnej (n_ticks) z flagą RSF_GAP
    s_overflows++;
    s_gap_ticks += r.n_ticks;
    return;
  }
  if (s_gap_ticks) {
    uint32_t t = r.n_ticks + s_gap_ticks;
    r.n_ticks = t > 254 ? 254 : (uint8_t)t;
    r.flags |= RSF_GAP;
    s_gap_ticks = 0;
  }
  append(&r, sizeof(r), true);
  s_n_samples++;
  if (s_reset_at && s_n_samples >= s_reset_at) NVIC_SystemReset();  // SIM: zanik zasilania
}

static void error(void) { s_state = RS_ERROR; }

static void start_record(RaceSample& cur, uint32_t cur_t) {
  update_full();
  if (s_state != RS_IDLE) return;
  s_rec_addr = s_write_addr;
  s_data_addr = s_rec_addr + PREFIX_SIZE;
  uint32_t seq = flash_next_seq();
  flash_note_seq(seq);
  uint32_t pfx[3] = {PREFIX_MAGIC, seq, 0xFFFFFF00u | SLOT_WRITING};
  if (!flash_raw_write(s_rec_addr, pfx, sizeof(pfx))) return error();

  uint32_t first = (s_pre_w + PRE_N - s_pre_n) % PRE_N;
  memset(&s_hdr, 0, sizeof(s_hdr));
  s_hdr.magic = RACE_MAGIC;
  s_hdr.format_version = RACE_FORMAT_VERSION;
  s_hdr.header_size = RACE_HEADER_SIZE;
  s_hdr.seq = seq;
  s_hdr.unix_s = logger_unix_now();
  s_hdr.boot_count = flash_boot_count();
  s_hdr.uptime_ms = millis();
  s_hdr.hall_lines = HALL_LINES;
  s_hdr.motor_pole_pairs = MOTOR_POLE_PAIRS;
  s_hdr.edges_per_rev = EDGES_PER_REV;
  s_hdr.throttle_fwd_higher = THROTTLE_FORWARD_IS_HIGHER;
  s_hdr.sample_rate_hz = RACE_RATE_HZ;
  s_hdr.imu_odr_hz = IMU_ODR_HZ;
  s_hdr.accel_range_g = IMU_ACCEL_RANGE_G;
  s_hdr.gyro_range_dps = IMU_GYRO_RANGE_DPS;
  s_hdr.accel_scale_g = imu_accel_scale_g();
  s_hdr.gyro_scale_dps = imu_gyro_scale_dps();
  s_hdr.throttle_neutral_us = throttle_neutral_learned() ? throttle_neutral_us() : 0;
  s_hdr.throttle_full_us = THROTTLE_FULL_SCALE_US;
  s_hdr.trigger_index = s_pre_n;
  s_hdr.tick_rate_hz = SAMPLE_RATE_HZ;
  s_hdr.decim = RACE_DECIM;
  s_hdr.flags = (logger_time_set() ? RF_TIME_SET : 0) |
                (throttle_neutral_learned() ? RF_NEUTRAL_LEARNED : 0) |
                (imu_ok() ? 0 : RF_IMU_ERROR) | (s_sim ? RF_SIMULATED : 0);
  s_hdr.t0_us = s_pre_n ? s_pre_t[first] : cur_t;
  s_hdr.sample_size = sizeof(RaceSample);
  s_hdr.trailer_size = sizeof(RaceTrailer);
  s_hdr.fw_version = (FW_VERSION_MAJOR << 16) | (FW_VERSION_MINOR << 8) | FW_VERSION_PATCH;
  s_hdr.period_unit_us = RACE_PERIOD_UNIT_US;
  s_hdr.pretrigger_ms = RACE_PRETRIGGER_MS;
  s_hdr.end_idle_s = RACE_END_IDLE_S;
  s_hdr.start_pct = RACE_START_PCT;
  s_hdr.hall_pullup_v_x10 = HALL_PULLUP_V_X10;
  s_hdr.hall_timeout_us = HALL_TIMEOUT_US;
  s_hdr.hall_min_edge_us = HALL_MIN_EDGE_US;

  s_appended = s_flushed = 0;
  s_crc = crc32_begin();
  s_n_samples = 0;
  s_idle_n = 0;
  s_gap_ticks = 0;
  s_trailer_pending = false;
  s_state = RS_RECORDING;
  append(&s_hdr, sizeof(s_hdr), true);
  for (uint32_t i = 0; i < s_pre_n; i++) append_sample(s_pre[(first + i) % PRE_N]);
  s_pre_n = 0;
  append_sample(cur);
}

static void complete_record(void) {
  uint32_t len = s_appended;
  uint32_t crc = s_crc;
  bool ok = write_word(s_rec_addr + 12, len) && write_word(s_rec_addr + 16, crc) &&
            write_word(s_rec_addr + 8, 0xFFFFFF00u | SLOT_COMPLETE);
  if (!ok) return error();
  if (s_n < RACE_MAX_RECORDS) {
    RaceRec& r = s_recs[s_n++];
    r.addr = s_rec_addr;
    r.seq = s_hdr.seq;
    r.len = len;
    r.crc = crc;
    r.unix_s = s_hdr.unix_s;
    r.boot_count = s_hdr.boot_count;
    r.uptime_ms = s_hdr.uptime_ms;
    r.n_samples = s_n_samples;
    r.state = SLOT_COMPLETE;
    r.end_reason = s_end_reason;
  }
  s_write_addr = page_up(s_data_addr + len);
  s_have_prev = false;
  s_state = s_end_reason == RACE_END_FULL ? RS_FULL : RS_IDLE;
  update_full();
}

void race_poll(void) {
  if (s_state == RS_ERASING) {
    if (!flash_race_erasing()) {
      s_n = 0;
      s_write_addr = RACE_AREA_ADDR;
      s_pre_n = 0;
      s_have_prev = false;
      s_state = RS_IDLE;
      update_full();
    }
    return;
  }
  if (s_state != RS_RECORDING && s_state != RS_FINISHING) return;
  if (s_trailer_pending && buf_free() >= sizeof(RaceTrailer)) {
    RaceTrailer t;
    memset(&t, 0, sizeof(t));
    t.magic = RACE_TRAILER_MAGIC;
    t.n_samples = s_n_samples;
    t.end_reason = s_end_reason;
    append(&t, 12, true);
    s_crc = crc32_end(s_crc);
    t.crc32 = s_crc;
    append(&t.crc32, 4, false);
    s_trailer_pending = false;
  }
  // Do 2 stron na wywołanie, tylko gdy flash wolny (bez czekania na kasowanie).
  for (int k = 0; k < 2; k++) {
    uint32_t pend = s_appended - s_flushed;
    if (!pend) break;
    uint32_t addr = s_data_addr + s_flushed;
    uint32_t to_page = PAGE - (addr % PAGE);
    if (pend < to_page && s_state != RS_FINISHING) break;   // czekamy na pełną stronę
    if (flash_raw_busy()) return;
    uint32_t pos = s_flushed % RACE_BUF_BYTES;
    uint32_t n = min(min(pend, to_page), (uint32_t)RACE_BUF_BYTES - pos);
    if (!flash_raw_write(addr, s_buf + pos, n)) return error();
    s_flushed += n;
  }
  if (s_state == RS_FINISHING && !s_trailer_pending && s_appended == s_flushed) complete_record();
}

// ------------------------------------------------------------ próbki
void race_push(const Sample& s, float pct, bool thr_valid, bool learned) {
  bool moving = s.hall_edges != s_tick_edges;
  s_tick_edges = s.hall_edges;
  // Próbki 20 Hz powstają zawsze (punkty driftu liczone na żywo także przy pełnej pamięci);
  // stan pamięci decyduje tylko o zapisie.

  s_acc.a[0] += s.ax; s_acc.a[1] += s.ay; s_acc.a[2] += s.az;
  s_acc.a[3] += s.gx; s_acc.a[4] += s.gy; s_acc.a[5] += s.gz;
  if (thr_valid) {
    s_acc.thr_sum += s.throttle_us;
    s_acc.thr_n++;
  }
  s_acc.flags |= s.flags & (SF_THROTTLE_VALID | SF_IMU_OVERRUN | SF_IMU_STALE);
  if (thr_valid && learned && pct > RACE_START_PCT) s_acc.start_hit = true;
  if (moving || (thr_valid && fabsf(pct) >= ARM_NEUTRAL_PCT)) s_acc.active = true;
  if (++s_acc.n < RACE_DECIM) return;

  RaceSample r;
  uint32_t de = s_have_prev ? s.hall_edges - s_prev_edges : 0;
  r.edges = de > 0xFFFF ? 0xFFFF : (uint16_t)de;
  uint32_t per = s.period_rev_us ? (s.period_rev_us + RACE_PERIOD_UNIT_US / 2) / RACE_PERIOD_UNIT_US : 0;
  r.period_rev = per > 0xFFFF ? 0 : (per == 0 && s.period_rev_us ? 1 : (uint16_t)per);
  r.throttle_us = s_acc.thr_n ? (uint16_t)((s_acc.thr_sum + s_acc.thr_n / 2) / s_acc.thr_n) : 0;
  int16_t avg[6];
  for (int i = 0; i < 6; i++) avg[i] = (int16_t)lroundf((float)s_acc.a[i] / s_acc.n);
  r.ax = avg[0]; r.ay = avg[1]; r.az = avg[2];
  r.gx = avg[3]; r.gy = avg[4]; r.gz = avg[5];
  r.flags = s_acc.flags | (s.flags & (SF_HALL_VALID | SF_DIR_FORWARD | SF_DIR_VALID));
  uint32_t ticks = s_have_prev ? ((s.t_us - s_prev_t) + SAMPLE_PERIOD_US / 2) / SAMPLE_PERIOD_US
                               : RACE_DECIM;
  r.n_ticks = ticks < 1 ? 1 : (ticks > 254 ? 254 : (uint8_t)ticks);
  bool start_hit = s_acc.start_hit, active = s_acc.active;
  memset(&s_acc, 0, sizeof(s_acc));
  s_prev_edges = s.hall_edges;
  s_prev_t = s.t_us;
  s_have_prev = true;
  if (s_sample_cb) s_sample_cb(r);
  if (s_state != RS_IDLE && s_state != RS_RECORDING) return;

  if (s_state == RS_IDLE) {
    if (start_hit) {
      start_record(r, s.t_us);
    } else {
      s_pre[s_pre_w] = r;
      s_pre_t[s_pre_w] = s.t_us;
      s_pre_w = (s_pre_w + 1) % PRE_N;
      if (s_pre_n < PRE_N) s_pre_n++;
    }
    return;
  }
  // RS_RECORDING
  s_idle_n = active ? 0 : s_idle_n + 1;
  append_sample(r);
  if (s_state == RS_RECORDING && s_idle_n >= END_IDLE_N) finish(RACE_END_IDLE);
}

// ------------------------------------------------------------ dostęp
RaceState race_state(void) { return s_state; }

const char* race_state_name(void) {
  switch (s_state) {
    case RS_IDLE: return "IDLE";
    case RS_RECORDING: return "RECORDING";
    case RS_FINISHING: return "FINISHING";
    case RS_FULL: return "FULL";
    case RS_ERASING: return "ERASING";
    case RS_DIRTY: return "DIRTY";
    default: return "ERROR";
  }
}

bool race_recording(void) { return s_state == RS_RECORDING || s_state == RS_FINISHING; }
int race_count(void) { return s_n; }

int race_count_unread(void) {
  int n = 0;
  for (int i = 0; i < s_n; i++) n += s_recs[i].state == SLOT_COMPLETE;
  return n;
}

const RaceRec* race_rec(int i) { return (i >= 0 && i < s_n) ? &s_recs[i] : nullptr; }

int race_find(uint32_t seq) {
  for (int i = 0; i < s_n; i++)
    if (s_recs[i].seq == seq) return i;
  return -1;
}

bool race_read(int i, uint32_t offset, void* buf, uint32_t len) {
  if (i < 0 || i >= s_n || offset > s_recs[i].len || len > s_recs[i].len - offset) return false;
  return flash_raw_read(s_recs[i].addr + PREFIX_SIZE + offset, buf, len);
}

bool race_mark_downloaded(uint32_t seq) {
  int i = race_find(seq);
  if (i < 0) return false;
  if (s_recs[i].state == SLOT_DOWNLOADED) return true;
  if (!write_word(s_recs[i].addr + 8, 0xFFFFFF00u | SLOT_DOWNLOADED)) return false;
  s_recs[i].state = SLOT_DOWNLOADED;
  return true;
}

int race_erase(bool force) {
  if (race_recording()) return -1;
  if (s_state == RS_ERASING) return 0;
  if (!force && (race_count_unread() > 0 || s_state == RS_DIRTY)) return -2;
  flash_race_erase_start();
  s_state = RS_ERASING;
  memset(&s_acc, 0, sizeof(s_acc));
  return 0;
}

void race_stop(void) { finish(RACE_END_COMMAND); }

uint32_t race_used_bytes(void) {
  uint32_t end = race_recording() ? s_data_addr + s_appended : s_write_addr;
  return end - RACE_AREA_ADDR;
}

uint32_t race_size_bytes(void) { return s_limit - RACE_AREA_ADDR; }
uint32_t race_current_s(void) { return race_recording() ? s_n_samples / RACE_RATE_HZ : 0; }
uint32_t race_overflows(void) { return s_overflows; }
uint32_t race_scan_ms(void) { return s_scan_ms; }
void race_set_sim_flag(bool on) { s_sim = on; }
void race_set_sample_cb(void (*cb)(const RaceSample&)) { s_sample_cb = cb; }
void race_set_reset_at(uint32_t n) { s_reset_at = n; }

void race_set_limit(uint32_t bytes) {
  s_limit = (bytes && bytes < RACE_AREA_SIZE) ? RACE_AREA_ADDR + bytes : AREA_END;
  if (s_state == RS_FULL && s_n < RACE_MAX_RECORDS) s_state = RS_IDLE;
  update_full();
}
