// Zapis całych wyścigów do obszaru wyścigów we flash (RACE_AREA_*).
// Próbka RACE_RATE_HZ = średnia z taktów 500 Hz. Wyścig zaczyna się sam
// (gaz > RACE_START_PCT, z RACE_PRETRIGGER_MS wstecz), kończy po RACE_END_IDLE_S
// postoju. Rekordy dopisywane po kolei; pełna pamięć = stop do RERASE.
// Układ obszaru: rekordy od granic stron 256 B:
//   prefiks 32 B: +0 magic "X4FP", +4 seq, +8 stan (bajt niski, FF→FE→FC→F8),
//                 +12 długość danych, +16 CRC32 danych (= CRC z trailera)
//   dane od +32: plik wyścigu (RaceHeader + próbki + [wypełnienie] + RaceTrailer)
// Zanik zasilania w trakcie (stan FE) → przy starcie rekord domykany trailerem
// (powód RACE_END_RECOVERED).
#pragma once

#include <stdint.h>
#include "run_format.h"

enum RaceState : uint8_t {
  RS_IDLE,        // czeka na gaz (jest miejsce)
  RS_RECORDING,
  RS_FINISHING,   // dopisywanie końcówki i trailera
  RS_FULL,        // brak miejsca – czeka na RERASE
  RS_ERASING,
  RS_DIRTY,       // obszar zawiera obce dane (np. stare sloty startów) – RERASE FORCE
  RS_ERROR,       // błąd zapisu flash
};

struct RaceRec {
  uint32_t addr;        // adres prefiksu
  uint32_t seq, len, crc;
  uint32_t unix_s, boot_count, uptime_ms;
  uint32_t n_samples;
  uint8_t  state;       // SLOT_COMPLETE / SLOT_DOWNLOADED
  uint8_t  end_reason;
};

void        race_begin(void);       // po flash_begin(): skan, domknięcie przerwanego
void        race_push(const Sample& s, float throttle_pct, bool throttle_valid, bool neutral_learned,
                      uint16_t steer_us);
void        race_poll(void);        // zapis bufora do flash (nieblokujący) – pętla i wait hook
RaceState   race_state(void);
const char* race_state_name(void);
bool        race_recording(void);   // RECORDING albo FINISHING
int         race_count(void);       // kompletne + pobrane
int         race_count_unread(void);
const RaceRec* race_rec(int i);     // od najstarszego
int         race_find(uint32_t seq);
bool        race_read(int i, uint32_t offset, void* buf, uint32_t len);
bool        race_mark_downloaded(uint32_t seq);
// 0 = OK, -1 = trwa nagrywanie, -2 = niepobrane wyścigi (bez force)
int         race_erase(bool force);
void        race_stop(void);        // RSTOP
uint32_t    race_used_bytes(void);
uint32_t    race_size_bytes(void);  // z ewentualnym limitem testowym
uint32_t    race_current_s(void);   // długość bieżącego nagrania [s]
uint32_t    race_overflows(void);   // próbki zgubione przez pełny bufor RAM
uint32_t    race_scan_ms(void);     // czas skanu przy starcie
void        race_set_sim_flag(bool on);
// Wołane dla każdej próbki RACE_RATE_HZ, niezależnie od stanu pamięci (punkty driftu)
void        race_set_sample_cb(void (*cb)(const RaceSample&));
void        race_set_limit(uint32_t bytes);     // SIM: mniejszy obszar (0 = pełny)
void        race_set_reset_at(uint32_t n);      // SIM: reset po n próbkach (test zaniku zasilania)
