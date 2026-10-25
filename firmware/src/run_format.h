// Format binarny startu – wspólny dla flash, BLE i plików.
// Little-endian. Opis pól: docs/FORMAT.md. Zmiana = podbicie RUN_FORMAT_VERSION
// i zmiana w app/js/format.js oraz tools/rc_launch/parse.py.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "config.h"

#define RUN_MAGIC           0x52463458u   // "X4FR"
#define RUN_HEADER_SIZE     128

// Flagi próbki
#define SF_HALL_VALID       0x01  // period_rev_us znany
#define SF_THROTTLE_VALID   0x02  // throttle_us poprawny
#define SF_DIR_FORWARD      0x04  // kierunek (tylko gdy SF_DIR_VALID)
#define SF_DIR_VALID        0x08  // kierunek znany (tryb 3)
#define SF_IMU_OVERRUN      0x10  // FIFO IMU przepełnione od poprzedniej próbki
#define SF_IMU_STALE        0x20  // brak świeżej próbki IMU (powtórzona)
#define SF_TRIGGER          0x40  // próbka wyzwolenia

// Flagi startu (nagłówek)
#define RF_EDGES_TRUNCATED  0x0001
#define RF_TIME_SET         0x0002  // unix_s poprawny
#define RF_NEUTRAL_LEARNED  0x0004
#define RF_IMU_ERROR        0x0008
#define RF_RAM_ONLY         0x0010  // nie zapisany do flash (błąd zapisu)
#define RF_SIMULATED        0x0020  // dane z komend SIM_* (test na biurku)

struct __attribute__((packed)) Sample {
  uint32_t t_us;          // znacznik czasu próbki
  uint32_t hall_edges;    // licznik zboczy (narastający)
  uint32_t period_rev_us; // ostatni okres pełnego obrotu, 0 = brak/postój
  uint16_t throttle_us;   // szerokość impulsu gazu, 0 = brak sygnału
  int16_t  ax, ay, az;
  int16_t  gx, gy, gz;
  uint8_t  flags;         // SF_*
  uint8_t  reserved;
};
static_assert(sizeof(Sample) == 28, "Sample musi mieć 28 B");

// Surowe zbocze: bity 0..28 = czas względem t0_us [us], bit 29 = poziom po
// zboczu, bity 30..31 = linia (0=A, 1=B, 2=C). Okno nagrania << 536 s.
typedef uint32_t EdgeWord;
#define EDGE_T_MASK         0x1FFFFFFFu
#define EDGE_LEVEL_BIT      29
#define EDGE_LINE_SHIFT     30

struct __attribute__((packed)) RunHeader {
  uint32_t magic;               //  0 RUN_MAGIC
  uint16_t format_version;      //  4
  uint16_t header_size;         //  6 = 128
  uint32_t seq;                 //  8
  uint32_t unix_s;              // 12 0 = nieznany
  uint32_t boot_count;          // 16
  uint32_t uptime_ms;           // 20 w chwili wyzwolenia
  uint8_t  hall_lines;          // 24
  uint8_t  motor_pole_pairs;    // 25
  uint8_t  edges_per_rev;       // 26
  uint8_t  throttle_fwd_higher; // 27
  uint16_t sample_rate_hz;      // 28
  uint16_t imu_odr_hz;          // 30
  uint16_t accel_range_g;       // 32
  uint16_t gyro_range_dps;      // 34
  float    accel_scale_g;       // 36 g / LSB
  float    gyro_scale_dps;      // 40 dps / LSB
  uint16_t throttle_neutral_us; // 44 0 = nieznane
  uint16_t throttle_full_us;    // 46 THROTTLE_FULL_SCALE_US
  uint32_t trigger_index;       // 48 indeks próbki wyzwolenia
  uint32_t n_samples;           // 52
  uint32_t n_edges;             // 56
  uint32_t flags;               // 60 RF_*
  uint32_t t0_us;               // 64 t_us pierwszej próbki (baza czasu zboczy)
  uint32_t trigger_t_us;        // 68
  uint16_t sample_size;         // 72 = 28
  uint16_t edge_size;           // 74 = 4
  uint32_t fw_version;          // 76 major<<16 | minor<<8 | patch
  uint16_t hall_min_edge_us;    // 80
  uint16_t pretrigger_ms;       // 82
  uint16_t posttrigger_ms;      // 84
  uint8_t  trigger_pct;         // 86
  uint8_t  hall_pullup_v_x10;   // 87
  uint32_t hall_timeout_us;     // 88
  uint8_t  reserved[32];        // 92..123 = 0
  uint32_t crc32;               // 124 CRC32 (IEEE) całości z tym polem = 0
};
static_assert(sizeof(RunHeader) == RUN_HEADER_SIZE, "RunHeader musi mieć 128 B");

static inline uint32_t run_total_size(uint32_t n_samples, uint32_t n_edges) {
  return RUN_HEADER_SIZE + n_samples * sizeof(Sample) + n_edges * sizeof(EdgeWord);
}

// Bufor zboczy: tylko gdy LOG_RAW_EDGES
#if LOG_RAW_EDGES
#define EDGE_LOG_LEN        MAX_EDGES
#else
#define EDGE_LOG_LEN        0
#endif

static_assert(FLASH_SLOT_HDR_SIZE + RUN_HEADER_SIZE + MAX_SAMPLES * sizeof(Sample) +
              EDGE_LOG_LEN * sizeof(EdgeWord) <= FLASH_SLOT_SIZE,
              "Start nie mieści się w slocie flash");
static_assert(FLASH_SLOT_SIZE % FLASH_ERASE_BLOCK == 0, "Slot = wielokrotność 64 kB");
static_assert(GET_BLOCK_MAX % 4 == 0, "GET_BLOCK_MAX wielokrotność 4");

// Ramka bloku danych (odpowiedź na GET)
#define BLOCK_MAGIC         0x42463458u   // "X4FB"
struct __attribute__((packed)) BlockHeader {
  uint32_t magic;
  uint32_t seq;
  uint32_t offset;
  uint32_t len;
  uint32_t crc32;   // CRC32 danych bloku
};
static_assert(sizeof(BlockHeader) == 20, "BlockHeader 20 B");

// Typy w strumieniu (BLE: pierwszy bajt notyfikacji; USB: bajt przed ramką binarną)
#define FRAME_TEXT          0x01
#define FRAME_BINARY        0x02

// ------------------------------------------------------------ wyścig
// Plik wyścigu = RaceHeader + n × RaceSample + [wypełnienie] + RaceTrailer.
// Format 1 (fw 1.1–1.2): próbka 20 B; format 2 (fw 1.3): 22 B – dopisany skręt na końcu.
// Trailer to zawsze ostatnie 16 B pliku (po zaniku zasilania między próbkami
// a trailerem może być wypełnienie – liczy się do CRC, parser je pomija).
#define RACE_MAGIC          0x4C463458u   // "X4FL"
#define RACE_TRAILER_MAGIC  0x45463458u   // "X4FE"
#define RACE_HEADER_SIZE    128

// Flagi próbki wyścigu (bity 0..5 jak SF_*, z całego okresu próbki: OR, hall/dir z ostatniego taktu)
#define RSF_GAP             0x40  // przed tą próbką zgubiono dane (przepełnienie bufora)
#define RSF_MARK_ZERO       0x80  // zawsze 0 – odróżnia próbkę od pustego flash (0xFF)

struct __attribute__((packed)) RaceSample {
  uint16_t edges;         // przyrost licznika zboczy w okresie próbki
  uint16_t period_rev;    // ostatni okres obrotu [RACE_PERIOD_UNIT_US], 0 = postój/nieznany
  uint16_t throttle_us;   // średnia z poprawnych impulsów, 0 = brak sygnału
  int16_t  ax, ay, az;    // średnie surowe z okresu próbki
  int16_t  gx, gy, gz;
  uint8_t  flags;         // SF_* | RSF_GAP; bit 7 = 0
  uint8_t  n_ticks;       // czas próbki w taktach 1/SAMPLE_RATE_HZ (zwykle RACE_DECIM), 1..254
  uint16_t steer_us;      // format 2: średnia szerokość impulsu skrętu, 0 = brak sygnału
};
static_assert(sizeof(RaceSample) == 22, "RaceSample (format 2) musi mieć 22 B");

struct __attribute__((packed)) RaceHeader {
  uint32_t magic;               //  0 RACE_MAGIC
  uint16_t format_version;      //  4 RACE_FORMAT_VERSION
  uint16_t header_size;         //  6 = 128
  uint32_t seq;                 //  8 (wspólna numeracja ze startami)
  uint32_t unix_s;              // 12 0 = nieznany
  uint32_t boot_count;          // 16
  uint32_t uptime_ms;           // 20 w chwili rozpoczęcia (gaz)
  uint8_t  hall_lines;          // 24
  uint8_t  motor_pole_pairs;    // 25
  uint8_t  edges_per_rev;       // 26
  uint8_t  throttle_fwd_higher; // 27
  uint16_t sample_rate_hz;      // 28 RACE_RATE_HZ
  uint16_t imu_odr_hz;          // 30
  uint16_t accel_range_g;       // 32
  uint16_t gyro_range_dps;      // 34
  float    accel_scale_g;       // 36
  float    gyro_scale_dps;      // 40
  uint16_t throttle_neutral_us; // 44
  uint16_t throttle_full_us;    // 46
  uint32_t trigger_index;       // 48 indeks próbki, w której wciśnięto gaz
  uint16_t tick_rate_hz;        // 52 SAMPLE_RATE_HZ (jednostka n_ticks)
  uint16_t decim;               // 54 RACE_DECIM
  uint32_t flags;               // 56 RF_*
  uint32_t t0_us;               // 60 koniec pierwszej próbki (zegar Xiao)
  uint16_t sample_size;         // 64 = 20
  uint16_t trailer_size;        // 66 = 16
  uint32_t fw_version;          // 68
  uint16_t period_unit_us;      // 72 RACE_PERIOD_UNIT_US
  uint16_t pretrigger_ms;       // 74
  uint16_t end_idle_s;          // 76
  uint8_t  start_pct;           // 78
  uint8_t  hall_pullup_v_x10;   // 79
  uint32_t hall_timeout_us;     // 80
  uint16_t hall_min_edge_us;    // 84
  uint16_t steer_neutral_us;    // 86 format 2: neutrum skrętu, 0 = nienauczone / brak
  uint16_t steer_full_us;       // 88 format 2: neutrum → pełny skręt [us]
  uint8_t  reserved[38];        // 90..127 = 0
};
static_assert(sizeof(RaceHeader) == RACE_HEADER_SIZE, "RaceHeader musi mieć 128 B");

// Powód zakończenia wyścigu
#define RACE_END_IDLE       0   // postój RACE_END_IDLE_S
#define RACE_END_FULL       1   // koniec pamięci
#define RACE_END_RECOVERED  2   // zanik zasilania – domknięty przy starcie
#define RACE_END_COMMAND    3   // RSTOP

struct __attribute__((packed)) RaceTrailer {
  uint32_t magic;               // RACE_TRAILER_MAGIC
  uint32_t n_samples;
  uint8_t  end_reason;          // RACE_END_*
  uint8_t  reserved[3];
  uint32_t crc32;               // CRC32 (IEEE) wszystkich bajtów pliku przed tym polem
};
static_assert(sizeof(RaceTrailer) == 16, "RaceTrailer 16 B");

// Specjalny seq startu, który został tylko w RAM (błąd zapisu flash)
#define RAM_RUN_SEQ         0u
