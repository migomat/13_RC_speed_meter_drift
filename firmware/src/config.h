// X4F Launch Logger – wszystkie parametry kompilacji.
// Jednostki w nazwach albo w komentarzach. Wartości oznaczone
// „DO ZMIERZENIA” trzeba potwierdzić na prawdziwym aucie.
#pragma once

#include <stdint.h>

#define FW_VERSION_MAJOR    1
#define FW_VERSION_MINOR    2
#define FW_VERSION_PATCH    0
#define FW_VERSION_STR      "1.2.0"

// ---------------------------------------------------------------- hall
// Tryb czujnika halla: 1 (tylko linia A) albo 3 (A/B/C).
// Można nadpisać z linii poleceń: -DHALL_LINES=3
#ifndef HALL_LINES
#define HALL_LINES          1
#endif
#if HALL_LINES != 1 && HALL_LINES != 3
#error "HALL_LINES musi być 1 albo 3"
#endif

#define MOTOR_POLE_PAIRS    1       // DO ZMIERZENIA / potwierdzenia
#define EDGES_PER_REV       (2 * HALL_LINES * MOTOR_POLE_PAIRS)
// [us] krótsze impulsy na linii = zakłócenie albo drgnięcie wirnika przy ruszaniu
// (zmierzone: pary zboczy co 130–270 µs). 40 % połówki okresu linii przy MAX_MOTOR_RPM
// (40 000 rpm → 750 µs) = 300 µs – szybsze zbocza fizycznie niemożliwe.
#define HALL_MIN_EDGE_US    (60000000UL / (MAX_MOTOR_RPM * 2UL * MOTOR_POLE_PAIRS) * 2 / 5)
// [us] brak zbocza → prędkość 0. Najniższe mierzalne obroty:
// 60e6 / (EDGES_PER_REV · timeout). Tryb 1 przy 50 ms nie zmierzyłby 500 rpm
// (zbocze co 60 ms), dlatego 80 ms (min. 375 rpm); tryb 3: 50 ms (200 rpm).
#if HALL_LINES == 1
#define HALL_TIMEOUT_US     80000
#else
#define HALL_TIMEOUT_US     50000
#endif
#define MAX_MOTOR_RPM       40000   // [rpm] do wymiarowania bufora zboczy

// Stan wysoki linii halla – zmierzony 2026-10-19: 3,8 V (do potwierdzenia, że
// linia przełącza się 3,8 V ↔ ~0 V przy obracaniu kołem). Dzielnik 47k (przy
// lucie) + 150k do masy → 2,89 V na pinie (próg „1” nRF52: 2,31 V, maks. 3,6 V).
// Firmware tej wartości nie używa – tylko informacyjnie w nagłówku startu (×10).
#define HALL_PULLUP_V_X10   38

// ---------------------------------------------------------------- piny
#define PIN_THROTTLE        D0
#define PIN_HALL_A          D1
#define PIN_HALL_B          D2      // tylko HALL_LINES == 3
#define PIN_HALL_C          D3      // tylko HALL_LINES == 3

// ---------------------------------------------------------------- gaz
#define THROTTLE_MIN_US             800    // [us] impulsy spoza zakresu odrzucane
#define THROTTLE_MAX_US             2200   // [us]
#define THROTTLE_TIMEOUT_MS         100    // [ms] brak impulsu → brak sygnału
#define THROTTLE_NEUTRAL_LEARN_MS   2000   // [ms] okno uczenia neutrum
#define THROTTLE_NEUTRAL_SPREAD_US  10     // [us] maks. rozrzut w oknie
#define THROTTLE_FULL_SCALE_US      500    // [us] neutrum → pełny gaz (100 %)
#define THROTTLE_FORWARD_IS_HIGHER  1      // 1: „do przodu” = dłuższy impuls (zależy od reverse w aparaturze)

// ---------------------------------------------------------------- logger
#define SAMPLE_RATE_HZ      500
#define SAMPLE_PERIOD_US    (1000000 / SAMPLE_RATE_HZ)
#define PRETRIGGER_MS       1000   // [ms] mieści spoczynek + wolniejsze wciśnięcie gazu
#define POSTTRIGGER_MS      3000
#define TRIGGER_THROTTLE_PCT 60     // [% zakresu do przodu od neutrum]
// Uzbrojenie tylko po postoju: gaz w neutrum i brak zboczy halla przez ARM_REST_MS.
// Próg trzeba przekroczyć w ciągu LAUNCH_WINDOW_MS od zejścia z postoju – manewrowanie
// (mały gaz, nawroty przód/tył) nie wyzwala nagrania.
#define ARM_NEUTRAL_PCT     10      // [%] |gaz| poniżej = neutrum
#define ARM_REST_MS         300     // [ms]
#define LAUNCH_WINDOW_MS    700     // [ms]

#define PRE_SAMPLES         (PRETRIGGER_MS * SAMPLE_RATE_HZ / 1000)
#define POST_SAMPLES        (POSTTRIGGER_MS * SAMPLE_RATE_HZ / 1000)
#define MAX_SAMPLES         (PRE_SAMPLES + POST_SAMPLES)

#define LOG_RAW_EDGES       1       // zapis surowych znaczników czasu zboczy
// Bufor zboczy: maks. obroty × zbocza/obrót × długość okna (+10 % zapasu)
#define MAX_EDGES           ((uint32_t)((uint64_t)MAX_MOTOR_RPM * EDGES_PER_REV * \
                              (PRETRIGGER_MS + POSTTRIGGER_MS) * 11 / (60ULL * 1000 * 10)))

// ---------------------------------------------------------------- IMU
#define IMU_ACCEL_RANGE_G   4       // ±4 g
#define IMU_GYRO_RANGE_DPS  1000    // ±1000 dps (drift: przekładki i obroty przekraczają 500 dps)
#define IMU_ODR_HZ          833
#define IMU_I2C_ADDR        0x6A    // LSM6DS3TR-C na Wire1 (SA0 = 0)
#define IMU_I2C_HZ          400000  // TWIM1: 400 kHz to maksimum nRF52840 (informacyjnie)

// ---------------------------------------------------------------- flash
#define FLASH_SLOT_SIZE     (128 * 1024)  // wielokrotność bloku kasowania 64 kB
#define FLASH_TOTAL_SIZE    (2 * 1024 * 1024)  // P25Q16H
#define FLASH_NUM_SLOTS     8             // starty: 8 × 128 kB = 1 MB, reszta = obszar wyścigów
#define FLASH_ERASE_BLOCK   (64 * 1024)
#define FLASH_SLOT_HDR_SIZE 256           // nagłówek slotu = 1 strona
#define GET_BLOCK_MAX       4096          // [B] maks. rozmiar bloku przy pobieraniu
#define SAVE_CHUNK_BYTES    1024          // [B] zapis na pętlę główną w SAVING

// ---------------------------------------------------------------- wyścigi
// Ciągły zapis całego wyścigu do drugiej części flash (bez nadpisywania):
// kasowanie tylko w całości komendą RERASE z aplikacji, pełna pamięć = stop.
#define RACE_AREA_ADDR      (FLASH_NUM_SLOTS * FLASH_SLOT_SIZE)
#define RACE_AREA_SIZE      (FLASH_TOTAL_SIZE - RACE_AREA_ADDR)   // 1 MB
#define RACE_RATE_HZ        20      // [Hz] próbka = średnia z SAMPLE_RATE_HZ / RACE_RATE_HZ taktów
                                    // (20 Hz: 1 MB ≈ 43 min; z tych próbek liczone są też punkty driftu)
#define RACE_DECIM          (SAMPLE_RATE_HZ / RACE_RATE_HZ)
#define RACE_PRETRIGGER_MS  1000    // [ms] zapis sprzed pierwszego gazu
#define RACE_START_PCT      30      // [% do przodu] gaz rozpoczynający wyścig
// [s] koniec wyścigu: koła stoją i gaz w neutrum (albo brak sygnału gazu) tak długo.
// Dłużej niż typowe czekanie na sędziego po wywrotce – krótsza przerwa nie dzieli wyścigu.
#define RACE_END_IDLE_S     30
#define RACE_PERIOD_UNIT_US 4       // [us] rozdzielczość okresu obrotu w próbce (uint16 → maks. 262 ms)
#define RACE_BUF_BYTES      8192    // [B] bufor RAM przed zapisem do flash (20 s przy 20 Hz)
#define RACE_MAX_RECORDS    32      // maks. wyścigów w obszarze

// ---------------------------------------------------------------- prędkościomierz
// Te same wartości domyślne co tools/config.yaml i ustawienia aplikacji.
#define FINAL_DRIVE_RATIO       5.0f    // DO UZUPEŁNIENIA
#define FRONT_TIRE_DIAMETER_MM  64.0f   // DO UZUPEŁNIENIA, kalibrowane
// Mapowanie osi płytki na osie auta: indeks osi płytki (0=x,1=y,2=z) i znak
#define IMU_AXIS_FORWARD        0
#define IMU_SIGN_FORWARD        (+1)
#define IMU_AXIS_LEFT           1
#define IMU_SIGN_LEFT           (+1)
#define IMU_AXIS_UP             2
#define IMU_SIGN_UP             (+1)
#define IMU_YAW_OFFSET_DEG      0.0f    // [deg] kąt montażu płytki w poziomie
// Osie i kąt montażu można zmienić w działaniu komendą MOUNT (aplikacja wysyła wynik
// kalibracji po połączeniu); wartości tutaj obowiązują po włączeniu zasilania.
#define ZUPT_MS                 200     // [ms] czas bezruchu do zerowania
#define ZUPT_GYRO_DPS           4.0f    // [dps] maks. |ω| (po odjęciu biasu) w bezruchu
#define ZUPT_ACC_G              0.06f   // [g] maks. ||a|_f − 1 g| w bezruchu (po filtrze)
#define ZUPT_LPF_TAU_S          0.1f    // [s] filtr |a|, |ω| przed progami ZUPT (drgania wentylatora ESC)
#define SLIP_ACC_THR            3.0f    // [m/s²] |a_front − a_long| = brak buksowania
#define SLIP_V_THR              0.5f    // [m/s] |v_front − v| = brak buksowania
#define GRIP_HOLD_MS            100     // [ms] jak długo warunek przyspieszeń → korekta
#define WHEEL_CORR_TAU_S        0.5f    // [s] stała czasowa ściągania do v_front
#define BRAKE_THROTTLE_PCT      (-10)   // [%] poniżej = strefa hamowania
#define EST_AFRONT_TAU_S        0.03f   // [s] wygładzanie a_front / a_long w estymatorze
#define LIVE_RATE_HZ            20

// ---------------------------------------------------------------- drift
// Punkty za drift (drift.c) – te same nazwy i wartości co tools/rc_launch/drift.py
// (DEFAULT_DRIFT), tools/config.yaml (drift:) i ustawienia aplikacji.
#define DRIFT_LPF_TAU_S             0.05f   // [s] filtr przyspieszeń i ω
#define DRIFT_BIAS_TAU_S            1.0f    // [s] uśrednianie biasu w postoju
#define DRIFT_V_TAU_S               0.15f   // [s] filtr prędkości z promienia skrętu
#define DRIFT_VDOT_TAU_S            0.3f    // [s] filtr przyspieszenia stycznego
#define DRIFT_STEADY_YAW_ACC_DPS2   150.0f  // [dps/s] powyżej – przyspieszenie styczne nieznane
#define DRIFT_STEADY_S              0.25f   // [s] po wejściu w łuk – przyspieszenie styczne nieznane
#define DRIFT_MAX_DT_S              0.2f    // [s] ograniczenie kroku
#define DRIFT_MIN_YAW_DPS           30.0f   // [dps] poniżej – jazda prosto
#define DRIFT_MIN_ACC_G             0.3f    // [g] poniżej – kąt nieokreślony
#define DRIFT_MIN_V_MPS             1.5f    // [m/s]
#define DRIFT_V_MAX_MPS             30.0f   // [m/s]
#define DRIFT_MIN_ANGLE_DEG         15.0f   // [deg] kąt, od którego liczy się drift
#define DRIFT_SPIN_ANGLE_DEG        80.0f   // [deg] powyżej przez DRIFT_SPIN_S = obrót
#define DRIFT_SPIN_S                0.25f   // [s]
#define DRIFT_CHAIN_GAP_S           1.0f    // [s] przerwa, po której łańcuch jest zaliczany
#define DRIFT_MIN_SEG_S             0.3f    // [s] minimalny segment (przekładka)
#define DRIFT_PTS_PER_M             10.0f   // punkty za metr driftu
#define DRIFT_PTS_PER_M_DEG         1.0f    // punkty za metr × stopień kąta
#define DRIFT_MAX_MULT              10      // maks. mnożnik
#define DRIFT_SPIN_LOSES_CHAIN      1       // 1 = obrót kasuje łańcuch, 0 = zalicza

// ---------------------------------------------------------------- BLE
#define BLE_NAME            "X4F-LOG"
#define BLE_MTU             247
#define BLE_TX_POWER_DBM    8       // [dBm] maks. dla nRF52840 (zasięg w aucie)
// PHY 1M ma ok. 3 dB lepszą czułość niż 2M (zasięg na torze). 2M tylko na czas pobierania
// (GET/RGET), powrót do 1M przy LIVE 1. 0 = zawsze 1M.
#define BLE_PHY_2M_DOWNLOAD 1

// ---------------------------------------------------------------- format
#define RUN_FORMAT_VERSION  1
#define RACE_FORMAT_VERSION 1
#define PROTOCOL_VERSION    3       // 2: komendy wyścigów RLIST/RGET/RDEL/RERASE/RSTOP
                                    // 3: punkty driftu w LIVE, DRIFT RESET, MOUNT

// ---------------------------------------------------------------- debug
// Komendy SIM_* (syntetyczne zbocza/gaz) dostępne tylko przez USB.
#define ENABLE_SIM_COMMANDS 1
