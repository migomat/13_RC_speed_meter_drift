# Format binarny startu (wersja 1)

Wspólny dla flash, BLE i plików `.bin`. Little-endian. Źródło prawdy:
`firmware/src/run_format.h`; dekodery: `app/js/format.js`, `tools/rc_launch/parse.py`.
Zmiana formatu = podbicie `RUN_FORMAT_VERSION` we wszystkich trzech miejscach.

```
[RunHeader 128 B][Sample × n_samples (28 B)][EdgeWord × n_edges (4 B)]
```

## RunHeader (128 B)

| off | typ | pole | opis |
|---|---|---|---|
| 0 | u32 | magic | `0x52463458` („X4FR”) |
| 4 | u16 | format_version | 1 |
| 6 | u16 | header_size | 128 |
| 8 | u32 | seq | numer startu (rosnący; 0 = start tylko w RAM) |
| 12 | u32 | unix_s | czas wyzwolenia, 0 = nieznany |
| 16 | u32 | boot_count | numer rozruchu Xiao |
| 20 | u32 | uptime_ms | czas od włączenia w chwili wyzwolenia |
| 24 | u8 | hall_lines | 1 albo 3 |
| 25 | u8 | motor_pole_pairs | |
| 26 | u8 | edges_per_rev | 2·hall_lines·pole_pairs |
| 27 | u8 | throttle_fwd_higher | 1: „do przodu” = dłuższy impuls |
| 28 | u16 | sample_rate_hz | 500 |
| 30 | u16 | imu_odr_hz | 833 |
| 32 | u16 | accel_range_g | 4 |
| 34 | u16 | gyro_range_dps | 500 |
| 36 | f32 | accel_scale_g | g/LSB (0,122e-3) |
| 40 | f32 | gyro_scale_dps | dps/LSB (17,5e-3) |
| 44 | u16 | throttle_neutral_us | 0 = nienauczone |
| 46 | u16 | throttle_full_us | µs od neutrum do 100 % |
| 48 | u32 | trigger_index | indeks próbki wyzwolenia |
| 52 | u32 | n_samples | |
| 56 | u32 | n_edges | |
| 60 | u32 | flags | patrz niżej |
| 64 | u32 | t0_us | t_us pierwszej próbki (baza czasu zboczy) |
| 68 | u32 | trigger_t_us | t_us próbki wyzwolenia |
| 72 | u16 | sample_size | 28 |
| 74 | u16 | edge_size | 4 |
| 76 | u32 | fw_version | major<<16 \| minor<<8 \| patch |
| 80 | u16 | hall_min_edge_us | |
| 82 | u16 | pretrigger_ms | |
| 84 | u16 | posttrigger_ms | |
| 86 | u8 | trigger_pct | |
| 87 | u8 | hall_pullup_v_x10 | informacyjnie (konfiguracja) |
| 88 | u32 | hall_timeout_us | |
| 92 | 32 B | reserved | zera |
| 124 | u32 | crc32 | CRC32 IEEE całego startu z tym polem liczonym jako 0 |

Flagi startu: `0x01` zbocza obcięte (bufor pełny), `0x02` czas ustawiony,
`0x04` neutrum nauczone, `0x08` błąd IMU, `0x10` start tylko w RAM (błąd flash),
`0x20` dane syntetyczne (komendy SIM / wektory testowe).

## Sample (28 B, 500 Hz)

```c
uint32_t t_us;          // licznik 1 MHz (TIMER4), zawija się co ~71,6 min – odejmować jako uint32
uint32_t hall_edges;    // licznik zboczy (narastający)
uint32_t period_rev_us; // okres pełnego obrotu silnika, 0 = brak/postój/pierwsze EDGES_PER_REV zboczy
uint16_t throttle_us;   // 0 = brak sygnału (> 100 ms)
int16_t  ax, ay, az;    // surowe LSB, osie płytki
int16_t  gx, gy, gz;
uint8_t  flags;         // 0x01 hall_valid, 0x02 throttle_valid, 0x04 kierunek A→B→C,
                        // 0x08 kierunek znany, 0x10 przepełnienie FIFO IMU,
                        // 0x20 brak świeżej próbki IMU, 0x40 próbka wyzwolenia
uint8_t  reserved;
```

## EdgeWord (4 B)

Bity 0–28: czas zbocza względem `t0_us` w µs (modulo 2²⁹, okno nagrania ≪ 536 s),
bit 29: poziom linii po zboczu, bity 30–31: linia (0 = A, 1 = B, 2 = C).

## Nagłówek slotu we flash (tylko na urządzeniu)

Slot 128 kB; strona 0 = nagłówek, każde pole w osobnym słowie 32-bit
(programowane niezależnie, przejścia stanów tylko 1→0):
`+0 magic "X4FS"`, `+4 seq`, `+8 stan` (bajt niski: FF wolny, FE zapis, FC kompletny,
F8 pobrany), `+12 długość`, `+16 CRC` (= crc32 startu). Dane startu od offsetu 256.

## Układ flash (2 MB)

| Adres | Obszar |
|---|---|
| 0x000000–0x0FFFFF | 8 slotów startów po 128 kB (`FLASH_NUM_SLOTS`) – bufor pierścieniowy |
| 0x100000–0x1EFFFF | obszar wyścigów 960 kB (`RACE_AREA_*`) – zapis po kolei, kasowanie w całości (fw ≤ 1.3: 1 MB) |
| 0x1F0000–0x1FFFFF | profil auta 64 kB (`CFG_AREA_*`, fw 1.4) – rekordy 64 B dopisywane po kolei, obowiązuje ostatni poprawny |

## Rekord profilu auta (64 B, `mount_store.cpp`)

`u32 magic 0x4D463458 ("X4FM") | u16 wersja 1 | i8 idx[3] | i8 znak[3] | f32 yaw_deg | i8 znak skrętu |
char nazwa[24] (UTF-8, zero na końcu) | u8 flagi | u8 full_pct | u8 full_ds | u16 dec_coast [mg] | u16 dec_brake [mg] |
12 B zera | u32 crc32 (poprzednie 60 B)`. Flagi: bit 0 bonus za pełny gaz, 1 pola full_* ważne, 2 nauka opóźnień,
3 pola dec_* ważne (fw 1.8; starsze rekordy – wartości z `config.h`).
idx/znak: osie płytki dla forward/left/up (0=x, 1=y, 2=z; ±1). Zapis rekordu = programowanie bez kasowania;
po 1024 zmianach blok kasowany w tle i zapis od początku. Brak poprawnego rekordu = wartości z `config.h`.

# Format pliku wyścigu (wersja 2; wersja 1 nadal czytana)

Ciągły zapis całego wyścigu, `RACE_RATE_HZ` = 20 Hz (fw 1.2+; fw 1.1: 50 Hz – częstotliwość zawsze z nagłówka).
Wersja 1 (fw 1.1–1.2): próbka 20 B. **Wersja 2 (fw 1.3)**: próbka 22 B – na końcu dopisany skręt
(`steer_us`), w nagłówku neutrum skrętu. Dekodery czytają obie (rozmiar próbki z nagłówka).
Aplikacja sprzed wersji 1.5 nie czyta wersji 2. Źródło prawdy: `run_format.h`
(`RaceHeader`, `RaceSample`, `RaceTrailer`); dekodery: `format.js` (`parseRace`),
`parse.py` (`parse_race`). Zmiana = podbicie `RACE_FORMAT_VERSION` we wszystkich trzech.

```
[RaceHeader 128 B][RaceSample × n (20 B / 22 B)][wypełnienie – opcjonalne][RaceTrailer 16 B]
```

Trailer to zawsze ostatnie 16 B pliku. Wypełnienie pojawia się tylko w wyścigu
domkniętym po zaniku zasilania (trailer trafia na następną czystą stronę flash);
parser je pomija, CRC je obejmuje.

## RaceHeader (128 B)

Pola 0–47 jak w `RunHeader` (magic `0x4C463458` „X4FL”, `format_version` = 2 (1), seq –
wspólna numeracja ze startami, unix_s, boot_count, uptime_ms w chwili gazu, konfiguracja
halla, `sample_rate_hz` = 20 (50), zakresy i skale IMU, neutrum gazu). Dalej:

| off | typ | pole | opis |
|---|---|---|---|
| 48 | u32 | trigger_index | indeks próbki, w której wciśnięto gaz (przed nią `pretrigger_ms` postoju) |
| 52 | u16 | tick_rate_hz | 500 – jednostka `n_ticks` |
| 54 | u16 | decim | taktów na próbkę (25 przy 20 Hz, 10 przy 50 Hz) |
| 56 | u32 | flags | jak w starcie (czas ustawiony, neutrum, błąd IMU, dane syntetyczne) |
| 60 | u32 | t0_us | zegar Xiao na końcu pierwszej próbki |
| 64 | u16 | sample_size | 22 (wersja 1: 20) |
| 66 | u16 | trailer_size | 16 |
| 68 | u32 | fw_version | |
| 72 | u16 | period_unit_us | 4 |
| 74 | u16 | pretrigger_ms | 3000 (fw ≤ 1.2: 1000) |
| 76 | u16 | end_idle_s | 30 |
| 78 | u8 | start_pct | 30 |
| 79 | u8 | hall_pullup_v_x10 | |
| 80 | u32 | hall_timeout_us | |
| 84 | u16 | hall_min_edge_us | |
| 86 | u16 | steer_neutral_us | wersja 2: neutrum skrętu, 0 = nienauczone / brak (wersja 1: 0) |
| 88 | u16 | steer_full_us | wersja 2: neutrum → pełny skręt [µs] (500) |
| 90 | 38 B | reserved | zera |

## RaceSample (22 B; wersja 1: 20 B) – średnia z `decim` taktów 500 Hz

```c
uint16_t edges;        // przyrost licznika zboczy w okresie próbki (droga = Σ edges / EDGES_PER_REV · obwód/FDR)
uint16_t period_rev;   // ostatni okres obrotu silnika w jednostkach 4 µs, 0 = postój/nieznany
uint16_t throttle_us;  // średnia z poprawnych impulsów, 0 = brak sygnału
int16_t  ax, ay, az;   // średnie surowe LSB (uśrednienie = filtr antyaliasingowy)
int16_t  gx, gy, gz;
uint8_t  flags;        // bity 0–5 jak w Sample (OR z okresu), 0x40 przerwa (zgubione próbki), bit 7 zawsze 0
uint8_t  n_ticks;      // czas próbki w taktach 2 ms (zwykle 25; więcej po przerwie), 1..254
uint16_t steer_us;     // wersja 2: średnia szerokość impulsu skrętu, 0 = brak sygnału
```

Skręt [%] = (steer_us − steer_neutral_us) · 100 / steer_full_us, znak surowy – kierunek „w lewo”
rozpoznaje analiza driftu (nauka w jeździe albo kalibracja).

Czas próbki = Σ n_ticks / tick_rate_hz (koniec próbki), względem próbki `trigger_index`.
Bit 7 flag = 0 i `n_ticks` ≠ 0xFF odróżniają próbkę od skasowanego flash (odtwarzanie po zaniku zasilania).

## RaceTrailer (16 B)

`u32 magic 0x45463458 ("X4FE") | u32 n_samples | u8 end_reason | 3 B zera | u32 crc32`

`end_reason`: 0 postój `RACE_END_IDLE_S`, 1 koniec pamięci, 2 zanik zasilania (domknięty przy starcie),
3 komenda `RSTOP`. `crc32` = CRC32 IEEE wszystkich bajtów pliku przed tym polem (to samo w `RLIST`).

## Rekord wyścigu we flash (tylko na urządzeniu)

Rekordy od granic stron 256 B, jeden za drugim. Prefiks 32 B (słowa 32-bit, przejścia 1→0):
`+0 magic "X4FP"`, `+4 seq`, `+8 stan` (FF wolny, FE zapis, FC kompletny, F8 pobrany, 00 pusty),
`+12 długość pliku`, `+16 CRC`. Plik wyścigu od offsetu 32. Przy starcie: skan rekordów (weryfikacja
CRC), rekord FE → zliczenie poprawnych próbek, trailer na następnej czystej stronie, stan FC; reszta
obszaru musi być skasowana (inaczej stan `DIRTY` – wymaga `RERASE FORCE`).
