# Okablowanie X4F-LOG

Urządzenie tylko **podsłuchuje** sygnały – żadnego wyjścia do ESC. Xiao zasilane
z własnej LiPo 1S (pady BAT+/BAT−); z elektroniką auta łączy je tylko masa i sygnały.

## Piny XIAO nRF52840 Sense (`firmware/src/config.h`)

| Pin Xiao | nRF52840 | Sygnał | Uwagi |
|---|---|---|---|
| D0 | P0.02 | gaz z odbiornika (CH2) | 10 kΩ szeregowo przy odczepie |
| D1 | P0.03 | hall A | dzielnik (patrz niżej) |
| D2 | P0.28 | hall B | tylko `HALL_LINES = 3` |
| D3 | P0.29 | hall C | tylko `HALL_LINES = 3` |
| D7 | P1.12 | skręt (serwo; przy żyroskopie – przewód ZA żyroskopem) | 10 kΩ szeregowo przy odczepie; wersja drift (fw ≥ 1.4.1) |
| GND | – | masa | z przewodu ESC → odbiornik |
| BAT+ / BAT− | – | LiPo 1S 100–150 mAh | osobne zasilanie |

Na płytce (bez lutowania): IMU LSM6DS3TR-C (Wire1: P0.07 SDA, P0.27 SCL, zasilanie P1.08),
flash P25Q16H (QSPI), dioda RGB (P0.26 R, P0.30 G, P0.06 B, aktywne niskim).

## Ogólny schemat

```
        ODBIORNIK                         ESC                       SILNIK
   ┌──────────────┐  przewód gazu  ┌──────────────┐  kabel sensora ┌────────┐
   │ CH2  S ──────┼───────●────────┼─ S           │────●───────────│ hall A │
   │      + ──────┼────────────────┼─ +  (BEC)    │    │ (B, C)    │        │
   │      − ──────┼───────●────────┼─ −           │    │           └────────┘
   └──────────────┘       │        └──────────────┘    │
                          │ gaz        masa            │ hall A (B, C)
                     [10k]│            │          [47k]│  ← rezystor przy lucie
                          │            │               │
                    ┌─────┴────────────┴───────────────┴─────┐
                    │ D0               GND             D1(D2,D3)│  XIAO nRF52840 Sense
                    │                                [150k]→GND │  ← przy Xiao (linia 3,8 V)
                    │ BAT+ ── LiPo 1S ── BAT−                   │
                    └──────────────────────────────────────────┘
```

Masę i gaz bierzemy z przewodu ESC → odbiornik (masa odbiornika = masa ESC = masa
czujników). Z kabla sensora lutujemy tylko linie halla – w trybie 1 to **jeden lut**.

## Dzielnik dla każdej linii halla

`HALL_PULLUP_V` – napięcie stanu wysokiego linii halla przy włączonym ESC.

**Zmierzone (2026-10-19): 3,8 V** → dzielnik **47 kΩ + 150 kΩ** (niżej). Do potwierdzenia:
przy powolnym obracaniu kołem linia przełącza się 3,8 V ↔ ~0 V (stan niski < 0,5 V)
– inaczej mierzona żyła to zasilanie czujników, a nie linia halla.

**ESC z linią halla 3,8 V (ten ESC):**

```
hall X ──[47k]──────────── przewód ────────┬──► Dx (Xiao)
         (przy miejscu lutu,               [150k]
          w koszulce)                       │
                                           GND (Xiao)
```

Na pinie: 3,8 V · 150/197 ≈ **2,89 V** (próg „1” nRF52840 = 0,7 · 3,3 V ≈ 2,31 V,
maks. 3,6 V; przy 4,2 V na linii → 3,2 V). Dzielnik 47k/68k dałby tylko 2,25 V –
**poniżej progu**, nie używać. Zamiennik: 47k + 100k → 2,59 V (mniejszy zapas).

Gdyby inny ESC miał inne napięcie – wartości dla innych przypadków:

**ESC podciąga do 5 V:**

```
hall X ──[47k]──────────── przewód ────────┬──► Dx (Xiao)
         (przy miejscu lutu,               [68k]
          w koszulce)                       │
                                           GND (Xiao)
```

Stan wysoki na pinie ≈ 5 V · 68/(47+68) ≈ 3,0 V (próg „1” nRF52 ≈ 0,7 · 3,3 V = 2,3 V).
47 kΩ przy odczepie chroni ESC przed zwarciem/uszkodzeniem długiego przewodu.

**ESC podciąga do 3,3 V:** tylko 10 kΩ szeregowo przy odczepie, bez rezystora do masy.

**Gaz:** 10 kΩ szeregowo przy odczepie, bez rezystora do masy (sygnał odbiornika 3,3 V).

**Skręt (D7):** tak samo jak gaz – 10 kΩ szeregowo przy odczepie (z przewodu sygnałowego serwa),
bez rezystora do masy; masa wspólna z przewodu gazu. Rezystor chroni wyjście odbiornika/żyroskopu
przy zwarciu przewodu i ogranicza prąd przez diody zabezpieczające pinu, gdyby sygnał miał 5 V.
Po podłączeniu `STATUS`: `steer_us` ≈ 1500 i zmienia się przy kręceniu kierownicą,
`steer_rejects` = 0 (inaczej serwo pracuje w trybie spoza 800–2200 µs).

**Auto bez halla:** D1 zewrzeć do GND przez ~100 kΩ (albo zostawić 150 kΩ z dzielnika halla) –
wiszący pin łapie zakłócenia jako fałszywe zbocza.

## Lutowanie do kabla (bez wtyczek)

1. Skalpelem zdjąć 2–3 mm izolacji z jednej żyły (AWG 28–30), nie naruszając sąsiednich.
2. Owinąć cienkim przewodem (silikon AWG 30 / wire-wrap), cynować krótko, niską temperaturą.
3. Izolacja: Kapton albo żywica UV; rezystor przy odczepie w koszulce.
4. Odciążenie: kropla kleju na gorąco albo opaska.
5. Przewody halla z dala od faz silnika, najlepiej skręcone z przewodem masy.

Rozkład pinów kabla sensora zależy od producenta silnika/ESC – **do sprawdzenia**
(zwykle 6 żył: GND, hall C, hall B, hall A, temperatura, +5 V; zmierzyć przed lutowaniem).

## Symulator `hall_sim` (ESP32) – testy na biurku

Wyjścia ESP32 mają 3,3 V – podłączać **bez dzielnika 47k/68k** (przez dzielnik napięcie
spadłoby poniżej progu stanu wysokiego).

| ESP32 | Xiao |
|---|---|
| GPIO25 | D1 (hall A) |
| GPIO26 | D2 (hall B) |
| GPIO27 | D3 (hall C) |
| GPIO32 | D0 (gaz) |
| GND | GND |

## Konfiguracja do ustawienia po pomiarach (`firmware/src/config.h`)

| Parametr | Co ustawić |
|---|---|
| `HALL_LINES` | 1 (3 przewody) albo 3 (5 przewodów) |
| `MOTOR_POLE_PAIRS` | liczba par biegunów silnika (2-biegunowy = 1) |
| `HALL_PULLUP_V_X10` | 50 albo 33 – zmierzone napięcie (informacyjnie w nagłówku startu) |
| `THROTTLE_FORWARD_IS_HIGHER` | 1, jeśli gaz „do przodu” = dłuższy impuls (zależy od reverse w aparaturze) |
| `IMU_AXIS_*`, `IMU_SIGN_*`, `IMU_YAW_OFFSET_DEG` | orientacja płytki w aucie (te same wartości w ustawieniach aplikacji) |
