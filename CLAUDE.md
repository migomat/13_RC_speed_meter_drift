# X4F Launch Logger – faza 1

Logger danych do analizy buksowania przednich kół przy ruszaniu w samochodzie RC
Xray X4F (1/10 touring, **napęd na przód**, przedni dyferencjał praktycznie
zablokowany – oba przednie koła kręcą się z tą samą prędkością).

Faza 1 = **tylko pomiar i analiza**. Urządzenie wyłącznie *podsłuchuje* sygnały,
nigdy nie steruje ESC. Nie ma czujników na tylnych kołach.

## Cel

Dla każdego startu zarejestrować:

- prędkość obrotową silnika (z czujnika halla silnika) → prędkość przednich kół,
- sygnał gazu z odbiornika,
- przyspieszenia i prędkości kątowe z IMU (rzeczywisty ruch auta).

Starty zapisywać w **pamięci flash Xiao jako bufor pierścieniowy** (wiele startów,
najstarszy nadpisywany, kasowanie po udanym pobraniu). Dane pobierać przez BLE
do **aplikacji webowej (PWA) na telefonie z Androidem** – tam liczyć poślizg,
przyspieszenie, metryki i porównywać ustawienia ESC / aparatury (punch, krzywa
gazu, boost itd.). Bez komputera na torze.

Python jest narzędziem pomocniczym (analiza w domu) i **implementacją
referencyjną** algorytmów analizy.

---

## Sprzęt

- **MCU:** Seeed Studio XIAO nRF52840 **Sense** (IMU LSM6DS3TR-C i 2 MB QSPI
  flash na płytce).
- **Zasilanie:** osobna LiPo 1S (100–150 mAh) na padach BAT+/BAT− Xiao.
  Jedyne połączenie z elektroniką auta to masa + sygnały.
- **Silnik:** bezszczotkowy sensorowy, 2-biegunowy (1 para biegunów) – do
  potwierdzenia, parametr `MOTOR_POLE_PAIRS`.
- **ESC:** podciąga linie halla (otwarty kolektor) do napięcia `HALL_PULLUP_V`
  (5 V albo 3,3 V – **do zmierzenia multimetrem**, nie zakładać).
- **Telefon:** Android, Chrome (Web Bluetooth).

### Tryby czujnika halla (wybór w czasie kompilacji)

| Tryb | Przewody do Xiao | Zbocza / obrót silnika* | Uwagi |
|---|---|---|---|
| `HALL_LINES = 1` | hall A, gaz, GND = **3 przewody** | 2 | jeden lut w kablu sensora, minimum okablowania |
| `HALL_LINES = 3` | hall A/B/C, gaz, GND = **5 przewodów** | 6 | 3× lepsza rozdzielczość przy ruszaniu, możliwa detekcja kierunku |

\* dla `MOTOR_POLE_PAIRS = 1`. Ogólnie: `EDGES_PER_REV = 2 * HALL_LINES * MOTOR_POLE_PAIRS`.

### Piny (XIAO)

| Pin | Funkcja |
|---|---|
| D0 | sygnał gazu z odbiornika (CH2) |
| D1 | hall A |
| D2 | hall B (tylko tryb 3) |
| D3 | hall C (tylko tryb 3) |
| D7 | skręt z odbiornika / za żyroskopem (wersja drift, 10 kΩ szeregowo) |
| GND | masa wspólna z ESC/odbiornikiem |

Piny zdefiniować w `config.h`, nie na sztywno w kodzie.

### Okablowanie – podłączenie przez lutowanie do kabla (bez wtyczek)

Wpinamy się w istniejący kabel sensora i przewód gazu, bez przecinania żył.

**Masę i gaz bierzemy z przewodu ESC → odbiornik** (minus i sygnał). Masa
odbiornika = masa ESC = masa czujników, więc z kabla sensora lutujemy tylko
linie halla. W trybie 1 to **jeden lut w kablu sensora**.

Dzielnik dla każdej linii halla, gdy `HALL_PULLUP_V = 5`:

```
hall X ──[47k]──────────── przewód ────────┬──► Dx (Xiao)
         (rezystor przy                    [68k]
          miejscu lutu)                     │
                                           GND (Xiao)
```

- 47 kΩ lutujemy **bezpośrednio przy odczepie** (w koszulce). Dzięki temu
  zwarcie lub uszkodzenie długiego przewodu nie wpłynie na pracę ESC.
- 68 kΩ do masy montujemy przy Xiao. Stan wysoki na pinie ≈ 3,0 V.
- Gdy `HALL_PULLUP_V = 3.3`: tylko 10 kΩ szeregowo, bez rezystora do masy.

Gaz: 10 kΩ szeregowo przy odczepie, bez rezystora do masy.

Technika lutowania (żyły kabla sensora są cienkie, ok. AWG 28–30):

1. Skalpelem zdjąć ok. 2–3 mm izolacji z jednej żyły, nie naruszając sąsiednich.
2. Owinąć cienkim przewodem (silikon AWG 30 lub drut do wire-wrap), cynować krótko,
   niską temperaturą.
3. Izolacja: taśma Kapton albo płynna izolacja / żywica UV (koszulki nie da się
   nasunąć na środek kabla z wtykami po obu stronach). Rezystor 47 kΩ może
   wejść w koszulkę nasuniętą na odczep.
4. Odciążenie: przymocować odczep do kabla kropelką kleju na gorąco lub opaską.

Przewody halla prowadzić z dala od przewodów fazowych silnika, najlepiej
skręcone z przewodem masy.

> **Agent:** sekcja sprzętowa jest informacyjna. Nie zakładaj wartości
> `HALL_PULLUP_V` ani rozkładu pinów kabla sensora – zostaw je w konfiguracji
> z komentarzem „do zmierzenia”.

---

## Struktura repozytorium

```
firmware/
  platformio.ini        (lub szkic Arduino – patrz niżej)
  src/
    config.h            wszystkie parametry kompilacji
    main.cpp
    timebase.*          sprzętowy timer 1 MHz
    hall.*              przerwania halla, bufor zboczy, prędkość
    throttle.*          pomiar szerokości impulsu
    imu.*               konfiguracja i odczyt LSM6DS3TR-C
    logger.*            bufor RAM, wyzwalanie, stany
    flash_store.*       bufor pierścieniowy startów w QSPI flash
    ble_link.*          BLE: komendy, lista, pobieranie i kasowanie startów
    led.*               sygnalizacja stanów (RGB, aktywne niskim)
app/                    aplikacja PWA na telefon (Android + Chrome)
  index.html            UI + logika, wszystkie biblioteki lokalnie (bez CDN)
  js/
    transport.js        interfejs Transport + WebBluetoothTransport + DemoTransport
    protocol.js         ramki, komendy, pobieranie blokami, CRC
    format.js           dekodowanie formatu binarnego startu
    analysis.js         obliczenia – zgodne z tools/rc_launch/analyze.py
    storage.js          IndexedDB: starty, tagi, ustawienia
    charts.js           wykresy (uPlot)
    ui.js
  vendor/uPlot.*        lokalna kopia biblioteki
  manifest.webmanifest
  sw.js                 service worker – praca offline
  icons/
  test/                 testy analysis.js na wektorach testowych
tools/
  rc_launch/            pakiet Pythona (pomocniczy, implementacja referencyjna)
    download.py         BLE z PC (ten sam protokół) / import eksportu z telefonu
    parse.py            dekodowanie formatu binarnego
    analyze.py          obliczenia: prędkości, poślizg, metryki
    plot.py             wykresy pojedynczego startu
    compare.py          porównanie wielu startów / ustawień
    make_test_vectors.py  syntetyczne starty + oczekiwane metryki
  config.yaml           FDR, średnica opon, mapowanie osi IMU
  runs/
test_vectors/           wspólne wektory testowe (Python i aplikacja)
hall_sim/               (opcjonalnie) symulator na ESP32
README.md
```

---

## Firmware

### Środowisko

- Rdzeń **Seeed nRF52 Boards (non-mbed)**, bazujący na Adafruit nRF52 –
  potrzebny Bluefruit do BLE.
- IMU: można użyć `Seeed_Arduino_LSM6DS3`, ale ODR, zakresy i filtry ustawić
  bezpośrednio rejestrami, jeśli biblioteka tego nie udostępnia.
- Sprawdzić, czy IMU na Xiao Sense wymaga włączenia zasilania osobnym pinem
  (`PIN_LSM6DS3TR_C_POWER` lub podobny w wariancie płytki) i na której magistrali
  I2C oraz pod jakim adresem (zwykle 0x6A) jest.
- Flash: dostęp surowy (bez systemu plików) przez `Adafruit_SPIFlash`
  z transportem QSPI. Zweryfikować typ układu na płytce (prawdopodobnie P25Q16H),
  jego obsługę w bibliotece oraz czy po starcie nie jest w trybie deep power-down.

### `config.h` – parametry minimalne

```c
#define HALL_LINES          1      // 1 albo 3
#define MOTOR_POLE_PAIRS    1
#define EDGES_PER_REV       (2 * HALL_LINES * MOTOR_POLE_PAIRS)
#define HALL_MIN_EDGE_US    50     // odrzucanie zakłóceń (glitch filter)
#define HALL_TIMEOUT_US     50000  // brak zbocza → prędkość 0

#define PIN_THROTTLE        D0
#define PIN_HALL_A          D1
#define PIN_HALL_B          D2
#define PIN_HALL_C          D3

#define SAMPLE_RATE_HZ      500
#define PRETRIGGER_MS       500
#define POSTTRIGGER_MS      3000
#define TRIGGER_THROTTLE_PCT 20    // % zakresu do przodu od neutrum

#define IMU_ACCEL_RANGE_G   4
#define IMU_GYRO_RANGE_DPS  500
#define IMU_ODR_HZ          833

#define LOG_RAW_EDGES       1      // zapis surowych znaczników czasu zboczy

#define FLASH_SLOT_SIZE     (128 * 1024)  // wielokrotność bloku kasowania 64 kB
#define FLASH_NUM_SLOTS     16            // 16 × 128 kB = 2 MB
#define GET_BLOCK_MAX       4096          // maks. rozmiar bloku przy pobieraniu

#define BLE_NAME            "X4F-LOG"
```

`static_assert`: nagłówek + maksymalna liczba próbek + maksymalna liczba surowych
zboczy mieści się w `FLASH_SLOT_SIZE`. Jeśli zboczy przybędzie ponad limit bufora,
nagrywanie kontynuować bez nich i ustawić flagę „edges truncated”.

Kod nie może zakładać `HALL_LINES == 3`. W trybie 1 piny B i C nie są
konfigurowane ani używane.

### Podstawa czasu

- Jeden wspólny 32-bitowy licznik 1 MHz na sprzętowym TIMERze nRF52
  (nie TIMER0 – używa go SoftDevice). Przepełnienie co ok. 71 min – obsłużyć
  arytmetyką bez znaku.
- Sprawdzić rozdzielczość `micros()` w tym rdzeniu. Jeśli jest gorsza niż 1 µs,
  **nie** używać jej do pomiarów halla i gazu.
- Jeśli to proste do zrobienia: przechwytywanie zboczy przez GPIOTE + PPI →
  CAPTURE timera (zero jittera przerwań). Wersja z ISR jest akceptowalna na start.

### Hall – pomiar prędkości (wspólny algorytm dla obu trybów)

1. Przerwanie na **obu zboczach** każdej używanej linii.
2. W ISR: znacznik czasu, odrzucenie zbocza, jeśli od poprzedniego minęło
   mniej niż `HALL_MIN_EDGE_US`, zapis do bufora kołowego długości
   `EDGES_PER_REV + 1`, inkrementacja licznika zboczy (uint32).
3. Prędkość liczona przy **każdym** zboczu z okresu **pełnego obrotu**:
   `period_rev_us = t[n] − t[n − EDGES_PER_REV]`.
   - Tryb 1: porównujemy zbocze z tym samym zboczem (tej samej polaryzacji)
     obrót wcześniej, więc niesymetryczne wypełnienie sygnału nie wprowadza
     błędu, a pomiar odświeża się co pół obrotu.
   - Tryb 3: każde zbocze porównujemy z tym samym zboczem tej samej linii
     obrót wcześniej, więc nierówne rozmieszczenie czujników się znosi.
4. Jeśli od ostatniego zbocza minęło więcej niż `HALL_TIMEOUT_US` → prędkość 0.
5. Przez pierwsze `EDGES_PER_REV` zboczy po postoju prędkość nieznana (flaga).
6. Tryb 3 (opcjonalnie): kierunek obrotu z sekwencji stanów A/B/C (flaga).

ISR krótka: bez obliczeń zmiennoprzecinkowych i bez `Serial`.

### Gaz

- Pomiar szerokości impulsu (typowo 1000–2000 µs) przez przerwanie
  na zmianie stanu albo przez capture timera.
- Odrzucanie impulsów spoza 800–2200 µs.
- **Uczenie neutrum** po starcie: 2 s średniej, gdy gaz jest stabilny
  (rozrzut < 10 µs). Kierunek „do przodu” ustawiany w `config.h`
  (`THROTTLE_FORWARD_IS_HIGHER`), bo zależy od reverse w aparaturze.
- Brak impulsu przez ponad 100 ms → flaga „brak sygnału”.

### IMU

- Akcelerometr ±4 g, żyroskop ±500 dps (wersja drift: ±1000), ODR 833 Hz, filtry wbudowane LSM6DS3
  ustawione rozsądnie (anty-aliasing).
- Odczyt próbek z FIFO lub w pętli z kontrolą, żeby nie gubić próbek.
- Zapis surowych wartości int16 (konwersja na jednostki po stronie aplikacji,
  współczynniki w nagłówku startu).

### Próbka logu (500 Hz)

```c
struct __attribute__((packed)) Sample {
  uint32_t t_us;          // znacznik czasu próbki
  uint32_t hall_edges;    // licznik zboczy (narastający)
  uint32_t period_rev_us; // ostatni okres pełnego obrotu, 0 = brak/postój
  uint16_t throttle_us;   // szerokość impulsu gazu, 0 = brak sygnału
  int16_t  ax, ay, az;
  int16_t  gx, gy, gz;
  uint8_t  flags;         // bity: hall_valid, throttle_valid, dir, imu_overrun...
  uint8_t  reserved;
};
```

Opcjonalnie (`LOG_RAW_EDGES`): osobny bufor surowych znaczników czasu zboczy
(uint32 + bit linii) z okna nagrania. Pozwala w analizie policzyć prędkość
z pełną rozdzielczością. Zmieścić się w RAM razem z SoftDevice – policzyć
i zgłosić budżet pamięci w README.

### Format startu (wspólny dla flash, BLE i plików)

Jeden start = **nagłówek samoopisujący** + próbki + opcjonalnie surowe zbocza.
Nagłówek zawiera: magic, wersję formatu, `seq`, czas (unix, jeśli znany),
`boot_count`, `uptime_ms` w chwili wyzwolenia, `HALL_LINES`,
`MOTOR_POLE_PAIRS`, `EDGES_PER_REV`, `SAMPLE_RATE_HZ`, zakresy IMU
i współczynniki skalowania, neutrum gazu, indeks próbki wyzwolenia, liczbę
próbek, liczbę zboczy, flagi (np. edges truncated) i CRC32 całości.
Aplikacja i Python nie mogą zgadywać konfiguracji – czytają ją z nagłówka.

### Logger i stany

```
IDLE → (gaz > próg) → RECORDING → (POSTTRIGGER_MS) → SAVING → IDLE
```

- Nagrywanie odbywa się **wyłącznie w RAM**: w IDLE bufor kołowy stale
  nadpisuje ostatnie `PRETRIGGER_MS`, po wyzwoleniu dopisuje `POSTTRIGGER_MS`.
- SAVING: kopiowanie startu z RAM do flash (patrz niżej). Wyzwolenia w tym
  czasie ignorowane. Czas zapisu zmierzyć i podać w README (cel: < 1 s).
- Po zapisie automatyczny powrót do IDLE – **kolejny start bez żadnej
  interakcji**. Można zrobić serię startów i pobrać wszystkie naraz.
- Okno nagrania musi obejmować stan spoczynku przed startem – analiza liczy
  z niego bias IMU.
- Błąd zapisu flash: stan błędu, LED, start zostaje w RAM i jest dostępny
  do pobrania jako specjalny `seq` (do czasu kolejnego startu).
- Nagrywanie ma zawsze priorytet nad transmisją BLE: w trakcie RECORDING
  i SAVING pobieranie jest wstrzymywane (odpowiedź `BUSY`).

### Pamięć flash – bufor pierścieniowy startów

**Wymagania:** wiele startów w pamięci, **najstarszy nadpisywany**, gdy brak
miejsca; start **kasowany po udanym pobraniu** na telefon; odporność na zanik
zasilania w dowolnym momencie.

- Flash podzielony na `FLASH_NUM_SLOTS` stałych slotów po `FLASH_SLOT_SIZE`.
  Jeden slot = jeden start. Brak systemu plików.
- Na początku slotu nagłówek slotu: magic, `seq` (uint32, rosnący, nigdy
  0xFFFFFFFF), bajt stanu, długość danych, CRC32 danych.
- Stany kodowane tak, by przejścia wymagały tylko zmiany bitów 1→0
  (bez kasowania):
  - `0xFF` – wolny (skasowany),
  - `0xFE` – zapis rozpoczęty,
  - `0xFC` – kompletny,
  - `0xF8` – pobrany (czeka na skasowanie).
- Kolejność zapisu: stan `0xFE` + `seq` → dane → długość i CRC → stan `0xFC`.
  Zanik zasilania w trakcie = slot w stanie `0xFE` → przy następnym starcie
  traktowany jako uszkodzony i kasowany.
- **Przy starcie** skan nagłówków wszystkich slotów: odtworzenie listy startów,
  następny `seq` = najwyższy + 1, sloty w stanie `0xFE`, `0xF8` lub niespójne
  → do skasowania. Licznik `boot_count` przechowywać w nagłówkach startów
  (najwyższy znaleziony + 1), bez osobnego obszaru konfiguracji.
- **Wybór slotu do zapisu:** pierwszy wolny; jeśli brak – slot z najniższym
  `seq` (najstarszy), nawet jeśli nie został pobrany. Każde nadpisanie
  niepobranego startu zwiększa licznik `overwritten_unread`, raportowany
  w `STATUS` (aplikacja ostrzega użytkownika).
- **Wstępne kasowanie:** w IDLE zawsze utrzymywać co najmniej jeden skasowany
  slot gotowy do zapisu, żeby SAVING nie czekał na kasowanie. Kasowanie
  bloków 64 kB nieblokujące (sprawdzanie rejestru statusu w pętli głównej),
  nigdy w trakcie SAVING.
- **Kasowanie po pobraniu:** wyłącznie na komendę `DEL <seq>` od aplikacji.
  Xiao natychmiast zmienia stan slotu na `0xF8` (szybka zmiana bitów),
  a właściwe kasowanie wykonuje w tle.
- Zużycie flash nie jest problemem (ok. 100 tys. cykli na sektor), ale nie
  kasować bez potrzeby.

### Czas

- Xiao nie ma zegara czasu rzeczywistego. Aplikacja po połączeniu wysyła
  `TIME <unix_s>`. Xiao trzyma przesunięcie, dopóki ma zasilanie z LiPo.
- Start nagrany przed ustawieniem czasu ma `unix = 0` oraz `boot_count`
  i `uptime_ms` – aplikacja przelicza jego przybliżony czas, jeśli w tej samej
  sesji zasilania później ustawiono czas.

### Prędkościomierz (estymacja prędkości na Xiao)

Prędkość auta do wyświetlania na żywo, liczona na Xiao z akcelerometru
z korektami. **Sama całka z akcelerometru dryfuje** (błąd biasu 0,01 g daje
ok. 0,1 m/s błędu na każdą sekundę), więc estymator łączy trzy źródła:

1. **Predykcja z IMU** (przy każdej próbce IMU):
   `v += a_long · dt`, gdzie `a_long` jest skorygowane o bias i pochylenie
   (jak w analizie, punkty 3–4) oraz o kąt montażu płytki w poziomie
   (`IMU_YAW_OFFSET_DEG`, żeby przyspieszenie boczne w zakrętach nie
   przeciekało do wzdłużnego).
2. **Zerowanie na postoju (ZUPT):** brak zboczy halla dłużej niż
   `HALL_TIMEOUT_US` + mała wariancja żyroskopu i akcelerometru przez
   `ZUPT_MS` (np. 200 ms) → `v = 0`, aktualizacja biasu akcelerometru
   i odniesienia pochylenia (kierunek grawitacji).
3. **Korekta z kół przednich, tylko gdy nie buksują ani nie blokują:**
   gdy `|a_front − a_long| < SLIP_ACC_THR` i `|v_front − v| < SLIP_V_THR`
   lub warunek trwa dłużej niż 100 ms, ściąganie `v` do `v_front`
   z stałą czasową `WHEEL_CORR_TAU_S` (np. 0,5 s).
   - Buksowanie (`a_front − a_long` duże albo `v_front > v + margines`) →
     brak korekty, czysta całka z IMU.
   - Hamowanie z blokadą kół (gaz w strefie hamowania i `v_front < v − margines`)
     → brak korekty.
4. Ograniczenie `v ≥ 0` (jazda do tyłu pomijana w fazie 1).

Na Xiao trzymać też `v_max` od ostatniego zerowania (komenda `VMAX RESET`).

Estymator działa ciągle, niezależnie od stanu loggera, i nie może opóźniać
odczytu IMU ani nagrywania (obliczenia w pętli głównej, nie w ISR).

**Ta prędkość służy tylko do wyświetlania.** Do liczenia poślizgu zawsze używać
`v_imu` bez korekty z kół – w przeciwnym razie poślizg byłby liczony
częściowo z samego siebie i zostałby zaniżony.

Parametry w `config.h`: `IMU_YAW_OFFSET_DEG`, `ZUPT_MS`, `SLIP_ACC_THR`,
`SLIP_V_THR`, `WHEEL_CORR_TAU_S`.

### LED

| Stan | Kolor |
|---|---|
| uczenie neutrum | żółty miga |
| IDLE (uzbrojony) | zielony |
| IDLE, w pamięci są niepobrane starty | zielony + krótki niebieski błysk co 2 s |
| RECORDING | czerwony |
| SAVING | fioletowy |
| połączony z telefonem | niebieski |
| błąd (IMU, flash, brak gazu) | czerwony miga |

### BLE – protokół

Transport: Nordic UART Service (Bluefruit `BLEUart`), nazwa `BLE_NAME`.
Poprosić o MTU 247, PHY 2M i krótki interwał połączenia. Zmierzyć
i podać w README osiągniętą przepustowość (cel: ≥ 20 kB/s).

**Ramkowanie powiadomień (Xiao → telefon):** pierwszy bajt każdej
notyfikacji to typ: `0x01` – fragment odpowiedzi tekstowej (linie JSON
zakończone `\n`), `0x02` – fragment ramki binarnej.

**Komendy (telefon → Xiao)**, tekst zakończony `\n`:

| Komenda | Odpowiedź |
|---|---|
| `INFO` | JSON: wersja firmware, wersja formatu, `HALL_LINES`, `EDGES_PER_REV`, rozmiar slotu, liczba slotów |
| `STATUS` | JSON: stan, neutrum gazu, liczba startów, wolne sloty, `overwritten_unread`, czy czas ustawiony |
| `TIME <unix_s>` | `OK` |
| `LIST` | JSON: lista startów `{seq, len, crc, unix, boot, uptime_ms}`, od najstarszego |
| `GET <seq> <offset> <len>` | ramka binarna: nagłówek bloku (magic, seq, offset, len, CRC32 bloku) + dane; `len ≤ GET_BLOCK_MAX` |
| `DEL <seq>` | `OK DEL <seq>` albo `ERR` |
| `ARM` | `OK` – przywrócenie IDLE po błędzie |
| `LIVE 1` / `LIVE 0` | strumień JSON 20 Hz: `v` (m/s, z estymatora), `v_max`, `v_front`, flaga buksowania, rpm, gaz, ax, stan |
| `VMAX RESET` | `OK` – zerowanie prędkości maksymalnej |

Błędy jako JSON `{"err": "...", ...}`; w trakcie RECORDING/SAVING `{"err": "BUSY"}`.

**Pobieranie jest blokowe i wznawialne:** aplikacja prosi o kolejne bloki,
sprawdza CRC każdego bloku (do 3 ponowień), na końcu CRC całego startu.
Rozłączenie w trakcie niczego nie kasuje – aplikacja wznawia od ostatniego
poprawnego offsetu.

Te same komendy dostępne przez USB Serial (testy na biurku, Python).

---

## Aplikacja na telefon (`app/`) – główne narzędzie

PWA działająca w Chrome na Androidzie, przez Web Bluetooth. Hostowana przez
HTTPS (Web Bluetooth tego wymaga) – domyślnie GitHub Pages; instrukcja
wdrożenia w README. Po pierwszym otwarciu z internetem działa **offline**
(service worker, wszystkie zasoby lokalnie, żadnych CDN), instalowana
„na ekran główny”.

### Architektura

- **Warstwa transportu oddzielona od reszty:** interfejs
  `Transport { connect(), send(cmd), onFrame(cb), disconnect() }`.
  Implementacje: `WebBluetoothTransport` i `DemoTransport` (wirtualne Xiao
  generujące syntetyczne starty – do testów bez płytki). Nikt poza
  `transport.js` nie odwołuje się do `navigator.bluetooth`.
  Dzięki temu w razie problemów z Web Bluetooth wystarczy dopisać
  `CapacitorBleTransport` (patrz „Plan awaryjny”).
- `analysis.js` implementuje **dokładnie** te same algorytmy co
  `tools/rc_launch/analyze.py` i musi przejść wspólne wektory testowe
  (tolerancja metryk 1%).
- Surowe dane startu przechowywane w IndexedDB **w oryginalnej postaci
  binarnej**; wyniki analizy liczone przy wyświetlaniu (zmiana ustawień,
  np. średnicy opony, działa wstecz na wszystkich startach).
- Bez frameworków (czysty JS, moduły ES); wykresy: **uPlot**, kopia lokalna.

### Web Bluetooth na Androidzie

- `requestDevice` z filtrem po prefiksie nazwy i UUID usługi NUS, wywoływane
  z gestu użytkownika (przycisk).
- Ponowne łączenie bez wyboru z listy przez `navigator.bluetooth.getDevices()`,
  jeśli jest dostępne; w przeciwnym razie zwykły wybór urządzenia.
- Obsługa braku uprawnień (Bluetooth / „Urządzenia w pobliżu”, wyłączony BT)
  z czytelnym komunikatem po polsku.
- Wake Lock w trakcie pobierania, żeby ekran nie gasł.

### Przepływ „Pobierz wszystko”

1. `TIME`, `STATUS`, `LIST`.
2. Dla każdego startu od najstarszego: pobieranie blokami → weryfikacja CRC →
   dekodowanie nagłówka → zapis do IndexedDB.
3. **`DEL <seq>` wysyłany dopiero po zakończeniu transakcji IndexedDB**
   (`oncomplete`). Niepowodzenie na którymkolwiek etapie = brak `DEL`.
4. Pasek postępu, liczba startów, ostrzeżenie, jeśli `overwritten_unread > 0`.
5. Po pobraniu: szybkie przypisanie tagu całej serii (np. „punch 3”) –
   przyciski z ostatnio używanymi tagami; tag i notatkę można zmienić później.

### Widoki

- **Połączenie:** stan, przycisk pobierania, podgląd na żywo (`LIVE`),
  tryb demo.
- **Prędkościomierz:** duże cyfry z aktualną prędkością (km/h, przełączane
  na m/s), pod spodem prędkość maksymalna z przyciskiem zerowania
  (`VMAX RESET`) i wskaźnik buksowania (zielony/czerwony). Dane z `LIVE`.
  Brak danych dłużej niż 300 ms → wyraźny komunikat „brak sygnału” zamiast
  zamrożonej wartości (zasięg BLE przy aucie na torze bywa ograniczony).
  Ekran nie gaśnie (Wake Lock). W trybie demo animowany syntetyczny przejazd.
- **Starty:** lista z datą, tagiem, notatką i kluczowymi metrykami;
  filtrowanie po tagu; usuwanie.
- **Start:** zsynchronizowane wykresy na wspólnej osi czasu:
  gaz [%] | `v_front`, `v_imu` i `v_disp` | `a_front` i `a_long` | poślizg | pitch.
  Szybkie przyciski zakresu (0–0,5 s, 0–1 s, całość) i zaznaczanie zakresu
  palcem; metryki pod wykresami. Czytelne w poziomie i w pionie.
- **Porównanie:** nałożone `v_imu(t)` wybranych startów lub tagów, tabela
  średnich i rozrzutu metryk dla tagów, wykres punktowy przyspieszenie
  vs poślizg ze wszystkich startów.
- **Ustawienia:** FDR, średnica opony, mapowanie osi IMU, `lowpass_hz`,
  `v_min`; domyślne wartości jak w `config.yaml`.

### Eksport i import

- Eksport pojedynczego startu do CSV (policzone przebiegi) przez Web Share API
  lub pobranie pliku.
- Eksport/import całej bazy jako jeden plik JSON (surowe dane base64 + tagi,
  notatki, ustawienia) – kopia zapasowa i przeniesienie na PC do Pythona.

Interfejs po polsku, duże elementy dotykowe, tryb jasny i ciemny.

### Plan awaryjny (jeśli Web Bluetooth sprawia problemy)

Opakować tę samą aplikację w **Capacitor** jako natywny APK na Androida
z wtyczką `@capacitor-community/bluetooth-le`. Zmienia się tylko warstwa
transportu (`CapacitorBleTransport`); UI, analiza i przechowywanie zostają
bez zmian. Nie implementować teraz – tylko nie blokować tej drogi.

---

## Narzędzia w Pythonie (`tools/rc_launch`) – pomocnicze

Zależności: `bleak`, `numpy`, `scipy`, `matplotlib`, `pyyaml`.

### `config.yaml`

```yaml
final_drive_ratio: 5.0       # do uzupełnienia
front_tire_diameter_mm: 64.0 # do uzupełnienia, kalibrowane
imu_axes:                    # mapowanie osi płytki na osie auta
  forward: "+x"
  left: "+y"
  up: "+z"
pitch_axis_gyro: "y"
lowpass_hz: 25
v_min_mps: 0.3
```

### `download.py`

- BLE z PC tym samym protokołem (`LIST` / `GET` / `DEL`). Domyślnie kasuje
  start po zapisaniu i weryfikacji, flaga `--keep` wyłącza kasowanie.
- Import pliku eksportu z aplikacji do `runs/`.
- Zapis do `runs/RRRRMMDD_HHMMSS_<tag>.bin` oraz `.json` z metadanymi.
- Tryb `--live` dla komendy `LIVE`.

### `analyze.py` – obliczenia (specyfikacja wspólna z `analysis.js`)

1. Konwersja surowych wartości na jednostki z użyciem nagłówka.
2. Mapowanie osi IMU na osie auta wg konfiguracji.
3. **Bias:** średnie z okna sprzed wyzwolenia, ale tylko jeśli auto stało
   (mała wariancja żyroskopu i akcelerometru). W przeciwnym razie ostrzeżenie.
4. **Pochylenie:** całka z żyroskopu osi poprzecznej od chwili spoczynku.
   Przyspieszenie wzdłużne skorygowane o składową grawitacji
   (`a_long = a_forward − g·sin(pitch)`, konwencję znaków zweryfikować).
5. Filtr dolnoprzepustowy zero-fazowy (Butterworth 2. rzędu przepuszczony
   w przód i w tył, `lowpass_hz`). W JS ta sama implementacja biquada,
   z tą samą obsługą warunków brzegowych – inaczej wyniki się rozjadą.
6. `v_imu` = całka z `a_long` od momentu startu (pierwsze zbocze halla
   albo przekroczenie progu gazu – oba zapisać).
7. **Prędkość przodu:**
   `v_front = (1e6 / period_rev_us) / FDR · π · D` [m/s].
   Jeśli są surowe zbocza – liczyć z nich (każde zbocze vs zbocze obrót
   wcześniej), a następnie przeliczyć na siatkę 500 Hz. Między zboczami
   przy niskiej prędkości użyć ograniczenia górnego:
   `v ≤ (1 obrót) / (czas od ostatniego zbocza)`.
8. `a_front` = pochodna `v_front` (wygładzona).
9. **Poślizg:** `s = (v_front − v_imu) / max(v_front, v_min)`.
10. **Wskaźnik buksowania bez całkowania:** `a_front − a_long`
    (nie dryfuje – główny wskaźnik w fazie 1).
11. Droga: z licznika zboczy (przód) i z podwójnej całki IMU.
12. **Prędkość „wyświetlana”** `v_disp`: ten sam estymator co prędkościomierz
    na Xiao (predykcja z IMU, ZUPT, korekta z kół bez buksowania), te same
    parametry. Tylko do prezentacji i metryki prędkości maksymalnej –
    **nie** do poślizgu.

### Metryki startu

- średnie `a_long` w 0–300 ms i 0–1000 ms,
- `v_imu` po 0,5 s i 1,0 s,
- prędkość maksymalna w nagraniu (z `v_disp`),
- czas do przejechania 1 m i 2 m (z IMU),
- szczytowy poślizg i czas jego wystąpienia,
- całka nadwyżki `a_front − a_long` (ile „przebuksowano”),
- czas od przekroczenia progu gazu do pierwszego zbocza halla (opóźnienie ESC).

### `plot.py`, `compare.py`

Odpowiedniki widoków „Start” i „Porównanie” z aplikacji, zapis do PNG.

### `make_test_vectors.py`

Generuje do `test_vectors/` syntetyczne starty w formacie binarnym (oba tryby
`HALL_LINES`, z buksowaniem i bez, z szumem IMU) oraz pliki z oczekiwanymi
metrykami. Używane przez testy Pythona, testy `app/test/` i `DemoTransport`.

---

## Symulator (opcjonalny, `hall_sim/`)

Program na ESP32 do testów bez auta:

- generuje sygnały halla (1 lub 3 linie, przesunięcie 120° el.) z rampą
  obrotów 0 → 30 000 rpm,
- generuje sygnał PWM gazu 50–300 Hz,
- komendy przez Serial: zadana rampa, skok gazu, zakłócenia (pojedyncze szpilki).

Uwaga: ESP32 daje sygnał 3,3 V. Podczas testów z symulatorem podłączać go
do Xiao **bez dzielnika 47k/68k** (przez dzielnik napięcie spadłoby poniżej
progu stanu wysokiego).

---

## Kolejność realizacji i kryteria odbioru

1. **Szkielet:** miganie LED, USB Serial, `INFO`. Kompiluje się w obu trybach
   `HALL_LINES`.
2. **IMU:** stabilne 833 Hz bez gubienia próbek (licznik overrun = 0
   przez 60 s). Auto w spoczynku: |a| ≈ 1 g.
3. **Gaz:** zgodność pomiaru z symulatorem ±2 µs; uczenie neutrum działa.
4. **Hall:** zgodność rpm z symulatorem ±0,5% w zakresie 500–30 000 rpm
   w obu trybach; szpilki krótsze niż `HALL_MIN_EDGE_US` odrzucane; prędkość
   wraca do 0 po zatrzymaniu.
5. **Logger:** wyzwolenie z pre-triggerem, poprawna liczba próbek, stany LED.
6. **Flash:**
   - 20 startów pod rząd przy 16 slotach → w pamięci 16 najnowszych,
     `overwritten_unread = 4`;
   - zanik zasilania (odłączenie LiPo) w trakcie SAVING → po ponownym
     uruchomieniu brak uszkodzonych startów na liście, pozostałe nietknięte;
   - `DEL` → slot zwolniony, po restarcie start nie wraca;
   - czas SAVING zmierzony i < 1 s.
7. **BLE:** `LIST` / `GET` / `DEL` z USB Serial i przez BLE; przerwane pobieranie
   wznawiane od offsetu; przepustowość zmierzona.
8. **Wektory testowe:** `make_test_vectors.py`, testy `analyze.py`.
9. **Aplikacja – tryb demo:** pełny przepływ (pobieranie, `DEL`, tagi,
   wykresy, porównanie) na `DemoTransport`; `analysis.js` przechodzi
   wektory testowe.
10. **Aplikacja – prawdziwe Xiao:** połączenie z Chrome na Androidzie,
    pobranie serii startów, start kasowany z Xiao tylko po zapisaniu w telefonie
    (test: wyłączenie BT w trakcie pobierania → start nadal na Xiao).
    Działanie offline po instalacji.
11. **Test na stole z autem:** podniesiony przód, gaz → `v_front` rośnie,
    `v_imu` ≈ 0, poślizg ≈ 100%.
12. **Kalibracja:** powolne pchanie auta ręką i łagodne starty bez buksowania →
    `v_front` ≈ `v_imu`; korekta średnicy opony w ustawieniach aplikacji.
13. **Prędkościomierz:**
    - auto stoi 60 s → wskazanie 0 (ZUPT działa, brak dryfu);
    - spokojna jazda bez buksowania → wskazanie zgodne z `v_front` ±3%;
    - start z buksowaniem → wskazanie nie skacze razem z kołami
      (porównać z `v_imu` z nagrania);
    - jazda po łuku → brak wyraźnego wzrostu wskazania od przyspieszenia
      bocznego (sprawdzenie `IMU_YAW_OFFSET_DEG`);
    - estymator w Pythonie i w JS daje te same wyniki na wektorach testowych.

## Tryb wyścigu (faza 1b, fw 1.1)

Drugi tryb pracy obok startów: **ciągły zapis całego wyścigu** (np. 6 min) do drugiej
połowy flash. Szczegóły formatu: `docs/FORMAT.md`, komendy: `docs/PROTOCOL.md`, opis: `README.md`.

- Flash: 8 slotów startów (1 MB) + obszar wyścigów 960 kB (`RACE_AREA_*`) + profil auta 64 kB (`CFG_AREA_*`).
- Próbka `RACE_RATE_HZ` = 20 Hz (fw 1.2; fw 1.1: 50 Hz) (20 B, średnia z taktów 500 Hz). Wyścig zaczyna się sam
  (gaz > `RACE_START_PCT`, 1 s pre-trigger), kończy po `RACE_END_IDLE_S` postoju.
- **Bez nadpisywania:** pełna pamięć = stop nagrywania wyścigów do skasowania z aplikacji.
  Kasowanie tylko w całości (`RERASE`) i tylko gdy wszystkie wyścigi pobrane (`RDEL` po zapisie
  u odbiorcy); `RERASE FORCE` – tylko świadomie (obce dane w obszarze).
- Zanik zasilania w trakcie wyścigu to normalne zakończenie – rekord domykany przy starcie.
- Analiza: `analyze_race` (Python) = `analyzeRace` (JS), wspólne wektory `test_vectors/race_*`.
- Okrążenia i mapa toru: `race_laps` / `raceLaps` – granice co 360° kursu z żyroskopu, pozycja
  z drogi kół, tor = średnia dopasowanych okrążeń (opis w README). Wektory zawierają okrążenia i tor.

## Punkty za drift (fw 1.3, aplikacja 1.5 – projekt 13_RC_speed_meter_drift)

Zakładka *Drift*: punkty łańcucha (góra), mnożnik (środek), suma (dół). Liczy Xiao (`drift.c`)
z próbek wyścigu 20 Hz i wysyła w `LIVE` (tablica `d`); aplikacja tylko wyświetla. Opis: README.

- Kąt driftu z kierunku przyspieszenia poziomego w łuku, prędkość z promienia V = |a|/|ω| –
  **nie z halla** (koła napędzane w drifcie buksują).
- Ten sam algorytm w `firmware/src/drift.c`, `tools/rc_launch/drift.py` (referencja) i
  `app/js/drift.js`; zmiana = wszystkie trzy + wektory `test_vectors/drift_*` (Python, JS
  i test C `tools/drift_host`). Stałe: `DRIFT_*` = `config.yaml: drift` = ustawienia aplikacji.
- Montaż płytki (osie + kąt w poziomie) decyduje o kącie driftu: kalibracja z nagrania jazdy bez
  driftu (`mount_calibrate` / `mountCalibrate`), wysyłana do Xiao komendą `MOUNT` (protokół 3).
- Skręt (fw 1.3, pin D7 – osobny od halla, oba tryby `HALL_LINES`): kontra potwierdza drift; kierunek skrętu automatycznie
  (nauka w drift.c, kalibracja) albo ręcznie – nigdy nie zgadywać na stałe. Format wyścigu 2 (22 B).
- Auto do driftu może nie mieć halla: nic w drifcie nie może wymagać zboczy (postój z IMU + gaz).
- Profile aut (aplikacja 1.6): aktywny profil → `MOUNT … <nazwa>`; Xiao zapisuje go na stałe w `CFG_AREA`
  (ostatni blok QSPI, rekordy 64 B, `mount_store.cpp`) – zapis tylko poza nagrywaniem i bez blokowania.

## Synchronizacja (aplikacja 1.3)

Wspólne dane telefonu i komputera przez prywatne repo GitHub (`app/js/sync.js`, opis w README).
Surowe pliki w repo są niezmienne; usunięcie w aplikacji = znacznik `deleted` w metadanych.
Token tylko na urządzeniu – nigdy w eksporcie ani w kodzie.

## Zasady dla agenta

- Nie zmieniać założenia, że urządzenie tylko słucha – żadnego wyjścia do ESC.
- Prędkość z estymatora (`v`, `v_disp`) nigdy nie wchodzi do obliczeń poślizgu
  ani wskaźnika buksowania – tam tylko `v_imu` bez korekty z kół.
- **Start może zostać skasowany z Xiao tylko po potwierdzonym zapisie
  po stronie odbiorcy** (`DEL` po weryfikacji CRC i zapisie). Jedyny inny
  wyjątek to nadpisanie najstarszego przy braku miejsca.
- Wszystkie stałe w `config.h` / `config.yaml` / ustawieniach aplikacji,
  z jednostkami w nazwach lub komentarzach.
- Brak alokacji dynamicznej w firmware; bufory statyczne z policzonym rozmiarem.
- Brak `delay()` w pętli głównej poza inicjalizacją; operacje flash
  nieblokujące.
- Każda zmiana formatu binarnego lub protokołu = podbicie wersji
  i obsługa w `format.js`, `protocol.js` i `parse.py` jednocześnie.
- Zmiana algorytmu analizy = zmiana w `analyze.py` i `analysis.js` jednocześnie
  oraz przejście wektorów testowych w obu.
- Niepewne szczegóły sprzętowe (piny IMU, zasilanie IMU, typ i tryb flash,
  rozdzielczość `micros()`, wolne TIMERy przy SoftDevice) sprawdzić
  w źródłach rdzenia i bibliotek, nie zgadywać, i zapisać wnioski w README.
