# X4F Drift Logger

Projekt pochodny X4F Launch Loggera (`10_RC_speed_meter`): ten sam sprzęt i firmware
plus **punkty za drift na żywo** (jak w Need for Speed), wyścigi zapisywane w 20 Hz.

Logger startów samochodu RC na XIAO nRF52840 Sense.
Urządzenie tylko podsłuchuje czujnik halla silnika i sygnał gazu, nagrywa IMU
i zapisuje starty we flash. Pobieranie odbywa się przez BLE do aplikacji PWA na Androidzie.
Drugi tryb (fw 1.1): **ciągły zapis całych wyścigów** (od fw 1.2: 20 Hz) do osobnej części flash.
Fw 1.2–1.3 (drift): Xiao liczy **punkty za drift** z tych samych próbek 20 Hz – zakładka *Drift* w aplikacji;
fw 1.3: skręt (kontra), działanie bez czujnika halla, format wyścigu 2.
Specyfikacja: `CLAUDE.md`. Okablowanie: `docs/WIRING.md`. Formaty: `docs/FORMAT.md`, `docs/PROTOCOL.md`.

```
firmware/      szkic Arduino (kod w firmware/src), rdzeń Seeed nRF52 (non-mbed)
app/           PWA (czysty JS, uPlot lokalnie), testy: node --test app/test/
tools/         rc_launch (Python): download/parse/analyze/plot/compare, testy: pytest
test_vectors/  wspólne wektory testowe (generator: rc_launch.make_test_vectors)
hall_sim/      symulator halla/gazu na ESP32 (opcjonalny), kompilacja: arduino-cli compile -b esp32:esp32:esp32 hall_sim
docs/          format binarny i protokół
```

## Szybki start

```bash
# narzędzia (raz)
arduino-cli config add board_manager.additional_urls https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
arduino-cli core install Seeeduino:nrf52          # testowane: 1.1.13
arduino-cli lib install "Adafruit SPIFlash"       # testowane: 5.1.1
uv tool install adafruit-nrfutil                  # upload przez bootloader
uv venv .venv && uv pip install --python .venv -r <(echo bleak numpy scipy matplotlib pyyaml pyserial pytest)

# firmware (tryb HALL_LINES=1; tryb 3: dodać oba --build-property)
arduino-cli compile -b Seeeduino:nrf52:xiaonRF52840Sense -u -p /dev/ttyACM0 firmware
#   --build-property "compiler.cpp.extra_flags=-DHALL_LINES=3" --build-property "compiler.c.extra_flags=-DHALL_LINES=3"

# testy
cd tools && ../.venv/bin/python -m pytest -q          # analiza + estymator C vs Python
node --test app/test/                                 # analysis.js vs wektory
.venv/bin/python tools/fw_test.py                     # testy na płytce (USB, komendy SIM)

# pobieranie z PC
cd tools && ../.venv/bin/python -m rc_launch.download          # BLE, kasuje po zapisie
../.venv/bin/python -m rc_launch.download --usb --keep         # USB, bez kasowania (starty i wyścigi)
../.venv/bin/python -m rc_launch.download --usb --erase-races  # skasuj pamięć wyścigów (gdy wszystkie pobrane)
../.venv/bin/python -m rc_launch.analyze runs/race_*.bin       # metryki wyścigu
../.venv/bin/python -m rc_launch.plot runs/*.bin --range 1
../.venv/bin/python -m rc_launch.compare runs/*.bin
```

Jeśli płytka przestanie odpowiadać na USB, dwukrotne wciśnięcie RESET uruchamia
bootloader. Firmware wgrywa się wtedy normalnie.

## Aplikacja (PWA) – wdrożenie na GitHub Pages

Web Bluetooth wymaga HTTPS, a localhost jest traktowany jako bezpieczny. Workflow
`.github/workflows/pages.yml` publikuje katalog `app/` przy każdym pushu na `main`:
w ustawieniach repozytorium wybierz *Settings → Pages → Source: GitHub Actions*.
Na telefonie otwórz `https://<użytkownik>.github.io/<repo>/` w Chrome, a następnie
*menu → Dodaj do ekranu głównego*. Po pierwszym otwarciu aplikacja działa offline
dzięki service workerowi. Po każdej zmianie plików aplikacji podbij `CACHE` w `app/sw.js`.

Test lokalny: `python3 -m http.server 8765 --directory app`, potem
`http://localhost:8765/#demo`, czyli tryb demo z wirtualnym Xiao.

## Tryb wyścigu

Obok startów (500 Hz, 4 s, bufor pierścieniowy) Xiao nagrywa **całe wyścigi**:

* **Pamięć:** flash podzielony na pół – 8 slotów startów (1 MB) i obszar wyścigów (1 MB).
* **Próbka 20 Hz, 20 B** (`RACE_RATE_HZ`; fw 1.1: 50 Hz): średnia z 25 taktów 500 Hz – przyrost zboczy halla
  (dokładna droga), ostatni okres obrotu, średni gaz, średnie 6 osi IMU, czas w taktach 2 ms.
  400 B/s → **ok. 43 min jazdy** w 1 MB (50 Hz: 17 min – jedna stała w `config.h`).
* **Start sam:** gaz > 30 % (`RACE_START_PCT`), z 1 s nagrania sprzed gazu (postój = bias IMU).
* **Koniec:** koła stoją i gaz w neutrum (albo brak sygnału gazu – wyłączony odbiornik) przez
  30 s (`RACE_END_IDLE_S`). Krótsze czekanie na sędziego nie dzieli wyścigu.
* **Bez nadpisywania:** wyścigi dopisywane po kolei; pełna pamięć = koniec wyścigu i stop nagrywania
  (LED: żółty błysk co 2 s) do skasowania z aplikacji. Kasowanie tylko w całości i tylko gdy wszystkie
  wyścigi są pobrane (`RERASE`; `FORCE` – obszar z obcymi danymi lub świadome porzucenie).
* **Zanik zasilania** (odłączenie LiPo w trakcie jazdy – normalne zakończenie): przy starcie rekord
  domykany trailerem; giną tylko próbki z bufora RAM (≤ 0,3 s).
* **Zapis nieblokujący:** bufor RAM 8 kB (`RACE_BUF_BYTES`, 20 s), zapis stron tylko gdy flash wolny,
  także w czasie czekania na BLE. Pełny bufor → próbka gubiona, czas doliczany do następnej (flaga przerwy).
* Wyścigi i starty działają jednocześnie (start spod świateł nagrywa się dodatkowo w 500 Hz).
* **Aplikacja:** zakładka *Wyścigi* – lista, wykresy (gaz, prędkość kół, obroty, przyspieszenie
  wzdłużne i boczne, prędkość skrętu), metryki, CSV, tagi; „Pobierz wszystko” pobiera też wyścigi,
  karta *Pamięć wyścigów* pokazuje zajętość i kasowanie. Analiza: `analyze_race` / `analyzeRace`
  (wspólne wektory `test_vectors/race_*.bin`).
* **Okrążenia i mapa toru** (tor stały, jazda w kółko; `race_laps` / `raceLaps`): pozycja = droga
  z licznika zboczy × kierunek z całki żyroskopu (bias z postoju przed startem). Granica okrążenia:
  kurs od startu przekracza kolejne wielokrotności 360°. Okrążenia z drogą ±20 % od mediany
  (`race_lap_dist_tol` – buksowanie, uderzenie) pomijane. Tor = średnia dobrych okrążeń
  przepróbkowanych po drodze (200 punktów) i dopasowanych do wzorca obrotem i przesunięciem
  (3 iteracje). W aplikacji: mapa z kolorem prędkości, tabela okrążeń (czas, strata do najlepszego,
  droga, v maks.), dotknięcie okrążenia = to okrążenie na mapie i na wykresach.
  Salon 2026-10-24: 16 okrążeń, tor ok. 11,8 m, najlepsze 6,00 s; Python = JS z dokładnością 1e-14.
  Ograniczenie: buksowanie zawyża drogę (tor surowy „odpływa”), dlatego dopasowanie okrążeń.

## Punkty za drift (fw 1.3, aplikacja 1.5)

Zakładka **Drift**: u góry punkty bieżącego łańcucha, na środku mnożnik, na dole suma; pasek kąta,
wskaźnik **KONTRA** i skręt, najlepszy łańcuch, przekładki, obroty, „Zeruj punkty” (`DRIFT RESET`).
Wibracja i napis przy przekładce, zaliczeniu łańcucha (+punkty) i obrocie.

* **Liczy Xiao** (`firmware/src/drift.c`) z próbek wyścigu 20 Hz – zawsze, także przy pełnej pamięci
  wyścigów. LIVE tylko pokazuje wynik, więc przerwy w zasięgu BLE nie gubią punktów.
* **Kąt driftu z IMU:** przy jeździe po łuku przyspieszenie poziome jest prostopadłe do prędkości,
  więc jego kierunek w osiach auta daje kąt między osią auta a kierunkiem jazdy. Prędkość
  z promienia skrętu V = |a|/|ω| – działa **bez czujnika halla** (w drifcie koła i tak buksują).
  Przyspieszenie styczne (gaz/hamowanie w łuku) odejmowane, gdy łuk ustalony.
* **Skręt (fw 1.3, opcjonalny):** sygnał serwa na **D7** (osobny pin, fw 1.4.1 – wcześniej D2) przez
  **10 kΩ szeregowo przy odczepie**, bez rezystora do masy (jak gaz); masa wspólna. Pin nie koliduje
  z hallem (D1–D3), więc ta sama płytka działa w aucie on-road i w driftowozie bez przelutowania.
  **Kontra** (koła skręcone przeciwnie do obrotu auta) potwierdza drift: przy znanym skręcie drift
  poniżej 40° liczy się tylko z kontrą, za to próg przyspieszenia spada z 0,3 do 0,1 g (śliskie auto).
  Od 40° drift liczy się także bez kontry (fw 1.7.1; wcześniej 90°): kontra znika przy każdej przekładce
  (kierownica przechodzi przez środek) i to rwało łańcuch – jazda 27.10: 5 → 28 przekładek, drift 90 → 127 s.
  **Kierunek skrętu** (normalny / odwrócony serwo) – automatycznie: Xiao uczy się w jeździe (zakręt
  z przyczepnością = skręt w stronę zakrętu, głęboki drift = kontra; 3 s zgodnych dowodów), a
  kalibracja w aplikacji wyznacza go z jazdy bez driftu i wysyła do Xiao (`MOUNT … <znak>`).
  Można też wybrać ręcznie w ustawieniach. Zły znak = kontra wygląda jak skręt w zakręt i drift się
  nie liczy, dlatego automat ma pierwszeństwo nad zgadywaniem.
  Przy żyroskopie drift (gyro w aucie) sygnał podbierać **za żyroskopem** (przewód do serwa) – przed nim
  nie ma kontry, którą dodaje żyroskop. Po podłączeniu sprawdzić w `STATUS` `steer_us` i
  `steer_rejects` (serwo w trybie wąskopasmowym / 760 µs wypadnie poza 800–2200 µs).
* **Bez halla:** pin D1 zewrzeć do GND przez ~100 kΩ (wiszący pin = fałszywe zbocza). Postój
  wykrywany z IMU: gaz w neutrum i spokojne przyspieszenia/ω przez 1,5 s (`DRIFT_REST_*`); nagrania
  mają 3 s pre-triggera, żeby przeliczenie w aplikacji też złapało bias. **Prędkościomierz bez halla
  nie działa** (prędkość kół = 0 ściąga wskazanie do zera) – punkty za drift nie zależą od niego.
* **Punkty:** w drifcie `V·dt·(10 + 1·kąt°)` – długość i kąt; **przekładka** (drift w drugą stronę
  bez przerwy > 1 s) = mnożnik +1 (do ×10); przerwa > 1 s = łańcuch zaliczony: suma += punkty × mnożnik;
  **obrót** (kąt > 80° przez 0,25 s) = łańcuch stracony. Drift liczy się od 15°, V ≥ 1,5 m/s.
  Wszystkie stałe: `config.h` (`DRIFT_*`) = `config.yaml` (`drift:`) = ustawienia aplikacji.
* **Ten sam algorytm** w `drift.c` (float32, test na PC `tools/drift_host`), `rc_launch/drift.py`
  (referencja) i `app/js/drift.js`; wektory `test_vectors/drift_eights` (skręt normalny) i
  `drift_eights_rev` (odwrócony): łuk z przyczepnością (nauka znaku), ósemki 25–35° z kontrą,
  2 przekładki, obrót. Python = JS = C (kąt do 1e-4°), kąt w ustalonym drifcie w granicach 4°,
  znak skrętu nauczony w obu wariantach, zły ręczny znak blokuje punkty (test), bez skrętu działa
  jak fw 1.2, jazda z przyczepnością (`race_oval`) = 0 punktów. Porównania sumowanych czasów
  z marginesem 1 ms (Σdt we float32 i float64 różni się o ułamek próbki).
* **Montaż płytki jest kluczowy** – błąd kąta montażu = błąd kąta driftu 1:1. Po zamontowaniu Xiao
  w aucie: kilka spokojnych okrążeń **bez driftu**, pobranie, *Wyścig → Kalibracja montażu → Wyznacz
  → Zastosuj* (osie, kąt w poziomie i kierunek skrętu). Aplikacja ostrzega, gdy w nagraniu jest drift
  albo rozrzut jest duży. Montaż wysyłany do Xiao (`MOUNT`) po każdym połączeniu – Xiao pamięta go do
  wyłączenia zasilania (po włączeniu połączyć telefon przed jazdą). Zmiana montażu zeruje filtry,
  punkty i nauczony znak skrętu.
  Salon 2026-10-24 (X4F FWD): do przodu +y, w lewo −x, kąt −18,5° (rozrzut 24° – ciasno i wolno).
* **Profile aut (aplikacja 1.6, fw 1.4):** *Opcje → Profil auta* – każde auto ma własny komplet ustawień
  (montaż i osie IMU, kierunek skrętu, przełożenie, opona, punkty za drift). Wybrany profil aplikacja
  wysyła do Xiao (`MOUNT … <nazwa>`), a Xiao **zapisuje go na stałe** w ostatnim bloku 64 kB pamięci QSPI
  (rekord 64 B w ~2 ms, bez kasowania; obszar wyścigów zmniejszony do 960 kB) – po resecie i bez telefonu
  liczy z ostatnim profilem, dopóki nie wybierzesz innego. Z drukowaną obudową / gniazdem o stałej
  orientacji kalibracja montażu wystarczy raz na auto. (Wewnętrzny flash z LittleFS odrzucony – zapis
  750–940 ms blokował pętlę pomiaru.)
* **Bonus za pełny gaz (fw 1.5–1.6):** przełącznik, próg (np. 65 % – auto ograniczone w aparaturze) i czas
  w profilu auta (`DRIFT BONUS <0|1> <próg %> <czas s>`, zapisywane na Xiao) – pełny gaz dłużej niż czas =
  punkty driftu ×1,5 (`DRIFT_FULL_*`, ten sam algorytm w C/Python/JS, wektor `drift_eights` ma fazy pełnego gazu).
* **Dźwięk silnika jak w grach (aplikacja 2.0):** silnik gra cały czas (🔊 na zakładce Drift), obroty idą za
  gazem z LIVE (wolne ~900 → 7500 rpm przy progu pełnego gazu), na progu ogranicznik odcina zapłony (~14 Hz),
  po puszczeniu gazu strzały z wydechu i turbina: gwizd sprężarki z doładowaniem, syk zaworu upustowego
  z „ćwierkaniem”. AudioWorklet (`engine-worklet.js`) składa dźwięk z pojedynczych zapłonów rzędowej czwórki,
  nieregularne odstępy zapłonów (bez „chiptune”), od ~2500 rpm druga warstwa – tonalne wycie dolotu (3 harmoniczne)
  i pomruk wału; strzały z wydechu = synteza modalna (impuls pobudza 3 tłumione rezonanse rury ~95/310/780 Hz,
  rozstrajane przy każdym strzale, + krótki trzask); ogranicznik: cykle o zmiennej długości i spadek obrotów w przerwie,
  i pomruk wału, filtr wydechu otwiera się z obrotami; strzały mają osobny tor bez przesteru, filtr DC.
  Sprawdzone offline (`LimiterSound.render`, podgląd WAV): obroty rosną z gazem
  dla 0/30/60/100 % gazu, modulacja odcięcia 14,1 Hz, strzały do 90× głośniejsze od wolnych obrotów.
  Własne nagrania: `app/sounds/limiter.mp3` (zamiast syntezy na odcięciu) i `blowoff.mp3`.
* **Montaż – źródłem prawdy jest Xiao (aplikacja 2.1):** przy połączeniu aplikacja przejmuje montaż, kierunek
  skrętu i pełny gaz z Xiao do profilu o tej samej nazwie; wysyła je tylko po świadomej zmianie (kalibracja, Opcje,
  wybór / nazwa profilu – `mount_dirty`). Wcześniej profil z domyślnymi osiami nadpisywał poprawny montaż (dwie jazdy
  25–26.10 bez punktów; z poprawnym montażem jazda 26.10: ~26 000 pkt, 13 przekładek, 1 obrót).
  Aplikacja 2.3.2: przy połączeniu zawsze przejmuje montaż z Xiao (także przy innej nazwie profilu i gdy zapis na
  Xiao jest w toku – to tam nadpisała Xiao trzeci raz, jazda 27.10); osie i kąt wysyła tylko po kalibracji,
  ręcznej zmianie osi / kąta albo wyborze innego auta, i pyta o potwierdzenie, gdy osie na Xiao są inne.
* **Auto-korekta odchyłki montażu (fw 1.7):** na wprost
  z gazem / hamowaniem (|ω| < 8 °/s, koła prosto, |a| ≥ 0,06 g przez 0,3 s) przyspieszenie leży na osi auta – różnica
  kierunku koryguje odchyłkę (stała czasowa 3 s takiej jazdy) – do wyłączenia zasilania. fw 1.7.0 dopisywał ją do
  zapisanego montażu; fw 1.7.1 już nie (płytka jest przyklejona na stałe, a dopisywanie zmieniało montaż szumem).
  Niezależne sprawdzenie: symetria kątów driftu lewo/prawo i jazda na wprost dają zgodnie ~+16° (jazdy 26–27.10).
  Jazda w kółko bez prostych nie daje próbek – kilka przyspieszeń i hamowań na prostej na początku sesji wystarczy.
* **Gdzie płytka:** jak najbliżej środka ciężkości (w aucie do driftu przesunięty do tyłu – wyznaczyć
  wagą: odległość od tylnej osi = rozstaw × masa przodu / masa całości, albo balansując auto na pręcie),
  na osi symetrii, płasko i sztywno. Płytka poza środkiem ciężkości mierzy kąt w swoim miejscu (z przodu
  mniejszy, z tyłu większy): różnica ≈ ω·d / V – przy 115 °/s, d = 10 cm i 4 m/s ok. 3°; w przekładce
  (ε rzędu 20–40 rad/s²) chwilowo 0,2–0,4 g zakłóceń. Pianka montażowa OK,
  byle płytka się nie przesuwała (inaczej kalibracja nieaktualna).
* **Ograniczenia (do sprawdzenia na prawdziwym drifcie):** metoda zakłada w przybliżeniu ustaloną
  jazdę po łuku; progi i punkty da się stroić w aplikacji na nagraniach (przeliczenie wsteczne);
  Xiao na żywo używa stałych z `config.h`. Żyroskop ±1000 dps.

## Zasięg BLE

**Krótka utrata zasięgu nie zrywa połączenia (fw 1.5.1, aplikacja 1.8):** Xiao prosi o czas nadzoru
połączenia 30 s (`BLE_SUP_TIMEOUT_MS`; wcześniej domyślne 2 s) i interwał 7,5 ms – prośba ponawiana
2/4/6 s po połączeniu, bo ta z chwili połączenia bywa odrzucana (trwa zmiana PHY/MTU). Sprawdzone z
laptopem (BlueZ): 30 000 ms i 7,5 ms przyjęte dopiero przy ponowieniu (wcześniej po cichu zostawało
2 s i 30 ms). Wynegocjowane wartości w `STATUS`: `ble_sup_ms`, `ble_interval_ms` (telefon może
przyjąć mniej). Gdy połączenie jednak padnie, aplikacja **łączy się ponownie sama** z tym samym Xiao
(bez wybierania z listy, próby do skutku), wznawia LIVE, wysyła czas i profil; punkty liczy Xiao,
więc nic nie ginie. „Rozłącz” wyłącza ponowne łączenie.

TX już na maksimum nRF52840 (+8 dBm). Od fw 1.2 łącze pracuje na PHY 1M (ok. 3 dB lepsza czułość
niż 2M), a 2M włącza się tylko na czas pobierania. Gdy zasięg spada do ~3 m przy pracującym
wentylatorze, najpierw sprawdź przyczynę: wentylator odłączony vs. włączony (silnik stoi).
* **Zakłócenia od wentylatora** (szczotkowy silniczek iskrzy w paśmie 2,4 GHz): kondensator
  ceramiczny 100 nF wprost na zaciskach wentylatora, krótkie skręcone przewody, ferryt; Xiao jak
  najdalej od wentylatora i przewodów silnika/ESC.
* **Tłumienie:** karbonowe podwozie, pakiet LiPo i elektronika blokują 2,4 GHz. Xiao wyżej (pod
  karoserią, nad podwoziem), antena (koniec płytki z anteną – sprawdzić w dokumentacji Seeed) wolna,
  nie przy metalu ani karbonie; karoseria malowana farbą metaliczną też tłumi.
* XIAO nRF52840 nie ma złącza zewnętrznej anteny – większy zasięg = inna płytka z u.FL albo
  przekaźnik (ESP32 przy torze). BLE Coded PHY (long range) – do sprawdzenia, czy działa z Chrome
  na Androidzie.

## Synchronizacja między urządzeniami (GitHub)

Dane aplikacji leżą w IndexedDB danej przeglądarki. Wspólne wyniki na telefonie i komputerze
zapewnia synchronizacja przez **prywatne repozytorium** `migomat/x4f-dane` (`app/js/sync.js`):

* Układ: `runs/<id>.bin` + `runs/<id>.json`, `races/<id>.bin` + `races/<id>.json` – surowy plik
  (niezmienny) i metadane (tag, notatka, `updated_at`; `deleted: true` po usunięciu na urządzeniu).
* Jedna synchronizacja = najwyżej jeden commit (Git Data API: bloby → drzewo → commit → ref bez force;
  równoczesny commit innego urządzenia → ponowienie). Konflikt metadanych: nowszy `updated_at`.
* Usunięcie w aplikacji usuwa nagranie na pozostałych urządzeniach; surowy plik zostaje w repo (kopia).
* Token: fine-grained PAT ograniczony do `x4f-dane`, *Contents: read and write*; zapisywany tylko
  w IndexedDB urządzenia, nie trafia do eksportu JSON. Instrukcja w aplikacji (*Ustawienia*).
* Automatycznie przy otwarciu aplikacji i po pobraniu z Xiao (gdy jest internet), ręcznie przyciskiem.
* Python: `git clone git@github.com:migomat/x4f-dane.git` – pliki `.bin` czytają bezpośrednio
  `rc_launch.analyze` / `plot` / `compare`.
* Test: `app/test/sync.test.mjs` – dwa urządzenia przez atrapę API GitHub.

## Ustalenia sprzętowe (sprawdzone w źródłach i na płytce)

| Temat | Wniosek |
|---|---|
| `micros()` | `DWT->CYCCNT/64`, gdy DWT włączony, inaczej tick RTOS (1 ms). Nie używać: zawija się po ~67 s i stoi w uśpieniu CPU. Zamiast tego **TIMER4** 32-bit 1 MHz. |
| TIMERy | TIMER0 – SoftDevice; TIMER2 – rdzeń czyta `CC[0]` (wersja bootloadera) w `init()`; TIMER1/3/4 wolne. Wybrany TIMER4 (6 rejestrów CC: 3× hall, gaz, takt 500 Hz, „teraz”). |
| Zegar | Po włączeniu SoftDevice wymuszany HFXO (`sd_clock_hfclk_request`), bo HFINT ma błąd ±1–2 %. |
| Przechwytywanie zboczy | GPIOTE (kanał przydzielony przez `attachInterrupt`) → PPI (kanały 0–3, przez `sd_ppi_*`) → `TIMER4.CAPTURE`. Znacznik czasu bez jittera przerwań; ISR tylko odczytuje CC i poziom pinu. |
| IMU | LSM6DS3TR-C na **Wire1** (P0.07 SDA, P0.27 SCL), adres 0x6A, WHO_AM_I = 0x6A. Zasilanie z pinu P1.08 (`PIN_LSM6DS3TR_C_POWER`) – **wymaga trybu H0H1**. Biblioteka Seeed na rdzeniu non-mbed wybiera zły bus (`TARGET_SEEED_…` nie jest zdefiniowane) – konfiguracja rejestrami. |
| I2C | Biblioteka `Wire` robi powtórzony start programowo. Przy aktywnym SoftDevice ~1 % odczytów FIFO było przesuniętych o 1–3 słowa (zmierzone: 0 błędów bez BLE, ~9/s z BLE). Własny sterownik TWIM1 z `SHORTS` LASTTX→STARTRX→STOP: 0 błędów. |
| Flash | Puya **P25Q16H** (JEDEC 85 60 15), 2 MB, QSPI. Nie ma go na domyślnej liście `Adafruit_SPIFlash` z tym rdzeniem – deskryptor przekazywany jawnie. Na wszelki wypadek komenda 0xAB (wyjście z deep power-down) przy starcie. Kasowanie bloku 64 kB: **~17 ms**, komenda wraca po ~49 µs (nieblokujące, odpytywanie WIP). Zapis 1 kB ≈ 7 ms. Ponowne programowanie słowa (1→0) działa. |
| BLE | `BLEConnection::getMtu()` zwraca MTU zgłoszone przez klienta (517), a nie wynegocjowane – ograniczane do 247. Notyfikacje przez `sd_ble_gatts_hvx` (semafor w `BLECharacteristic::notify` ścigał się z rozłączeniem). |
| Stos | Zadanie `loop` ma tylko 4 KB (`LOOP_STACK_SZ`), a hook przepełnienia przy `CFG_DEBUG=0` nic nie robi. Duże bufory są statyczne, zapas stosu raportowany w `STATUS.stack_free` (zmierzone ≥ 2,3 KB). HardFault → reset zamiast zawieszenia. |
| LED | RGB aktywne niskim (mimo `LED_STATE_ON 1` w variant.h). |

## Pomiary (płytka bez auta, SIM przez USB, laptop BT 4.2)

| Wielkość | Wynik | Cel |
|---|---|---|
| IMU | 838 Hz, overrun = 0 przez 60 s, misaligned = 0; szum |a| 1,1 mg, ω 0,06–0,09 dps | 833 Hz, 0 |
| Czas SAVING – tryb 1 (1750 próbek + ~1500 zboczy, 55 kB) | 480–590 ms (CRC 18–20 ms) | < 1 s |
| Czas SAVING – tryb 3 (1750 próbek + ~4500 zboczy, 67 kB) | 756 ms (CRC 25 ms) | < 1 s |
| Pobieranie USB | 77 kB/s | – |
| Pobieranie BLE (bleak, BlueZ, BT 4.2) | 28 kB/s, MTU 247 | ≥ 20 kB/s |
| LIVE | 20 Hz | 20 Hz |
| RAM (statyczny) | tryb 1: 127 kB (fw 1.1: + bufor wyścigów 8 kB, próbki 2000 × 28 B) z 232 kB dostępnych dla aplikacji | – |
| Kasowanie obszaru wyścigów (1 MB) | < 0,3 s | – |
| Skan wyścigów przy starcie (z weryfikacją CRC i sprawdzeniem pustości) | 0,32–0,34 s | – |
| Pobieranie wyścigu USB | 65 kB/s | – |
| Wyścig SIM: 664 próbki, zanik zasilania po 333 → odtworzone 324, limit 30 kB → koniec „pełna pamięć”, kolejny nie startuje, `RERASE` bez pobrania odrzucony | ✅ | – |

Budżet RAM: próbki 1750 × 28 B = 49 kB; log zboczy wymiarowany z `MAX_MOTOR_RPM`
= 40 000 rpm × okno 3,5 s × 1,1: tryb 1 5133 × 4 B = 20,5 kB, tryb 3 15 400 × 4 B
= 61,6 kB; takty 256 × 16 B; SoftDevice S140 zajmuje RAM od 0x20000000 do 0x20006000.

## Status kryteriów odbioru

| # | Kryterium | Status |
|---|---|---|
| 1 | Szkielet, oba tryby kompilują się | ✅ (oba tryby wgrane i przetestowane na płytce) |
| 2 | IMU 833 Hz bez gubienia, \|a\| ≈ 1 g | ✅ (60 s, 0 overrun) |
| 3 | Gaz ±2 µs, uczenie neutrum | ✅ algorytm (SIM); ⏳ zgodność z symulatorem ESP32 – wymaga okablowania |
| 4 | Hall ±0,5 % 500–30 000 rpm, szpilki, powrót do 0 | ✅ algorytm w trybach 1 i 3, kierunek A→B→C (SIM); ⏳ tor sprzętowy GPIOTE/PPI – wymaga hall_sim/auta |
| 5 | Logger: pre-trigger, liczba próbek, LED | ✅ (1750 próbek, trigger 250, odstęp 2000 µs) |
| 6 | Flash: 20 startów → 16 + overwritten 4 (fw 1.1: 12 → 8 + 4); zanik zasilania; DEL (także DEL → natychmiastowy GET, seria pobierz+DEL); SAVING < 1 s | ✅ (zanik zasilania symulowany resetem w trakcie zapisu – `SIM RESETAT`; test z odłączeniem LiPo do wykonania) |
| 7 | LIST/GET/DEL przez USB i BLE, wznawianie, przepustowość | ✅ |
| 8 | Wektory testowe, testy analyze.py | ✅ 33 testy Pythona (w tym estymator C vs Python i wyścigi), 17 testów JS; płytka: 38/38 (`fw_test.py`, fw 1.0) |
| 9 | Aplikacja – demo, analysis.js przechodzi wektory | ✅ (pobieranie, DEL po zapisie, tagi, wykresy, porównanie, zerwanie połączenia) |
| 10 | Aplikacja z prawdziwym Xiao na Androidzie, offline | ⏳ wymaga telefonu i hostingu HTTPS |
| 11–13 | Testy z autem, kalibracja, prędkościomierz na torze | ⏳ wymagają auta |

## Odstępstwa i interpretacje specyfikacji

* **`HALL_TIMEOUT_US`**: w trybie 1 przy 50 ms nie da się zmierzyć 500 rpm (zbocze co 60 ms),
  dlatego tryb 1 ma 80 ms (min. 375 rpm), tryb 3 – 50 ms.
* **Wstępne kasowanie slotu**: gdy pamięć jest pełna, najstarszy slot kasowany jest w chwili
  *wyzwolenia* (w trakcie RECORDING, ~35 ms), a nie z wyprzedzeniem w IDLE – inaczej przy 16 slotach
  w pamięci zostawałoby 15 startów i test „20 → 16, overwritten = 4” byłby niemożliwy. SAVING nigdy
  nie czeka na kasowanie. Wolne sloty po starcie są w tle sprawdzane, czy są puste.
* **CRC**: jedno CRC32 (IEEE) na start – z polem `crc32` nagłówka liczonym jako 0; to samo w `LIST`,
  nagłówku slotu i nagłówku startu.
* **Własna usługa NUS** (te same UUID) zamiast `BLEUart` – każda notyfikacja musi zaczynać się bajtem typu.
* **Start analizy** `i0` = wcześniejsze z (próg gazu, pierwsze zbocze halla) – ESC rusza silnik
  poniżej progu wyzwolenia. `esc_delay_s` może być ujemne.
* **Pochylenie**: prędkość pochylenia = −ω wokół osi „left” (wynika z `imu_axes`); `a_long =
  G·(a_fwd − a_ref − sin Δpitch)`, gdzie `a_ref` (bias + składowa grawitacji w spoczynku) jest
  średnią z okna spoczynku.
* **Estymator** (`v_disp`): reguła „lub warunek przyspieszeń trwa > 100 ms” sprawia, że przy kołach
  w powietrzu kręcących się ze *stałą* prędkością wskazanie po ~0,1 s dochodzi do prędkości kół.
  Na `v_imu` i poślizg to nie wpływa. W trakcie buksowania (koła przyspieszają) wskazanie nie skacze.
* **Wyzwalanie (2026-10-19, po testach w salonie)**: z 32 nagrań ~18 było manewrowaniem (mały gaz,
  nawroty przód/tył). Teraz próg 60 % (`TRIGGER_THROTTLE_PCT`) i uzbrojenie tylko po postoju: gaz
  w neutrum (|gaz| < `ARM_NEUTRAL_PCT`) i brak zboczy halla przez `ARM_REST_MS` = 300 ms; próg trzeba
  przekroczyć w ciągu `LAUNCH_WINDOW_MS` = 700 ms od zejścia z postoju. `PRETRIGGER_MS` = 1000 ms,
  żeby wolniejsze wciśnięcie gazu i spoczynek zmieściły się w nagraniu (2000 próbek).
* **Drgania wentylatora ESC**: na postoju oś pionowa akcelerometru drga ±0,05 g (~180 Hz, aliasy ~18 Hz).
  Spoczynek w analizie liczony na sygnałach po filtrze `lowpass_hz`, próg 0,06 g; ZUPT na |a|, |ω|
  po filtrze IIR (`ZUPT_LPF_TAU_S` = 0,1 s), próg 0,06 g – fałszywy ZUPT w ruchu blokuje wymóg braku zboczy halla.
* **BLE nie blokuje pomiaru**: przy słabym zasięgu wysyłanie mogło zatrzymać pętlę główną do 1 s
  (zgubione próbki IMU, luki w nagraniu – widoczne w startach z salonu). `LIVE` jest porzucany przy
  pełnej kolejce (`ble_dropped` w STATUS), a w czasie czekania na bufor (GET) pętla dalej czyta IMU
  i takty. Moc nadawania +8 dBm (`BLE_TX_POWER_DBM`), PHY w `BLE_PHY_2M`.
* **Wgrywanie bez przycisku RESET**: „touch 1200” w rdzeniu (`enterSerialDfu()`) zapisuje
  `NRF_POWER->GPREGRET` bezpośrednio – przy włączonym SoftDevice zapis nie dociera do bootloadera
  i płytka wracała do aplikacji (wgrywanie wymagało podwójnego RESET). Firmware nadpisuje słabe
  `tud_cdc_line_coding_cb` i przy 1200 bd ustawia magię DFU przez `sd_power_gpregret_set`
  (tak jak `BLEDfu`) – `arduino-cli upload` działa bez przycisku. Zapasowo komenda USB `BOOT [UF2]`.
  STATUS: `boot_hf` = 1, gdy poprzedni reset był z HardFault.
* **Sloty startów: 8 zamiast 16** (fw 1.1) – druga połowa flash to obszar wyścigów. Test
  „20 startów → 16 + overwritten 4” brzmi teraz „12 → 8 + 4”.
* Komendy `SIM …`, `REBOOT`, `BOOT`, `DBG` – tylko USB, do testów na biurku.

## LED

| Stan | Kolor |
|---|---|
| uczenie neutrum | żółty miga |
| IDLE (uzbrojony) | zielony |
| zapis wyścigu | zielony z krótką przerwą co 1 s |
| w pamięci niepobrane starty / wyścigi | zielony + krótki niebieski błysk co 2 s |
| pamięć wyścigów pełna (albo wymaga przygotowania) | żółty błysk co 2 s |
| RECORDING (start) | czerwony |
| SAVING | fioletowy |
| połączony z telefonem | niebieski |
| błąd (IMU, flash, brak gazu, błąd zapisu wyścigu) | czerwony miga |

## Do zrobienia przy aucie

1. Zmierzyć `HALL_PULLUP_V` multimetrem i rozkład pinów kabla sensora; potwierdzić `MOTOR_POLE_PAIRS`.
2. `THROTTLE_FORWARD_IS_HIGHER` wg reverse w aparaturze.
3. Kryteria 3–4 z `hall_sim` (ESP32 bez dzielnika), 10 z telefonem, 11–13 z autem.
4. FDR i średnica opony w ustawieniach aplikacji / `tools/config.yaml`, `IMU_YAW_OFFSET_DEG`.
