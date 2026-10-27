# Protokół X4F-LOG (wersja 8)

Wersja 2 (fw 1.1.0) dodaje komendy wyścigów `RLIST`/`RGET`/`RDEL`/`RERASE`/`RSTOP` i pola
`race_*`. Wersja 3 (fw 1.2.0, drift) dodaje punkty za drift w `LIVE` (`d`), `DRIFT RESET`
i `MOUNT`. Wersja 4 (fw 1.3.0) dodaje skręt: `st` w `LIVE`, kontrę i znak skrętu w `d`,
znak skrętu w `MOUNT`, pola `steer_*` w `STATUS`; `LIVE` bez pola `zupt`. Wersja 5 (fw 1.4.0):
profil auta – nazwa w `MOUNT`, zapis na stałe w QSPI (`mount_saved`), po resecie ostatni profil.
Wersja 6 (fw 1.5.0): bonus za pełny gaz – `DRIFT BONUS 0|1` (część profilu), flaga `full` w `d`.
Wersja 7 (fw 1.6.0): `DRIFT BONUS 0|1 <próg %> <czas s>`, w odpowiedzi `full_pct`, `full_s`.
Wersja 8 (fw 1.7.0): auto-korekta odchyłki montażu – `trim` w `d` (ostatnie pole, [°]) i `yaw_trim` w `STATUS`;
fw 1.7.0 dopisywał korektę do zapisanego montażu (yaw w `MOUNT`), fw 1.7.1 już nie (korekta tylko do wyłączenia).
Wersja 9 (fw 1.8.0): `DRIFT DEC <puszczony gaz g> <hamulec g> <nauka 0|1>` – opóźnienie bokiem w drifcie (profil,
zapis na stałe); w `INFO` / odpowiedziach `MOUNT` i `DRIFT` pola `dec_coast`, `dec_brake`, `dec_auto`, w `STATUS`
`dec_coast_now`, `dec_brake_now` (w użyciu – nauczone). Nauka zapisuje nowe wartości w profilu poza nagrywaniem
(≥ 2 s nauki, zmiana ≥ 0,005 g). `RLIST [<od indeksu>]`: gdy lista nie mieści się w odpowiedzi, pole `next` = indeks
dalszej części, `count` = liczba wyścigów (wcześniej lista była ucinana bez znaku). Klient sprawdza `proto` z `INFO` – starszy firmware nie zna nowych komend.

Transport BLE: Nordic UART Service (`6e400001-…`), RX `6e400002` (zapis, telefon → Xiao),
TX `6e400003` (notyfikacje). Nazwa rozgłoszeniowa `X4F-LOG`. MTU do 247 (payload 244).
Te same komendy działają przez USB Serial (115200, CDC).

## Ramkowanie Xiao → host

* **BLE**: pierwszy bajt każdej notyfikacji to typ: `0x01` fragment tekstu (linie
  zakończone `\n`), `0x02` fragment ramki binarnej. Ramka binarna może zajmować wiele
  notyfikacji; jej długość wynika z nagłówka bloku.
* **USB**: tekst wysyłany wprost (linie `\n`); ramka binarna poprzedzona jednym bajtem
  `0x02`. Linie tekstu nigdy nie zaczynają się od `0x02`.

Ramka binarna (odpowiedź na `GET` i `RGET`):

```
u32 magic 0x42463458 ("X4FB") | u32 seq | u32 offset | u32 len | u32 crc32(dane) | dane[len]
```

## Komendy (tekst zakończony `\n`)

| Komenda | Odpowiedź |
|---|---|
| `INFO` | `{"fw","format","proto","hall_lines","pole_pairs","edges_per_rev","slot_size","slots","sample_rate","get_block_max","max_samples","max_edges","jedec","name","race_rate","race_area","race_sample_size","drift","steer","gyro_range","mount"}` – `mount`: `["+x","+y","+z",0.0]` (osie auta forward/left/up w osiach płytki, kąt w poziomie [°]) |
| `STATUS` | `{"state","neutral_us","neutral_learned","throttle_ok","throttle_us","runs","free_slots","overwritten_unread","time_set","ram_run","imu_ok","flash_ok","imu_overruns",…,"ble_dropped","hall_pins","armed"}` – `hall_pins`: bieżący stan pinów halla (bity A\|B<<1\|C<<2, diagnostyka okablowania), `armed`: postój zaliczony, wyzwolenie możliwe; wyścigi: `race_state`, `races`, `races_unread`, `race_used`, `race_size` [B], `race_rec_s`, `race_overflows` (próbki zgubione przez pełny bufor RAM), `race_scan_ms` (skan przy starcie); `ble_sup_ms` / `ble_interval_ms` (fw 1.5.1 – wynegocjowany czas nadzoru i interwał połączenia BLE); skręt (v4): `steer_us` (bieżący impuls, 0 = brak), `steer_neutral_us`, `steer_learned`, `steer_pulses`, `steer_rejects` (impulsy spoza 800–2200 µs – np. serwo w trybie wąskopasmowym), `steer_sign` |
| `TIME <unix_s>` | `OK` |
| `LIST` | `{"runs":[{"seq","len","crc","unix","boot","uptime_ms"[,"ram":true]}…]}` od najstarszego |
| `GET <seq> <offset> <len>` | ramka binarna; `len ≤ 4096`; przez BLE przełącza na PHY 2M (szybsze pobieranie) |
| `DEL <seq>` | `OK DEL <seq>` albo `{"err":"NOT_FOUND","seq":…}` |
| `ARM` | `OK` – powrót z ERROR do IDLE |
| `LIVE 1` / `LIVE 0` | `OK`; potem 20 Hz: `{"live":1,"v","v_max","v_front","slip","rpm","thr","thr_pct","ax","state","race","race_s","st","d":[…]}` – `race`: stan zapisu wyścigu, `race_s`: długość bieżącego wyścigu [s]; `st`: skręt [%] (+ = w lewo, gdy znak znany; inaczej znak surowy; 0 = brak sygnału); drift w tablicy `d`: `[pts, mult, total, last, best, ang, on, chain, banks, lost, trans, cs, steer_sign, full, trim]` = punkty bieżącego łańcucha (przed mnożnikiem), mnożnik, suma, ostatni zaliczony łańcuch (×mnożnik), najlepszy, kąt driftu [°] (0 poza łukiem), drift teraz (0/1), łańcuch trwa (0/1), zaliczone łańcuchy, obroty, przekładki, kontra (0/1), znak skrętu w użyciu (1/−1, 0 = nieznany), pełny gaz ≥ czas (0/1, v6 – bonus), bieżąca auto-korekta odchyłki montażu [°] (v8). Cała linia mieści się w jednej notyfikacji BLE (najgorszy przypadek ok. 241 B z `\n`). Wynik liczy Xiao – zgubione linie LIVE nic nie zmieniają. `LIVE 1` przez BLE przełącza na PHY 1M (zasięg) |
| `VMAX RESET` | `OK` |
| `DRIFT RESET` | `OK` – zerowanie punktów za drift (v3) |
| `DRIFT DEC <g> <g> 0\|1` | `{"mount":…,"dec_coast","dec_brake","dec_auto"}` (v9) – opóźnienie bokiem w drifcie na puszczonym gazie / hamulcu i nauka w jeździe; część profilu |
| `DRIFT BONUS 0\|1` | `{"mount":…,"drift_bonus":…}` (v6) – bonus za pełny gaz: gaz ≥ 95 % nieprzerwanie ≥ 1 s → punkty w drifcie ×1,5; zapisywany z profilem (zostaje po resecie) |
| `MOUNT [<forward> <left> <up> <yaw_deg> [<znak skrętu> [<nazwa>]]]` | `{"mount":[f, l, u, yaw, znak, "nazwa"],"mount_saved":…,"mount_save_us":…}` – bez argumentów odczyt; np. `MOUNT +y -x +z -18.5 -1 driftowóz` ustawia montaż płytki dla prędkościomierza i driftu (układ prawoskrętny, \|yaw\| ≤ 180), znak skrętu (v4: 1 normalny, −1 odwrócony, 0 = nauka w jeździe) i nazwę profilu auta (v5: bez spacji, bez `"` i `\`, < 24 B UTF-8; aplikacja zamienia spacje na `_`). Od v5 Xiao zapisuje profil na stałe (QSPI, rekord ~2 ms, tylko gdy nic się nie nagrywa) i po resecie wczytuje ostatni; ten sam profil wysłany ponownie niczego nie zapisuje. Zmiana montażu lub znaku skrętu zeruje estymator, filtry, punkty driftu i nauczony znak skrętu. Zła wartość: `{"err":"BAD_MOUNT"}` |
| `RLIST [od]` | `{"races":[{"seq","len","crc","unix","boot","uptime_ms","n","end","read"}…],"state","used","size","count"[,"next"]}` (v9: `next` – dalsza część listy od tego indeksu) – od najstarszego; `read`: oznaczony jako pobrany; `state`: IDLE / RECORDING / FINISHING / FULL / ERASING / DIRTY / ERROR |
| `RGET <seq> <offset> <len>` | ramka binarna jak `GET`; dane = plik wyścigu (`docs/FORMAT.md`) |
| `RDEL <seq>` | `OK RDEL <seq>` – oznacza wyścig jako pobrany (miejsce zwalnia dopiero `RERASE`) |
| `RERASE [FORCE]` | `OK` – kasowanie całego obszaru wyścigów w tle (< 0,3 s); `{"err":"UNREAD",…}` gdy są niepobrane wyścigi albo obszar `DIRTY` (wtedy tylko `FORCE`); `{"err":"BUSY"}` w trakcie nagrywania wyścigu |
| `RSTOP` | `OK` – zakończenie bieżącego wyścigu (testy) |

Tylko USB (testy na biurku): `SIM RPM <rpm>`, `SIM RAMP <rpm> <ms>`, `SIM THR <us>`,
`SIM TRIG`, `SIM GLITCH <us>`, `SIM OFF`, `SIM RESETAT <offset>` (reset w trakcie
zapisu – test zaniku zasilania), `SIM RACELIMIT <B>` (mniejszy obszar wyścigów, 0 = pełny),
`SIM RACERESETAT <n>` (reset po n próbkach wyścigu – test zaniku zasilania), `REBOOT`, `BOOT [UF2]`, `DBG`.

Błędy: `{"err":"…"}`. W trakcie RECORDING/SAVING `GET` i `DEL` zwracają `{"err":"BUSY"}`.

## Pobieranie

1. `TIME`, `STATUS`, `LIST`.
2. Dla każdego startu: `GET` kolejnych bloków (CRC bloku, do 3 ponowień), na końcu
   CRC całości (z polem `crc32` nagłówka liczonym jako 0) porównane z `crc` z `LIST`.
3. Zapis u odbiorcy (IndexedDB `oncomplete` / plik na dysku), dopiero potem `DEL`.
4. Rozłączenie niczego nie kasuje; pobieranie wznawia się od ostatniego poprawnego offsetu.

`seq 0` = start, który został tylko w RAM po błędzie zapisu flash (do kolejnego startu).

## Wyścigi

1. `RLIST`; dla każdego wyścigu z `read: false`: `RGET` blokami, CRC całości = CRC32 pliku bez
   ostatnich 4 B, porównane z `crc` z `RLIST` i polem `crc32` trailera.
2. Zapis u odbiorcy, potem `RDEL <seq>`.
3. Kasowanie pamięci wyścigów tylko w całości (`RERASE`) – aplikacja proponuje je, gdy wszystkie
   wyścigi są pobrane. Pełna pamięć = nagrywanie wyścigów wstrzymane do `RERASE`.
