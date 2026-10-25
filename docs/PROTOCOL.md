# Protokół X4F-LOG (wersja 3)

Wersja 2 (fw 1.1.0) dodaje komendy wyścigów `RLIST`/`RGET`/`RDEL`/`RERASE`/`RSTOP` i pola
`race_*`. Wersja 3 (fw 1.2.0, drift) dodaje punkty za drift w `LIVE` (`d`), `DRIFT RESET`
i `MOUNT`. Klient sprawdza `proto` z `INFO` – starszy firmware nie zna nowych komend.

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
| `INFO` | `{"fw","format","proto","hall_lines","pole_pairs","edges_per_rev","slot_size","slots","sample_rate","get_block_max","max_samples","max_edges","jedec","name","race_rate","race_area","race_sample_size","drift","gyro_range","mount"}` – `mount`: `["+x","+y","+z",0.0]` (osie auta forward/left/up w osiach płytki, kąt w poziomie [°]) |
| `STATUS` | `{"state","neutral_us","neutral_learned","throttle_ok","throttle_us","runs","free_slots","overwritten_unread","time_set","ram_run","imu_ok","flash_ok","imu_overruns",…,"ble_dropped","hall_pins","armed"}` – `hall_pins`: bieżący stan pinów halla (bity A\|B<<1\|C<<2, diagnostyka okablowania), `armed`: postój zaliczony, wyzwolenie możliwe; wyścigi: `race_state`, `races`, `races_unread`, `race_used`, `race_size` [B], `race_rec_s`, `race_overflows` (próbki zgubione przez pełny bufor RAM), `race_scan_ms` (skan przy starcie) |
| `TIME <unix_s>` | `OK` |
| `LIST` | `{"runs":[{"seq","len","crc","unix","boot","uptime_ms"[,"ram":true]}…]}` od najstarszego |
| `GET <seq> <offset> <len>` | ramka binarna; `len ≤ 4096`; przez BLE przełącza na PHY 2M (szybsze pobieranie) |
| `DEL <seq>` | `OK DEL <seq>` albo `{"err":"NOT_FOUND","seq":…}` |
| `ARM` | `OK` – powrót z ERROR do IDLE |
| `LIVE 1` / `LIVE 0` | `OK`; potem 20 Hz: `{"live":1,"v","v_max","v_front","slip","rpm","thr","thr_pct","ax","zupt","state","race","race_s","d":[…]}` – `race`: stan zapisu wyścigu, `race_s`: długość bieżącego wyścigu [s]; drift (v3) w tablicy `d` (cała linia mieści się w jednej notyfikacji BLE): `[pts, mult, total, last, best, ang, on, chain, banks, lost, trans]` = punkty bieżącego łańcucha (przed mnożnikiem), mnożnik, suma, ostatni zaliczony łańcuch (×mnożnik), najlepszy, kąt driftu [°] (0 poza łukiem), drift teraz (0/1), łańcuch trwa (0/1), zaliczone łańcuchy, obroty, przekładki. Wynik liczy Xiao – zgubione linie LIVE nic nie zmieniają. `LIVE 1` przez BLE przełącza na PHY 1M (zasięg) |
| `VMAX RESET` | `OK` |
| `DRIFT RESET` | `OK` – zerowanie punktów za drift (v3) |
| `MOUNT [<forward> <left> <up> <yaw_deg>]` | `{"mount":[…]}` (v3) – bez argumentów odczyt; np. `MOUNT +y -x +z -18.5` ustawia montaż płytki dla prędkościomierza i driftu (układ prawoskrętny, \|yaw\| ≤ 180), do wyłączenia zasilania; zmiana montażu zeruje estymator, filtry i punkty driftu; aplikacja wysyła wynik kalibracji po połączeniu. Zła wartość: `{"err":"BAD_MOUNT"}` |
| `RLIST` | `{"races":[{"seq","len","crc","unix","boot","uptime_ms","n","end","read"}…],"state","used","size"}` – od najstarszego; `read`: oznaczony jako pobrany; `state`: IDLE / RECORDING / FINISHING / FULL / ERASING / DIRTY / ERROR |
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
