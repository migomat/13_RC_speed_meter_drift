// Bufor pierścieniowy startów w zewnętrznym flash QSPI (P25Q16H, 2 MB).
// FLASH_NUM_SLOTS stałych slotów po FLASH_SLOT_SIZE, bez systemu plików.
// Nagłówek slotu (strona 0 slotu), każde pole w osobnym słowie 32-bit:
//   +0 magic "X4FS", +4 seq, +8 stan (bajt niski), +12 długość, +16 CRC32 danych
// Stany (przejścia tylko 1→0): FF wolny, FE zapis, FC kompletny, F8 pobrany.
// Dane startu (RunHeader + próbki + zbocza) od offsetu FLASH_SLOT_HDR_SIZE.
#pragma once

#include <stdint.h>
#include <stddef.h>

#define SLOT_MAGIC       0x53463458u   // "X4FS"
#define SLOT_FREE        0xFF
#define SLOT_WRITING     0xFE
#define SLOT_COMPLETE    0xFC
#define SLOT_DOWNLOADED  0xF8
#define SLOT_BAD         0x00   // tylko w RAM: niespójny, do skasowania

struct SlotInfo {
  uint8_t  state;
  bool     erase_pending;
  bool     blank_ok;        // wolny slot sprawdzony (same 0xFF) albo skasowany przez nas
  uint32_t check_off;       // postęp sprawdzania pustości
  uint32_t seq, len, crc;
  uint32_t unix_s, boot_count, uptime_ms;
};

bool     flash_begin(void);              // inicjalizacja + skan slotów
bool     flash_ok(void);
uint32_t flash_jedec_id(void);
// Kasowanie w tle (nieblokujące). Nie wołać w trakcie SAVING.
void     flash_poll(void);
bool     flash_busy(void);               // trwa kasowanie
// W chwili wyzwolenia: zapewnia wolny slot (ew. kasuje najstarszy).
void     flash_prepare_for_run(void);
bool     flash_has_free_slot(void);      // wolny i skasowany
// Zapis startu
int      flash_write_begin(uint32_t seq);
bool     flash_write(int slot, uint32_t offset, const void* data, uint32_t len);
bool     flash_write_finish(int slot, uint32_t len, uint32_t crc,
                            uint32_t unix_s, uint32_t boot, uint32_t uptime_ms);
// Odczyt / kasowanie
int      flash_find(uint32_t seq);        // indeks slotu kompletnego albo -1
bool     flash_read(int slot, uint32_t offset, void* buf, uint32_t len);
bool     flash_mark_downloaded(uint32_t seq);
const SlotInfo* flash_slot(int i);
int      flash_count_runs(void);         // kompletne (niepobrane)
int      flash_count_free(void);         // bez startów (także czekające na kasowanie)
int      flash_count_erased(void);       // skasowane i sprawdzone – gotowe do zapisu
uint32_t flash_next_seq(void);
uint32_t flash_boot_count(void);         // ten rozruch
uint32_t flash_overwritten_unread(void);
// Testy: kolejność slotów do LIST (od najstarszego)
int      flash_sorted(int* out_idx, int max);
// Obszar wyścigów (RACE_AREA_ADDR..+RACE_AREA_SIZE) – dostęp surowy, adresy
// bezwzględne. Logika rekordów: race.cpp.
bool     flash_raw_busy(void);           // trwa zapis/kasowanie (bez czekania)
bool     flash_raw_read(uint32_t addr, void* buf, uint32_t len);
bool     flash_raw_write(uint32_t addr, const void* data, uint32_t len);
void     flash_race_erase_start(void);   // kasowanie całego obszaru w tle (flash_poll)
bool     flash_race_erasing(void);
// Obszar profilu (CFG_AREA_ADDR..+CFG_AREA_SIZE) – offsety względem początku obszaru.
bool     flash_cfg_read(uint32_t off, void* buf, uint32_t len);
bool     flash_cfg_write(uint32_t off, const void* data, uint32_t len);   // tylko gdy !flash_raw_busy()
void     flash_cfg_erase_start(void);    // kasowanie bloku w tle (flash_poll)
bool     flash_cfg_erasing(void);
void     flash_note_seq(uint32_t seq);   // seq / boot_count z wyścigów (wspólna numeracja)
void     flash_note_boot(uint32_t boot);
// Test zaniku zasilania: reset w połowie zapisu danych
extern volatile uint32_t g_flash_test_reset_at;
