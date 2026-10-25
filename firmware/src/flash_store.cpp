#include "flash_store.h"

#include <Arduino.h>
#include <Adafruit_SPIFlash.h>
#include "config.h"
#include "crc32.h"
#include "run_format.h"

static Adafruit_FlashTransport_QSPI s_transport;
static Adafruit_SPIFlash s_flash(&s_transport);

static SlotInfo s_slots[FLASH_NUM_SLOTS];
static bool     s_ok;
static uint32_t s_jedec;
static uint32_t s_next_seq = 1;
static uint32_t s_boot_count = 1;
static uint32_t s_overwritten_unread;
volatile uint32_t g_flash_test_reset_at;

// Kasowanie w tle: slot i bieżący blok (od ostatniego do pierwszego –
// blok z nagłówkiem kasowany na końcu, więc nagłówek FF ⇒ cały slot czysty).
static int  s_erase_slot = -1;
static int  s_erase_block;
static bool s_erase_running;

#define BLOCKS_PER_SLOT (FLASH_SLOT_SIZE / FLASH_ERASE_BLOCK)
#define RACE_BLOCKS     (RACE_AREA_SIZE / FLASH_ERASE_BLOCK)
static_assert(RACE_AREA_ADDR + RACE_AREA_SIZE <= FLASH_TOTAL_SIZE, "Obszar wyścigów poza flash");
static_assert(RACE_AREA_SIZE % FLASH_ERASE_BLOCK == 0, "Obszar wyścigów = wielokrotność 64 kB");

// Kasowanie obszaru wyścigów: następny blok (od końca) i czy blok trwa.
// Bloki slotów mają pierwszeństwo.
static int  s_race_erase_left;
static bool s_race_erase_running;
static bool s_cfg_erase_pending, s_cfg_erase_running;

static inline uint32_t slot_addr(int i) { return (uint32_t)i * FLASH_SLOT_SIZE; }

static uint8_t read_status(void) {
  uint8_t sr = 0xFF;
  s_transport.readCommand(0x05, &sr, 1);
  return sr;
}

static bool wip(void) { return read_status() & 0x01; }

// Czeka tylko na WIP. Nie rusza automatu kasowania – kolejny blok uruchamia
// wyłącznie flash_poll() (inaczej GET/DEL w trakcie kasowania zrywał kasowanie
// i slot zostawał w stanie „pobrany” na zawsze).
static void wait_ready(void) {
  while (wip()) {
  }
}

static bool write_word(uint32_t addr, uint32_t v) {
  wait_ready();
  return s_flash.writeBuffer(addr, (const uint8_t*)&v, 4) == 4;
}

static void scan_slot(int i) {
  SlotInfo& s = s_slots[i];
  memset(&s, 0, sizeof(s));
  uint32_t h[5];
  s_flash.readBuffer(slot_addr(i), (uint8_t*)h, sizeof(h));
  if (h[0] == 0xFFFFFFFFu && h[1] == 0xFFFFFFFFu && h[2] == 0xFFFFFFFFu &&
      h[3] == 0xFFFFFFFFu && h[4] == 0xFFFFFFFFu) {
    s.state = SLOT_FREE;
    return;
  }
  s.seq = h[1];
  uint8_t st = (uint8_t)(h[2] & 0xFF);
  if (h[0] != SLOT_MAGIC || (h[2] | 0xFF) != 0xFFFFFFFFu || h[1] == 0xFFFFFFFFu) {
    s.state = SLOT_BAD;
    s.erase_pending = true;
    return;
  }
  if (h[1] >= s_next_seq) s_next_seq = h[1] + 1;
  if (st != SLOT_COMPLETE) {
    s.state = (st == SLOT_DOWNLOADED) ? SLOT_DOWNLOADED : SLOT_BAD;
    s.erase_pending = true;
    return;
  }
  s.len = h[3];
  s.crc = h[4];
  if (s.len < RUN_HEADER_SIZE || s.len > FLASH_SLOT_SIZE - FLASH_SLOT_HDR_SIZE) {
    s.state = SLOT_BAD;
    s.erase_pending = true;
    return;
  }
  // Weryfikacja CRC całych danych – niespójny slot idzie do skasowania.
  static uint8_t buf[1024] __attribute__((aligned(4)));
  uint32_t crc = crc32_begin();
  RunHeader rh;
  for (uint32_t off = 0; off < s.len; off += sizeof(buf)) {
    uint32_t n = min((uint32_t)sizeof(buf), s.len - off);
    s_flash.readBuffer(slot_addr(i) + FLASH_SLOT_HDR_SIZE + off, buf, n);
    if (off == 0) {
      memcpy(&rh, buf, sizeof(rh));
      memset(buf + 124, 0, 4);  // CRC startu liczone z polem crc32 = 0
    }
    crc = crc32_update(crc, buf, n);
  }
  crc = crc32_end(crc);
  if (crc != s.crc || rh.magic != RUN_MAGIC) {
    s.state = SLOT_BAD;
    s.erase_pending = true;
    return;
  }
  s.state = SLOT_COMPLETE;
  s.unix_s = rh.unix_s;
  s.boot_count = rh.boot_count;
  s.uptime_ms = rh.uptime_ms;
  if (rh.boot_count >= s_boot_count) s_boot_count = rh.boot_count + 1;
  if (rh.seq >= s_next_seq) s_next_seq = rh.seq + 1;
}

bool flash_begin(void) {
  // Na wszelki wypadek wybudzenie z deep power-down (0xAB) – układ w DPD
  // odpowiada tylko na tę komendę.
  s_transport.begin();
  s_transport.runCommand(0xAB);
  delayMicroseconds(50);
  uint8_t id[3] = {0, 0, 0};
  s_transport.readCommand(0x9F, id, 3);
  s_jedec = ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
  // P25Q16H (JEDEC 85 60 15) nie jest na domyślnej liście Adafruit_SPIFlash
  // z rdzeniem Seeed non-mbed – przekazujemy deskryptor jawnie.
  static const SPIFlash_Device_t kDev = P25Q16H;
  s_ok = s_flash.begin(&kDev, 1);
  if (!s_ok) return false;
  wait_ready();
  s_boot_count = 1;
  for (int i = 0; i < FLASH_NUM_SLOTS; i++) scan_slot(i);
  return true;
}

bool flash_ok(void) { return s_ok; }
uint32_t flash_jedec_id(void) { return s_jedec; }

static void check_blank_step(int i) {
  static uint32_t buf[256];
  SlotInfo& s = s_slots[i];
  s_flash.readBuffer(slot_addr(i) + s.check_off, (uint8_t*)buf, sizeof(buf));
  for (uint32_t k = 0; k < 256; k++) {
    if (buf[k] != 0xFFFFFFFFu) {
      s.state = SLOT_BAD;
      s.erase_pending = true;
      return;
    }
  }
  s.check_off += sizeof(buf);
  if (s.check_off >= FLASH_SLOT_SIZE) s.blank_ok = true;
}

static void start_erase_block(void) {
  uint32_t addr = slot_addr(s_erase_slot) + (uint32_t)s_erase_block * FLASH_ERASE_BLOCK;
  s_transport.runCommand(0x06);  // WREN
  s_transport.eraseCommand(0xD8, addr);
  s_erase_running = true;
}

static void start_race_erase_block(void) {
  uint32_t addr = RACE_AREA_ADDR + (uint32_t)(s_race_erase_left - 1) * FLASH_ERASE_BLOCK;
  s_transport.runCommand(0x06);  // WREN
  s_transport.eraseCommand(0xD8, addr);
  s_race_erase_running = true;
}

void flash_poll(void) {
  if (!s_ok) return;
  if (s_race_erase_running) {
    if (wip()) return;
    s_race_erase_running = false;
    s_race_erase_left--;
  }
  if (s_cfg_erase_running) {
    if (wip()) return;
    s_cfg_erase_running = false;
    s_cfg_erase_pending = false;
  }
  if (s_erase_running) {
    if (wip()) return;
    s_erase_running = false;
    if (s_erase_block > 0) {
      s_erase_block--;
      start_erase_block();
      return;
    }
    SlotInfo& s = s_slots[s_erase_slot];
    memset(&s, 0, sizeof(s));
    s.state = SLOT_FREE;
    s.blank_ok = true;
    s_erase_slot = -1;
  }
  for (int i = 0; i < FLASH_NUM_SLOTS; i++) {
    if (s_slots[i].erase_pending) {
      s_slots[i].erase_pending = false;
      s_erase_slot = i;
      s_erase_block = BLOCKS_PER_SLOT - 1;
      start_erase_block();
      return;
    }
  }
  if (s_race_erase_left > 0) {
    start_race_erase_block();
    return;
  }
  if (s_cfg_erase_pending) {
    s_transport.runCommand(0x06);  // WREN
    s_transport.eraseCommand(0xD8, CFG_AREA_ADDR);
    s_cfg_erase_running = true;
    return;
  }
  // Sprawdzanie pustości wolnych slotów po starcie (np. śmieci po innym
  // firmware) – jeden kawałek na wywołanie.
  for (int i = 0; i < FLASH_NUM_SLOTS; i++) {
    if (s_slots[i].state == SLOT_FREE && !s_slots[i].blank_ok) {
      check_blank_step(i);
      return;
    }
  }
}

static bool blank_pending(void);

bool flash_busy(void) {
  if (s_erase_slot >= 0) return true;
  for (int i = 0; i < FLASH_NUM_SLOTS; i++)
    if (s_slots[i].erase_pending) return true;
  return false;
}

bool flash_has_free_slot(void) {
  for (int i = 0; i < FLASH_NUM_SLOTS; i++)
    if (s_slots[i].state == SLOT_FREE && !s_slots[i].erase_pending && i != s_erase_slot) return true;
  return false;
}

static bool blank_pending(void) {
  for (int i = 0; i < FLASH_NUM_SLOTS; i++)
    if (s_slots[i].state == SLOT_FREE && !s_slots[i].blank_ok) return true;
  return false;
}

void flash_prepare_for_run(void) {
  if (!s_ok || flash_has_free_slot() || flash_busy()) return;
  // Brak wolnego: nadpisujemy najstarszy (najniższy seq), nawet niepobrany.
  int victim = -1;
  for (int i = 0; i < FLASH_NUM_SLOTS; i++) {
    if (victim < 0 || s_slots[i].seq < s_slots[victim].seq) victim = i;
  }
  if (victim < 0) return;
  if (s_slots[victim].state == SLOT_COMPLETE) s_overwritten_unread++;
  s_slots[victim].state = SLOT_BAD;
  s_slots[victim].erase_pending = true;
}

int flash_write_begin(uint32_t seq) {
  if (!s_ok) return -1;
  // Dokończ ewentualne kasowanie (zwykle już dawno skończone – 2×~17 ms).
  while (flash_busy() || blank_pending()) {
    wait_ready();
    flash_poll();
  }
  int slot = -1;
  for (int i = 0; i < FLASH_NUM_SLOTS; i++) {
    if (s_slots[i].state == SLOT_FREE && s_slots[i].blank_ok) {
      slot = i;
      break;
    }
  }
  if (slot < 0) {
    flash_prepare_for_run();
    while (flash_busy()) {
      wait_ready();
      flash_poll();
    }
    for (int i = 0; i < FLASH_NUM_SLOTS; i++)
      if (s_slots[i].state == SLOT_FREE) { slot = i; break; }
    if (slot < 0) return -1;
  }
  uint32_t a = slot_addr(slot);
  // Kolejność: magic + seq + stan FE → dane → długość i CRC → stan FC
  bool ok = write_word(a + 0, SLOT_MAGIC) && write_word(a + 4, seq) &&
            write_word(a + 8, 0xFFFFFF00u | SLOT_WRITING);
  SlotInfo& s = s_slots[slot];
  memset(&s, 0, sizeof(s));
  s.state = SLOT_WRITING;
  s.seq = seq;
  if (seq >= s_next_seq) s_next_seq = seq + 1;
  if (!ok) {
    s.state = SLOT_BAD;
    s.erase_pending = true;
    return -1;
  }
  return slot;
}

bool flash_write(int slot, uint32_t offset, const void* data, uint32_t len) {
  if (slot < 0 || offset + len > FLASH_SLOT_SIZE - FLASH_SLOT_HDR_SIZE) return false;
  if (g_flash_test_reset_at && offset >= g_flash_test_reset_at) {
    NVIC_SystemReset();  // symulacja zaniku zasilania w trakcie SAVING
  }
  wait_ready();
  uint32_t a = slot_addr(slot) + FLASH_SLOT_HDR_SIZE + offset;
  return s_flash.writeBuffer(a, (const uint8_t*)data, len) == len;
}

bool flash_write_finish(int slot, uint32_t len, uint32_t crc, uint32_t unix_s,
                        uint32_t boot, uint32_t uptime_ms) {
  uint32_t a = slot_addr(slot);
  bool ok = write_word(a + 12, len) && write_word(a + 16, crc);
  // Weryfikacja odczytem przed oznaczeniem jako kompletny
  if (ok) {
    static uint8_t buf[1024] __attribute__((aligned(4)));
    uint32_t c = crc32_begin();
    for (uint32_t off = 0; off < len; off += sizeof(buf)) {
      uint32_t n = min((uint32_t)sizeof(buf), len - off);
      s_flash.readBuffer(a + FLASH_SLOT_HDR_SIZE + off, buf, n);
      if (off == 0) memset(buf + 124, 0, 4);
      c = crc32_update(c, buf, n);
    }
    ok = crc32_end(c) == crc;
  }
  ok = ok && write_word(a + 8, 0xFFFFFF00u | SLOT_COMPLETE);
  wait_ready();
  SlotInfo& s = s_slots[slot];
  if (!ok) {
    s.state = SLOT_BAD;
    s.erase_pending = true;
    return false;
  }
  s.state = SLOT_COMPLETE;
  s.len = len;
  s.crc = crc;
  s.unix_s = unix_s;
  s.boot_count = boot;
  s.uptime_ms = uptime_ms;
  return true;
}

int flash_find(uint32_t seq) {
  for (int i = 0; i < FLASH_NUM_SLOTS; i++)
    if (s_slots[i].state == SLOT_COMPLETE && s_slots[i].seq == seq) return i;
  return -1;
}

bool flash_read(int slot, uint32_t offset, void* buf, uint32_t len) {
  if (slot < 0) return false;
  wait_ready();  // odczyt niemożliwy w trakcie kasowania (max ~17 ms)
  uint32_t a = slot_addr(slot) + FLASH_SLOT_HDR_SIZE + offset;
  return s_flash.readBuffer(a, (uint8_t*)buf, len) == len;
}

bool flash_mark_downloaded(uint32_t seq) {
  int i = flash_find(seq);
  if (i < 0) return false;
  if (!write_word(slot_addr(i) + 8, 0xFFFFFF00u | SLOT_DOWNLOADED)) return false;
  s_slots[i].state = SLOT_DOWNLOADED;
  s_slots[i].erase_pending = true;
  return true;
}

const SlotInfo* flash_slot(int i) { return &s_slots[i]; }

int flash_count_runs(void) {
  int n = 0;
  for (int i = 0; i < FLASH_NUM_SLOTS; i++) n += s_slots[i].state == SLOT_COMPLETE;
  return n;
}

int flash_count_erased(void) {
  int n = 0;
  for (int i = 0; i < FLASH_NUM_SLOTS; i++)
    n += s_slots[i].state == SLOT_FREE && s_slots[i].blank_ok && !s_slots[i].erase_pending;
  return n;
}

int flash_count_free(void) {
  int n = 0;
  for (int i = 0; i < FLASH_NUM_SLOTS; i++)
    n += s_slots[i].state != SLOT_COMPLETE && s_slots[i].state != SLOT_WRITING;
  return n;
}

uint32_t flash_next_seq(void) { return s_next_seq; }
uint32_t flash_boot_count(void) { return s_boot_count; }
uint32_t flash_overwritten_unread(void) { return s_overwritten_unread; }

int flash_sorted(int* out_idx, int max) {
  int n = 0;
  for (int i = 0; i < FLASH_NUM_SLOTS && n < max; i++)
    if (s_slots[i].state == SLOT_COMPLETE) out_idx[n++] = i;
  for (int a = 1; a < n; a++) {
    int v = out_idx[a], b = a - 1;
    while (b >= 0 && s_slots[out_idx[b]].seq > s_slots[v].seq) {
      out_idx[b + 1] = out_idx[b];
      b--;
    }
    out_idx[b + 1] = v;
  }
  return n;
}

// ------------------------------------------------------------ obszar wyścigów
static inline bool in_race_area(uint32_t addr, uint32_t len) {
  return addr >= RACE_AREA_ADDR && len <= RACE_AREA_SIZE &&
         addr - RACE_AREA_ADDR <= RACE_AREA_SIZE - len;
}

bool flash_raw_busy(void) { return !s_ok || wip(); }

bool flash_raw_read(uint32_t addr, void* buf, uint32_t len) {
  if (!s_ok || !in_race_area(addr, len)) return false;
  wait_ready();
  return s_flash.readBuffer(addr, (uint8_t*)buf, len) == len;
}

bool flash_raw_write(uint32_t addr, const void* data, uint32_t len) {
  if (!s_ok || !in_race_area(addr, len) || flash_race_erasing()) return false;
  wait_ready();
  return s_flash.writeBuffer(addr, (const uint8_t*)data, len) == len;
}

void flash_race_erase_start(void) {
  if (!s_ok || flash_race_erasing()) return;
  s_race_erase_left = RACE_BLOCKS;
}

bool flash_race_erasing(void) { return s_race_erase_left > 0; }

static bool in_cfg(uint32_t off, uint32_t len) { return len <= CFG_AREA_SIZE && off <= CFG_AREA_SIZE - len; }

bool flash_cfg_read(uint32_t off, void* buf, uint32_t len) {
  if (!s_ok || !in_cfg(off, len)) return false;
  wait_ready();
  return s_flash.readBuffer(CFG_AREA_ADDR + off, (uint8_t*)buf, len) == len;
}

bool flash_cfg_write(uint32_t off, const void* data, uint32_t len) {
  if (!s_ok || !in_cfg(off, len) || s_cfg_erase_pending || wip()) return false;
  return s_flash.writeBuffer(CFG_AREA_ADDR + off, (const uint8_t*)data, len) == len;
}

void flash_cfg_erase_start(void) { if (s_ok) s_cfg_erase_pending = true; }
bool flash_cfg_erasing(void) { return s_cfg_erase_pending; }

void flash_note_seq(uint32_t seq) {
  if (seq != 0xFFFFFFFFu && seq >= s_next_seq) s_next_seq = seq + 1;
}

void flash_note_boot(uint32_t boot) {
  if (boot != 0xFFFFFFFFu && boot >= s_boot_count) s_boot_count = boot + 1;
}
