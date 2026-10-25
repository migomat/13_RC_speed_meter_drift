#include "mount_store.h"

#include <Arduino.h>
#include <string.h>
#include "config.h"
#include "crc32.h"
#include "flash_store.h"

// Rekordy po 64 B dopisywane po kolei w CFG_AREA (QSPI). Obowiązuje ostatni poprawny.
// Zapis rekordu = programowanie strony (< 1 ms, bez kasowania); pełny obszar (1024 zmiany)
// → kasowanie bloku w tle (flash_poll), potem zapis od początku.
#define MOUNT_MAGIC   0x4D463458u   // "X4FM"
#define MOUNT_VERSION 1
#define REC_SIZE      64
#define REC_COUNT     (CFG_AREA_SIZE / REC_SIZE)

struct __attribute__((packed)) MountRec {
  uint32_t magic;
  uint16_t version;
  int8_t   idx[3], sign[3];
  float    yaw_deg;
  int8_t   steer_sign;
  char     name[MOUNT_NAME_LEN];
  uint8_t  reserved[REC_SIZE - 4 - 2 - 6 - 4 - 1 - MOUNT_NAME_LEN - 4];
  uint32_t crc32;   // CRC bajtów przed tym polem
};
static_assert(sizeof(MountRec) == REC_SIZE, "MountRec = 64 B");

char g_mount_name[MOUNT_NAME_LEN];
static bool     s_pending, s_saved;
static uint32_t s_next;            // indeks następnego wolnego rekordu
static uint32_t s_save_us;

uint32_t mount_store_save_us(void) { return s_save_us; }

static bool rec_valid(const MountRec& m) {
  if (m.magic != MOUNT_MAGIC || m.version != MOUNT_VERSION || crc32(&m, REC_SIZE - 4) != m.crc32) return false;
  for (int k = 0; k < 3; k++)
    if (m.idx[k] < 0 || m.idx[k] > 2 || (m.sign[k] != 1 && m.sign[k] != -1)) return false;
  if (m.idx[0] == m.idx[1] || m.idx[0] == m.idx[2] || m.idx[1] == m.idx[2]) return false;
  return m.yaw_deg >= -180.0f && m.yaw_deg <= 180.0f && m.steer_sign >= -1 && m.steer_sign <= 1;
}

static bool all_ff(const void* p, size_t n) {
  const uint8_t* b = (const uint8_t*)p;
  for (size_t i = 0; i < n; i++)
    if (b[i] != 0xFF) return false;
  return true;
}

bool mount_store_load(void) {
  MountRec m, last;
  bool found = false;
  s_next = REC_COUNT;
  for (uint32_t i = 0; i < REC_COUNT; i++) {
    if (!flash_cfg_read(i * REC_SIZE, &m, sizeof(m))) return false;
    if (all_ff(&m, sizeof(m))) {
      s_next = i;
      break;
    }
    if (rec_valid(m)) {
      last = m;
      found = true;
    }
  }
  if (!found) return false;
  for (int k = 0; k < 3; k++) {
    g_mount.idx[k] = last.idx[k];
    g_mount.sign[k] = last.sign[k];
  }
  g_mount.yaw_deg = last.yaw_deg;
  g_mount.steer_sign = last.steer_sign;
  memcpy(g_mount_name, last.name, MOUNT_NAME_LEN);
  g_mount_name[MOUNT_NAME_LEN - 1] = 0;
  s_saved = true;
  return true;
}

void mount_store_request_save(void) {
  s_pending = true;
  s_saved = false;
}

bool mount_store_pending(void) { return s_pending; }
bool mount_store_saved(void) { return s_saved; }

void mount_store_poll(bool busy) {
  if (!s_pending || busy || flash_cfg_erasing() || flash_raw_busy()) return;
  if (s_next >= REC_COUNT) {   // obszar pełny (albo uszkodzony) – kasowanie w tle
    flash_cfg_erase_start();
    s_next = 0;
    return;
  }
  MountRec m;
  memset(&m, 0, sizeof(m));
  m.magic = MOUNT_MAGIC;
  m.version = MOUNT_VERSION;
  for (int k = 0; k < 3; k++) {
    m.idx[k] = (int8_t)g_mount.idx[k];
    m.sign[k] = (int8_t)g_mount.sign[k];
  }
  m.yaw_deg = g_mount.yaw_deg;
  m.steer_sign = (int8_t)g_mount.steer_sign;
  memcpy(m.name, g_mount_name, MOUNT_NAME_LEN);
  m.crc32 = crc32(&m, REC_SIZE - 4);
  uint32_t t0 = micros();
  if (!flash_cfg_write(s_next * REC_SIZE, &m, sizeof(m))) return;   // spróbuje w następnej pętli
  s_save_us = micros() - t0;
  s_next++;
  s_pending = false;
  s_saved = true;
}
