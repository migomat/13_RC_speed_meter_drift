// Komendy tekstowe (USB Serial i BLE NUS) – lista, pobieranie, kasowanie,
// podgląd na żywo. Opis protokołu: docs/PROTOCOL.md.
#pragma once

#include <stdint.h>

// Dane do STATUS / LIVE, aktualizowane przez pętlę główną
struct LiveData {
  float    v, v_max, v_front;   // m/s
  bool     spinning;
  float    rpm;                 // obroty silnika
  uint16_t throttle_us;
  float    throttle_pct;
  float    ax_g;                // przyspieszenie wzdłużne (surowe, oś „do przodu”)
  bool     zupt;
  // punkty driftu (drift.c)
  float    d_pts, d_total, d_last, d_best, d_angle;
  int      d_mult;
  bool     d_on, d_chain;
  uint32_t d_banks, d_lost, d_trans;
  bool     d_cs;                // kontra
  int      d_steer_sign;        // znak skrętu w użyciu (0 = nieznany)
  float    steer_pct;           // skręt [%] (+ = w lewo, gdy znak znany)
  bool     steer_valid;
};
// Montaż płytki (main.cpp): osie auta forward/left/up = oś płytki idx (0=x,1=y,2=z) · znak
struct Mount {
  int   idx[3];
  int   sign[3];
  float yaw_deg;
  int   steer_sign;   // skręt: 1 normalny, -1 odwrócony, 0 = nauka (drift.c)
};
extern Mount g_mount;
extern LiveData g_live;
extern uint32_t g_stack_free_min;   // [B] najmniejszy zapas stosu zadania loop
extern volatile uint32_t g_tick_overflows;
extern uint32_t g_boot_gpregret, g_boot_hf;   // diagnostyka resetów (main.cpp)
void enter_bootloader(uint8_t magic);         // main.cpp

bool ble_begin(void);           // konfiguracja SoftDevice – przed timebase/hall
void links_poll(uint32_t now_ms);
bool ble_connected(void);
bool vmax_reset_requested(void);
bool drift_reset_requested(void);
bool mount_changed(void);      // MOUNT zmienił osie/kąt – estymator i filtry driftu od nowa
// Wołane w trakcie czekania na bufor BLE (odczyt IMU, takty) – pomiar nie staje
void ble_set_wait_hook(void (*fn)(void));
