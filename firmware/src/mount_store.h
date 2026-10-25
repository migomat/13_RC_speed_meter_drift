// Montaż płytki i nazwa profilu auta zapisane na stałe w QSPI (CFG_AREA – ostatni blok
// 64 kB, osobno od startów i wyścigów). Po resecie Xiao wczytuje ostatni profil wysłany
// z aplikacji komendą MOUNT. (InternalFS/LittleFS odrzucony: zapis 750–940 ms blokował pętlę.)
#pragma once

#include <stdint.h>
#include "ble_link.h"

#define MOUNT_NAME_LEN 24   // [B] z zerem na końcu

extern char g_mount_name[MOUNT_NAME_LEN];

bool mount_store_load(void);           // setup(), po flash_begin(); false = brak zapisu (wartości z config.h)
void mount_store_request_save(void);   // MOUNT zmienił montaż/nazwę – zapis, gdy nic się nie nagrywa
bool mount_store_pending(void);
bool mount_store_saved(void);          // bieżący montaż jest zapisany
void mount_store_poll(bool busy);
uint32_t mount_store_save_us(void);  // [us] czas ostatniego zapisu (diagnostyka)      // pętla główna; busy = nagrywanie/zapis – zapis później
