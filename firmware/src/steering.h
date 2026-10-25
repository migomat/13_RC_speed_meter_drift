// Pomiar szerokości impulsu skrętu (PIN_STEERING = D7; PPI capture jak gaz) i uczenie neutrum.
// Kierunek „w lewo” rozpoznaje drift.c (nauka) albo kalibracja w aplikacji (MOUNT).
#pragma once

#include <stdint.h>

bool     steering_begin(void);          // po włączeniu SoftDevice
uint16_t steering_tick_width(uint32_t t_tick);   // ISR taktu: ostatni poprawny impuls albo 0
void     steering_poll(uint32_t now_us);         // pętla główna: uczenie neutrum
bool     steering_neutral_learned(void);
uint16_t steering_neutral_us(void);
uint32_t steering_pulse_count(void);
uint32_t steering_reject_count(void);
