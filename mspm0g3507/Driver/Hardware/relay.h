#ifndef RELAY_H
#define RELAY_H

#include <stdbool.h>
#include <stdint.h>

/* Set to 0U if the relay board is low-level triggered. */
#define RELAY_ACTIVE_HIGH (1U)

void Relay_Init(void);
void Relay_Set(bool on);
void Relay_On(void);
void Relay_Off(void);
void Relay_Toggle(void);
bool Relay_IsOn(void);

#endif
