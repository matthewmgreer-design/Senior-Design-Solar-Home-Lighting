#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void lighting_init(void);
void lighting_set_porch(bool on);
void lighting_set_foyer(bool on);
void lighting_set_security(bool on);

#ifdef __cplusplus
}
#endif