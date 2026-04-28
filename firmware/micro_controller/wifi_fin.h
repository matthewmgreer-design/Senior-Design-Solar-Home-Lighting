#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void wifi_init_sta(void);
bool wifi_is_connected(void);

#ifdef __cplusplus
}
#endif