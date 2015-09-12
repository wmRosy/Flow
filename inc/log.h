#pragma once

#include <stdint.h>

extern int log_ready;
#ifdef __cplusplus
extern "C" {
#endif
int format_sdcard();

int log_init();
int log_write(const void *data, int size);
int log_flush();
#ifdef __cplusplus
}
#endif