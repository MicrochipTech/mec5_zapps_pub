/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _APP_THREAD2_H
#define _APP_THREAD2_H

#include <stdint.h>
#include <zephyr/kernel.h>

int app_thread2_create(uint32_t start_delay_ms);
k_tid_t app_thread2_get_tid(void);

#endif /* _APP_THREAD2_H */
