/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file
 * @brief PWM LED control for Bluetooth mesh
 */

#ifndef PWM_LED_H__
#define PWM_LED_H__

#include <zephyr/kernel.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the PWM LED module.
 *
 * @return 0 on success, or (negative) error code otherwise.
 */
int pwm_led_init(void);

/**
 * @brief Set the LED brightness level.
 *
 * @param led_idx LED index (0-3 for nRF52840 DK)
 * @param level Brightness level (0-100)
 * @return 0 on success, or (negative) error code otherwise.
 */
int pwm_led_set(uint8_t led_idx, uint8_t level);

#ifdef __cplusplus
}
#endif

#endif /* PWM_LED_H__ */
