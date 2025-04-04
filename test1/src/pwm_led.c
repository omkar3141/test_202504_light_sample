/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include "pwm_led.h"

LOG_MODULE_REGISTER(pwm_led, CONFIG_BT_MESH_MODEL_LOG_LEVEL);

/* PWM LED devices for nRF52840 DK */
#define PWM_LED0_NODE DT_ALIAS(pwm_led0)
#define PWM_LED1_NODE DT_ALIAS(pwm_led1)
#define PWM_LED2_NODE DT_ALIAS(pwm_led2)
#define PWM_LED3_NODE DT_ALIAS(pwm_led3)

/* PWM period in microseconds - 1 ms = 1000 Hz */
#define PWM_PERIOD_US 1000

/* PWM LED specifications */
static const struct pwm_dt_spec pwm_leds[] = {
#if DT_NODE_HAS_STATUS(PWM_LED0_NODE, okay)
	PWM_DT_SPEC_GET(PWM_LED0_NODE),
#else
	{ 0 },
#endif
#if DT_NODE_HAS_STATUS(PWM_LED1_NODE, okay)
	PWM_DT_SPEC_GET(PWM_LED1_NODE),
#else
	{ 0 },
#endif
#if DT_NODE_HAS_STATUS(PWM_LED2_NODE, okay)
	PWM_DT_SPEC_GET(PWM_LED2_NODE),
#else
	{ 0 },
#endif
#if DT_NODE_HAS_STATUS(PWM_LED3_NODE, okay)
	PWM_DT_SPEC_GET(PWM_LED3_NODE),
#else
	{ 0 },
#endif
};

/* Number of available PWM LEDs */
#define PWM_LED_COUNT ARRAY_SIZE(pwm_leds)

int pwm_led_init(void)
{
	int err = 0;

	/* Check if all PWM devices are ready */
	for (int i = 0; i < PWM_LED_COUNT; i++) {
		if (pwm_leds[i].dev != NULL && !device_is_ready(pwm_leds[i].dev)) {
			LOG_ERR("Error: PWM device %s is not ready", pwm_leds[i].dev->name);
			err = -ENODEV;
		}
	}

	LOG_INF("PWM LED module initialized with %d LEDs", PWM_LED_COUNT);
	return err;
}

int pwm_led_set(uint8_t led_idx, uint8_t level)
{
	if (led_idx >= PWM_LED_COUNT || pwm_leds[led_idx].dev == NULL) {
		return -EINVAL;
	}

	/* Convert 0-100 level to pulse width (0 = off, 100 = full brightness) */
	uint32_t pulse_width = (level * PWM_PERIOD_US) / 100;

	/* For active-low LEDs (like on nRF52840 DK), invert the pulse width */
	pulse_width = PWM_PERIOD_US - pulse_width;

	return pwm_set_dt(&pwm_leds[led_idx], PWM_USEC(PWM_PERIOD_US), PWM_USEC(pulse_width));
}
