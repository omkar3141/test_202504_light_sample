/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic mesh OnOff server with PWM LED control sample
 */
#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <bluetooth/mesh/dk_prov.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include "model_handler.h"

LOG_MODULE_REGISTER(bt_mesh_onoff_srv_sample, LOG_LEVEL_INF);

static void bt_ready(int err)
{
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}

	LOG_INF("Bluetooth initialized");

	/* Initialize the button and LED hardware */
	err = dk_leds_init();
	if (err) {
		LOG_ERR("Initializing LEDs failed (err %d)", err);
		return;
	}

	err = dk_buttons_init(NULL);
	if (err) {
		LOG_ERR("Initializing buttons failed (err %d)", err);
		return;
	}

	/* Initialize the Bluetooth mesh stack */
	err = bt_mesh_init(bt_mesh_dk_prov_init(), model_handler_init());
	if (err) {
		LOG_ERR("Initializing mesh failed (err %d)", err);
		return;
	}

	/* Load stored settings (if available) */
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	/* This will be a no-op if settings_load() loaded provisioning info */
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

	LOG_INF("Mesh initialized");
	LOG_INF("OnOff Server with PWM LED control initialized");
}

int main(void)
{
	int err;

	LOG_INF("Initializing...");
	LOG_INF("Bluetooth Mesh OnOff Server with PWM LED control");

	/* Initialize and enable the Bluetooth subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
	}

	return 0;
}
