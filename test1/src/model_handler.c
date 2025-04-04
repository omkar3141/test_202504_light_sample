/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/logging/log.h>
#include "model_handler.h"
#include "pwm_led.h"

LOG_MODULE_REGISTER(model_handler, CONFIG_BT_MESH_MODEL_LOG_LEVEL);

/* Forward declarations */
static void led_set(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    const struct bt_mesh_onoff_set *set,
		    struct bt_mesh_onoff_status *rsp);

static void led_get(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    struct bt_mesh_onoff_status *rsp);

/* Define handlers for the OnOff Server model */
static const struct bt_mesh_onoff_srv_handlers onoff_handlers = {
	.set = led_set,
	.get = led_get,
};

/* Context for each LED (OnOff server instance) */
struct led_ctx {
	struct bt_mesh_onoff_srv srv;
	struct k_work_delayable work;
	uint8_t led_idx;
	uint8_t current_level;
	uint8_t target_level;
	uint32_t remaining_time_ms;
	bool value;
	bool is_on;
};

/* Create LED contexts for each button/LED pair on the nRF52840 DK */
static struct led_ctx led_ctx[] = {
#if DT_NODE_EXISTS(DT_ALIAS(led0))
	{
		.srv = BT_MESH_ONOFF_SRV_INIT(&onoff_handlers),
		.led_idx = 0,
	},
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
	{
		.srv = BT_MESH_ONOFF_SRV_INIT(&onoff_handlers),
		.led_idx = 1,
	},
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
	{
		.srv = BT_MESH_ONOFF_SRV_INIT(&onoff_handlers),
		.led_idx = 2,
	},
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led3))
	{
		.srv = BT_MESH_ONOFF_SRV_INIT(&onoff_handlers),
		.led_idx = 3,
	},
#endif
};

/* Number of available LED contexts */
#define LED_CTX_COUNT ARRAY_SIZE(led_ctx)

/* Start LED transition */
static void led_transition_start(struct led_ctx *led)
{
	uint32_t transition_time = led->remaining_time_ms;
	
	LOG_DBG("Starting transition for LED %d: %d -> %d over %d ms",
		led->led_idx, led->current_level, led->target_level, transition_time);
	
	/* Schedule the transition work */
	k_work_reschedule(&led->work, K_MSEC(10)); /* Update every 10ms */
	
	/* If the transition is instant, update immediately */
	if (transition_time == 0) {
		led->current_level = led->target_level;
		pwm_led_set(led->led_idx, led->is_on ? led->current_level : 0);
	}
}

/* Fill in status response with current state */
static void led_status(struct led_ctx *led, struct bt_mesh_onoff_status *status)
{
	/* Calculate remaining time in milliseconds */
	status->remaining_time = led->remaining_time_ms ? 
		k_ticks_to_ms_ceil32(k_work_delayable_remaining_get(&led->work)) : 0;
	
	status->target_on_off = led->value;
	
	/* Present state is considered ON if we're in transition or target is ON */
	status->present_on_off = led->is_on;
	
	LOG_DBG("LED %d status: present=%d, target=%d, remaining=%d",
		led->led_idx, status->present_on_off, status->target_on_off, 
		status->remaining_time);
}

/* Handle OnOff SET message */
static void led_set(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    const struct bt_mesh_onoff_set *set,
		    struct bt_mesh_onoff_status *rsp)
{
	struct led_ctx *led = CONTAINER_OF(srv, struct led_ctx, srv);
	
	/* If state is already as requested, do nothing */
	if (set->on_off == led->value) {
		goto respond;
	}
	
	LOG_DBG("LED %d set: %d -> %d", led->led_idx, led->value, set->on_off);
	
	/* Store target state */
	led->value = set->on_off;
	
	/* Set target level based on on/off state */
	led->target_level = set->on_off ? 100 : 0;
	
	/* Handle transition */
	if (set->transition && set->transition->time) {
		/* Convert transition time to milliseconds */
		led->remaining_time_ms = bt_mesh_model_transition_time(set->transition);
		
		LOG_DBG("LED %d transition: %d ms", led->led_idx, led->remaining_time_ms);
		
		/* Handle transition delay if specified */
		if (set->transition->delay) {
			k_work_reschedule(&led->work, K_MSEC(set->transition->delay));
		} else {
			led_transition_start(led);
		}
	} else {
		/* Immediate transition */
		led->remaining_time_ms = 0;
		led->current_level = led->target_level;
		led->is_on = set->on_off;
		pwm_led_set(led->led_idx, led->is_on ? led->current_level : 0);
	}
	
respond:
	/* Fill in response if requested */
	if (rsp) {
		led_status(led, rsp);
	}
}

/* Handle OnOff GET message */
static void led_get(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    struct bt_mesh_onoff_status *rsp)
{
	struct led_ctx *led = CONTAINER_OF(srv, struct led_ctx, srv);
	
	led_status(led, rsp);
}

/* Handle LED transition work */
static void led_work(struct k_work *work)
{
	struct led_ctx *led = CONTAINER_OF(k_work_delayable_from_work(work), 
					   struct led_ctx, work);
	
	/* If we're in a transition */
	if (led->current_level != led->target_level) {
		/* Calculate progress */
		uint32_t elapsed = led->remaining_time_ms - 
			k_ticks_to_ms_ceil32(k_work_delayable_remaining_get(&led->work));
		uint32_t total = led->remaining_time_ms;
		
		if (elapsed >= total) {
			/* Transition complete */
			led->current_level = led->target_level;
			led->is_on = led->value;
			led->remaining_time_ms = 0;
		} else {
			/* Calculate new level based on linear transition */
			int32_t delta = (int32_t)led->target_level - (int32_t)led->current_level;
			int32_t step = (delta * elapsed) / total;
			led->current_level = led->current_level + step;
			
			/* Continue transition */
			k_work_reschedule(&led->work, K_MSEC(10));
		}
		
		/* Update LED brightness */
		pwm_led_set(led->led_idx, led->is_on ? led->current_level : 0);
		
		/* If transition complete, publish new state */
		if (led->current_level == led->target_level) {
			struct bt_mesh_onoff_status status;
			
			led_status(led, &status);
			bt_mesh_onoff_srv_pub(&led->srv, NULL, &status);
			
			LOG_DBG("LED %d transition complete: level=%d", 
				led->led_idx, led->current_level);
		}
	}
}

/* Button handler callback for controlling OnOff servers */
static void button_handler_cb(uint32_t pressed, uint32_t changed)
{
	if (!bt_mesh_is_provisioned()) {
		return;
	}
	
	/* Handle button presses to toggle corresponding LEDs */
	for (int i = 0; i < LED_CTX_COUNT; i++) {
		if (changed & BIT(i) && pressed & BIT(i)) {
			struct bt_mesh_onoff_set set = {
				.on_off = !led_ctx[i].value,
			};
			
			/* Toggle the LED state */
			led_set(&led_ctx[i].srv, NULL, &set, NULL);
			
			LOG_INF("Button %d pressed, toggling LED %d to %s", 
				i, i, set.on_off ? "ON" : "OFF");
		}
	}
}

/* Set up a repeating delayed work to blink the DK's LEDs when attention is
 * requested.
 */
static struct k_work_delayable attention_blink_work;
static bool attention;

static void attention_blink(struct k_work *work)
{
	static int idx;
	const uint8_t pattern[] = {
#if DT_NODE_EXISTS(DT_ALIAS(led0))
		BIT(0),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
		BIT(1),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
		BIT(2),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led3))
		BIT(3),
#endif
	};
	
	if (attention) {
		dk_set_leds(pattern[idx++ % ARRAY_SIZE(pattern)]);
		k_work_reschedule(&attention_blink_work, K_MSEC(30));
	} else {
		/* Restore LED states when attention mode ends */
		for (int i = 0; i < LED_CTX_COUNT; i++) {
			pwm_led_set(i, led_ctx[i].is_on ? led_ctx[i].current_level : 0);
		}
	}
}

static void attention_on(const struct bt_mesh_model *mod)
{
	attention = true;
	k_work_reschedule(&attention_blink_work, K_NO_WAIT);
}

static void attention_off(const struct bt_mesh_model *mod)
{
	/* Will stop rescheduling blink timer */
	attention = false;
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

/* Define mesh elements */
static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(
		1, 
		BT_MESH_MODEL_LIST(
			BT_MESH_MODEL_CFG_SRV,
			BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
			BT_MESH_MODEL_ONOFF_SRV(&led_ctx[0].srv)
		),
		BT_MESH_MODEL_NONE
	),
#if DT_NODE_EXISTS(DT_ALIAS(led1))
	BT_MESH_ELEM(
		2, 
		BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&led_ctx[1].srv)),
		BT_MESH_MODEL_NONE
	),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
	BT_MESH_ELEM(
		3, 
		BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&led_ctx[2].srv)),
		BT_MESH_MODEL_NONE
	),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led3))
	BT_MESH_ELEM(
		4, 
		BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&led_ctx[3].srv)),
		BT_MESH_MODEL_NONE
	),
#endif
};

/* Define mesh composition data */
static const struct bt_mesh_comp comp = {
	.cid = CONFIG_BT_COMPANY_ID,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

/* Initialize model handler */
const struct bt_mesh_comp *model_handler_init(void)
{
	int err;
	
	/* Initialize PWM LED control */
	err = pwm_led_init();
	if (err) {
		LOG_ERR("Failed to initialize PWM LED control: %d", err);
	}
	
	/* Initialize attention blink work */
	k_work_init_delayable(&attention_blink_work, attention_blink);
	
	/* Initialize LED transition works */
	for (int i = 0; i < LED_CTX_COUNT; i++) {
		k_work_init_delayable(&led_ctx[i].work, led_work);
		
		/* Initialize LED states */
		led_ctx[i].current_level = 0;
		led_ctx[i].target_level = 0;
		led_ctx[i].remaining_time_ms = 0;
		led_ctx[i].value = false;
		led_ctx[i].is_on = false;
	}
	
	/* Register button handler */
	static struct button_handler button_handler = {
		.cb = button_handler_cb,
	};
	
	dk_button_handler_add(&button_handler);
	
	return &comp;
}
