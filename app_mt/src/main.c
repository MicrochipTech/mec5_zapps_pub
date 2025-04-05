/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <soc.h>
#include <device_mec5.h>
#include <mec_retval.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/zbus/zbus.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#include <app_version.h>

#ifdef CONFIG_USE_APP_THREAD1
#include "app_thread1.h"
#endif
#ifdef CONFIG_USE_APP_THREAD2
#include "app_thread2.h"
#endif

struct version_msg {
	uint8_t major;
	uint8_t minor;
	uint16_t build;
};

struct acc_msg {
	int x, y, z;
};

/* ---- ZBUS ---- */
ZBUS_CHAN_DEFINE(version_chan,		/* Name */
		struct version_msg,	/* Message type */
		NULL,			/* validator */
		NULL,			/* user data */
		ZBUS_OBSERVERS_EMPTY,	/* observers */
		ZBUS_MSG_INIT(.major = 0, .minor = 1, .build = 2)
);

ZBUS_CHAN_DEFINE(acc_data_chan,		/* Name */
		struct acc_msg,		/* Message type */
		NULL,			/* validator */
		NULL,			/* user data */
		ZBUS_OBSERVERS(foo_lis, bar_sub),	/* observers */
		ZBUS_MSG_INIT(.x = 0, .y = 1, .z = 0)
);

bool simple_chan_validator(const void *msg, size_t msg_size)
{
	ARG_UNUSED(msg_size);

	const int *simple = msg;

	if ((*simple >= 0) && (*simple < 10)) {
		return true;
	}

	return false;
}

ZBUS_CHAN_DEFINE(simple_chan, /* Name */
		 int,         /* Message type */

		 simple_chan_validator, /* Validator */
		 NULL,                  /* User data */
		 ZBUS_OBSERVERS_EMPTY,  /* observers */
		 0                      /* Initial value is 0 */
);

static void listener_callback_example(const struct zbus_channel *chan)
{
	const struct acc_msg *acc = zbus_chan_const_msg(chan);

	LOG_INF("From listener -> Acc x=%d, y=%d, z=%d", acc->x, acc->y, acc->z);
}

ZBUS_LISTENER_DEFINE(foo_lis, listener_callback_example);
ZBUS_SUBSCRIBER_DEFINE(bar_sub, 4);

static void subscriber_task(void)
{
	const struct zbus_channel *chan;

	while (!zbus_sub_wait(&bar_sub, &chan, K_FOREVER)) {
		struct acc_msg acc;

		if (&acc_data_chan == chan) {
			zbus_chan_read(&acc_data_chan, &acc, K_MSEC(500));

			LOG_INF("From subscriber -> Acc x=%d, y=%d, z=%d", acc.x, acc.y, acc.z);
		}
	}
}

K_THREAD_DEFINE(subscriber_task_id, CONFIG_MAIN_STACK_SIZE, subscriber_task, NULL, NULL, NULL, 3, 0,
		0);

static bool print_channel_data_iterator(const struct zbus_channel *chan, void *user_data)
{
	int *count = user_data;

	LOG_INF("%d - Channel %s:", *count, zbus_chan_name(chan));
	LOG_INF("      Message size: %d", zbus_chan_msg_size(chan));
	LOG_INF("      Observers:");

	++(*count);

	struct zbus_channel_observation *observation;

	for (int16_t i = chan->data->observers_start_idx, limit = chan->data->observers_end_idx;
	     i < limit; ++i) {
		STRUCT_SECTION_GET(zbus_channel_observation, i, &observation);

		__ASSERT(observation != NULL, "observation must be not NULL");

		LOG_INF("      - %s", observation->obs->name);
	}

	struct zbus_observer_node *obs_nd, *tmp;

	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&chan->data->observers, obs_nd, tmp, node) {
		LOG_INF("      - %s", obs_nd->obs->name);
	}

	return true;
}

static bool print_observer_data_iterator(const struct zbus_observer *obs, void *user_data)
{
	int *count = user_data;

	LOG_INF("%d - %s %s", *count,
		obs->type == ZBUS_OBSERVER_LISTENER_TYPE ? "Listener" : "Subscriber",
		zbus_obs_name(obs));

	++(*count);

	return true;
}

/* ---- Global variables ---- */
static volatile uint32_t spin_val;
static volatile int ret_val;

/* ---- prototypes ---- */
static void spin_on(uint32_t id, int rval);

static int zbus_test1(void);

int main(void)
{
	int ret = 0;

	LOG_INF("app_zbus sample: board: %s", DT_N_P_compatible_IDX_0);

	zbus_test1();

#ifdef CONFIG_USE_APP_THREAD1
	ret = app_thread1_create(0);
	LOG_INF("Thread 1 create returned (%d)", ret);
	if (ret == 0) {
		LOG_INF("Thread 1 TID = %p", app_thread1_get_tid());
	}
#endif
#ifdef CONFIG_USE_APP_THREAD2
	/* Create thread but don't allow it start. Zephyr kernel will not put this
	 * thread in its to run queue until k_thread_start(k_tid_t thread_id) is called.
	 */
	ret = app_thread2_create(UINT32_MAX);
	LOG_INF("Thread 2 create returned (%d)", ret);
	if (ret == 0) {
		LOG_INF("Thread 2 TID = %p", app_thread2_get_tid());
	}
#endif

	LOG_INF("Application Done (%d)", ret);
	spin_on((uint32_t)__LINE__, 0);

	return 0;
}

static void spin_on(uint32_t id, int rval)
{
	spin_val = id;
	ret_val = rval;

	log_panic(); /* flush log buffers */

	while (spin_val) {
		k_sleep(K_MSEC(5000));
	}
}

static int zbus_test1(void)
{
	int err =0, value = 0, count = 0;
	struct acc_msg acc1 = {.x = 1, .y = 1, .z = 1};
	const struct version_msg *v = zbus_chan_const_msg(&version_chan);

	LOG_INF("Sensor sample started raw reading, version %u.%u-%u!", v->major, v->minor,
		v->build);

	LOG_INF("Channel list:");
	zbus_iterate_over_channels_with_user_data(print_channel_data_iterator, &count);

	count = 0;

	LOG_INF("Observers list:");
	zbus_iterate_over_observers_with_user_data(print_observer_data_iterator, &count);
	zbus_chan_pub(&acc_data_chan, &acc1, K_SECONDS(1));

	k_msleep(1000);

	acc1.x = 2;
	acc1.y = 2;
	acc1.z = 2;
	zbus_chan_pub(&acc_data_chan, &(acc1), K_SECONDS(1));

	value = 5;
	err = zbus_chan_pub(&simple_chan, &value, K_MSEC(200));

	if (err == 0) {
		LOG_INF("Pub a valid value to a channel with validator successfully.");
	}

	value = 15;
	err = zbus_chan_pub(&simple_chan, &value, K_MSEC(200));

	if (err == -ENOMSG) {
		LOG_INF("Pub an invalid value to a channel with validator successfully.");
	}

	return 0;
}
