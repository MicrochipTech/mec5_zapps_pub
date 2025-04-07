/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <soc.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/zbus/zbus.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#include <device_mec5.h>
#include <mec_retval.h>

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define PWM_NODE DT_ALIAS(pwm_0)
#define TACH_CCT_NODE DT_ALIAS(tach_cct)

static const struct device *pwm_dev = DEVICE_DT_GET(PWM_NODE);
static const struct device *tach_cct_dev = DEVICE_DT_GET(TACH_CCT_NODE);

/* ---- Global variables ---- */
static volatile uint32_t spin_val;
static volatile int ret_val;

/* ---- prototypes ---- */
static void spin_on(uint32_t id, int rval);
static void pr_sensor_value(struct sensor_value *sval);

/* Delay for EVB PWM circuit to ramp up/down to new value */
#define PWM_RAMP_DELAY K_MSEC(2000)

/* MCHP PWM implements a 16-bit counter */
#define PWM_PERIOD_CYCLES 64000u

static uint32_t pulse_width_cycles_tbl[] = {
	12800u, /* 20% duty cycle */
	16000u, /* 25% */
	19200u, /* 30% */
	25600u, /* 40% */
	32000u, /* 50% */
	38400u, /* 60% */
	44800u, /* 70% */
	51200u, /* 80% */
	57600u, /* 90% */
	60800u, /* 95% */
	64000u, /* 100% */
};

int main(void)
{
	uint64_t pwm_cyc_per_sec = 0;
	uint32_t period_cycles = 0, pulse_width_cycles = 0, duty_cycle = 0;
	struct sensor_value sval = {0};
	int ret = 0, direction = 1;
	size_t n = 0, busy_count = 0;

	LOG_INF("app_tach_icct sample: board: %s", DT_N_P_compatible_IDX_0);

	if (!device_is_ready(pwm_dev)) {
		LOG_ERR("PWM device is not ready!");
		return -EIO;
	}

	if (!device_is_ready(tach_cct_dev)) {
		LOG_ERR("TACH CCT device is not ready!");
		return -EIO;
	}

	ret = pwm_get_cycles_per_sec(pwm_dev, 0, &pwm_cyc_per_sec);
	if (ret) {
		LOG_ERR("PWM driver error getting cycles per second (%d)", ret);
		return -EIO;
	}

	LOG_INF("PWM cycles per second = %llu", pwm_cyc_per_sec); /* 48,000,000 */

	period_cycles = PWM_PERIOD_CYCLES;
	while (1) {
		pulse_width_cycles = pulse_width_cycles_tbl[n];
		duty_cycle = (pulse_width_cycles * 100u) / period_cycles;
		LOG_INF("Set period cycles = %u, pulse width cycles = %u  duty_cycle = %u",
			period_cycles, pulse_width_cycles, duty_cycle);

		ret = pwm_set_cycles(pwm_dev, 0, period_cycles, pulse_width_cycles, 0);
		if (ret) {
			LOG_ERR("PWM driver set cycles returned error (%d)", ret);
			return -EIO;
		}

		LOG_INF("Delay for board circuit to ramp to final value");
		k_sleep(PWM_RAMP_DELAY);

		/* blocking routine triggering HW to measure all supported sensors
		 * or we can specify a single sensor blocking only for that sensor:
		 * int sensor_sample_fetch_chan(const struct device *dev, enum sensor_channel type)
		 */
		/* ret = sensor_sample_fetch(tach_cct_dev); */
		ret = sensor_sample_fetch_chan(tach_cct_dev, SENSOR_CHAN_RPM);
		if (ret) {
			LOG_ERR("Sensor fetch API returned error (%d)", ret);
			return ret;
		}

		/* blocking routine trigger HW to measure specified sensor type */
		busy_count = 0;
		do {
			ret = sensor_channel_get(tach_cct_dev, SENSOR_CHAN_RPM, &sval);
			if (ret == 0) {
				break;
			} else if (ret == -EBUSY) {
				busy_count++;
			} else {
				LOG_ERR("Sensor chan get API returned error (%d)", ret);
				break;
			}
		} while (ret == -EBUSY);

		if (busy_count) {
			LOG_INF("Sensor channel get busy count = %u", busy_count);
		}

		pr_sensor_value(&sval);

		if (direction) {
			n++;
			if (n >= ARRAY_SIZE(pulse_width_cycles_tbl)) {
				direction = 0;
				n = ARRAY_SIZE(pulse_width_cycles_tbl) - 1u;
				if (n) {
					n--;
				}
			}
		} else {
			if (n) {
				n--;
			} else {
				direction = 1u;
				n++;
			}
		}
	}

	LOG_INF("Application Done (%d)", ret);
	while (1) {
		k_sleep(K_MSEC(5000));
		LOG_INF("Wake: main");
	}

	spin_on((uint32_t)__LINE__, 0);

	return 0;
}

static void pr_sensor_value(struct sensor_value *sval)
{
	if (!sval) {
		return;
	}

	LOG_INF("Sensor value: v1 = %d(0x%0x),  v2 = %d(0x%0x)", sval->val1, sval->val1,
								 sval->val2, sval->val2);
}

static void spin_on(uint32_t id, int rval)
{
	spin_val = id;
	ret_val = rval;

	log_panic(); /* flush log buffers */

	while (spin_val) {
		;
	}
}
