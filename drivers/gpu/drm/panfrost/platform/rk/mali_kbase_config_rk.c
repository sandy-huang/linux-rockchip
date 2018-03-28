/*
 *
 * (C) COPYRIGHT ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 */

#include <mali_kbase.h>
#include <mali_kbase_defs.h>
#include <mali_kbase_config.h>
#include <backend/gpu/mali_kbase_pm_internal.h>

#include <linux/pm_runtime.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/delay.h>

static int kbase_platform_rk_init(struct kbase_device *kbdev)
{
	pm_runtime_enable(kbdev->dev);
	return 0;
}

static void kbase_platform_rk_term(struct kbase_device *kbdev)
{
	pm_runtime_disable(kbdev->dev);
}

struct kbase_platform_funcs_conf platform_funcs = {
	.platform_init_func = &kbase_platform_rk_init,
	.platform_term_func = &kbase_platform_rk_term,
};

/*---------------------------------------------------------------------------*/

static int rk_pm_callback_runtime_on(struct kbase_device *kbdev)
{
	return 0;
}

static void rk_pm_callback_runtime_off(struct kbase_device *kbdev)
{
}

static int rk_pm_callback_power_on(struct kbase_device *kbdev)
{
	int ret = 1; /* Assume GPU has been powered off */
	int err = 0;

	err = clk_enable(kbdev->clock);
	if (err) {
		pr_err("failed to enable clk: %d", err);
		return err;
	}

	/* we must enable vdd_gpu before pd_gpu_in_chip. */
	if (kbdev->regulator) {
		err = regulator_enable(kbdev->regulator);
		if (err) {
			pr_err("fail to enable regulator, err : %d.", err);
			return err;
		}
	}

	/* 若 mali_dev 的 runtime_pm 是 enabled 的, 则... */
	if (pm_runtime_enabled(kbdev->dev)) {
		pr_debug("to resume mali_dev syncly.");
		/* 对 pd_in_chip 的 on 操作,
		 * 将在 pm_domain 的 runtime_pm_callbacks 中完成.
		 */
		err = pm_runtime_get_sync(kbdev->dev);
		if (err < 0) {
			pr_err("failed to runtime resume device: %d.", err);
			return err;
		} else if (err == 1) { /* runtime_pm_status is still active */
			pr_debug("chip has NOT been powered off, no need to re-init.");
			ret = 0;
		}
	}

	KBASE_TIMELINE_GPU_POWER(kbdev, 1);

	return ret;
}

static void rk_pm_callback_power_off(struct kbase_device *kbdev)
{
	clk_disable(kbdev->clock);

	if (pm_runtime_enabled(kbdev->dev)) {
		pr_debug("to put_sync_suspend mali_dev.");
		pm_runtime_put_sync_suspend(kbdev->dev);
	}

	if (kbdev->regulator)
		regulator_disable(kbdev->regulator);

	KBASE_TIMELINE_GPU_POWER(kbdev, 0);
}

int rk_kbase_device_runtime_init(struct kbase_device *kbdev)
{
	return 0;
}

void rk_kbase_device_runtime_disable(struct kbase_device *kbdev)
{
}

struct kbase_pm_callback_conf pm_callbacks = {
	.power_on_callback = rk_pm_callback_power_on,
	.power_off_callback = rk_pm_callback_power_off,
	.power_runtime_init_callback = rk_kbase_device_runtime_init,
	.power_runtime_term_callback = rk_kbase_device_runtime_disable,
	.power_runtime_on_callback = rk_pm_callback_runtime_on,
	.power_runtime_off_callback = rk_pm_callback_runtime_off,
};

int kbase_platform_early_init(void)
{
	/* Nothing needed at this stage */
	return 0;
}
