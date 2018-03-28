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

#include "mali_kbase_rk.h"

/**
 * @file mali_kbase_config_rk.c
 * 对 platform_config_of_rk 的具体实现.
 *
 * mali_device_driver 包含两部分 :
 *      .DP : platform_dependent_part_in_mdd :
 *		依赖 platform 部分,
 *		源码在 <mdd_src_dir>/platform/<platform_name>/
 *		在 mali_device_driver 内部,
 *			记为 platform_dependent_part,
 *			也被记为 platform_specific_code.
 *      .DP : common_parts_in_mdd :
 *		arm 实现的通用的部分,
 *		源码在 <mdd_src_dir>/ 下.
 *		在 mali_device_driver 内部, 记为 common_parts.
 */

/*---------------------------------------------------------------------------*/

static int rk_pm_enable_regulator(struct kbase_device *kbdev);

static void rk_pm_disable_regulator(struct kbase_device *kbdev);

static int rk_pm_enable_clk(struct kbase_device *kbdev);

static void rk_pm_disable_clk(struct kbase_device *kbdev);

/*---------------------------------------------------------------------------*/

static int kbase_platform_rk_init(struct kbase_device *kbdev)
{
	int ret = 0;
	struct rk_context *platform;

	platform = kzalloc(sizeof(*platform), GFP_KERNEL);
	if (!platform) {
		pr_err("err.");
		return -ENOMEM;
	}

	platform->kbdev = kbdev;

	kbdev->platform_context = (void *)platform;
	pm_runtime_enable(kbdev->dev);

EXIT:
	return ret;
}

static void kbase_platform_rk_term(struct kbase_device *kbdev)
{
	struct rk_context *platform =
		(struct rk_context *)kbdev->platform_context;

	pm_runtime_disable(kbdev->dev);
	kbdev->platform_context = NULL;

	if (platform) {
		platform->kbdev = NULL;
		kfree(platform);
	}
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

	err = rk_pm_enable_clk(kbdev);
	if (err) {
		pr_err("failed to enable clk: %d", err);
		return err;
	}

	/* we must enable vdd_gpu before pd_gpu_in_chip. */
	err = rk_pm_enable_regulator(kbdev);
	if (err) {
		pr_err("fail to enable regulator, err : %d.", err);
		return err;
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
	rk_pm_disable_clk(kbdev);

	if (pm_runtime_enabled(kbdev->dev)) {
		pr_debug("to put_sync_suspend mali_dev.");
		pm_runtime_put_sync_suspend(kbdev->dev);
	}

	rk_pm_disable_regulator(kbdev);

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
#ifdef CONFIG_PM
	.power_runtime_init_callback = rk_kbase_device_runtime_init,
	.power_runtime_term_callback = rk_kbase_device_runtime_disable,
	.power_runtime_on_callback = rk_pm_callback_runtime_on,
	.power_runtime_off_callback = rk_pm_callback_runtime_off,
#else // ifdef CONFIG_PM
	.power_runtime_init_callback = NULL,
	.power_runtime_term_callback = NULL,
	.power_runtime_on_callback = NULL,
	.power_runtime_off_callback = NULL,
#endif // ifdef CONFIG_PM
};

int kbase_platform_early_init(void)
{
	/* Nothing needed at this stage */
	return 0;
}

/*---------------------------------------------------------------------------*/

void kbase_platform_rk_shutdown(struct kbase_device *kbdev)
{
	pr_info("to make vdd_gpu enabled for turning off pd_gpu in pm_framework.");
	rk_pm_enable_regulator(kbdev);
}

/*---------------------------------------------------------------------------*/

static int rk_pm_enable_regulator(struct kbase_device *kbdev)
{
	int ret = 0;

	if (!kbdev->regulator) {
		pr_warn("no mali regulator control, no need to enable.");
		goto EXIT;
	}

	pr_debug("to enable regulator.");
	ret = regulator_enable(kbdev->regulator);
	if (ret) {
		pr_err("fail to enable regulator, ret : %d.", ret);
		goto EXIT;
	}

EXIT:
	return ret;
}

static void rk_pm_disable_regulator(struct kbase_device *kbdev)
{
	if (!(kbdev->regulator)) {
		pr_warn("no mali regulator control, no need to disable.");
		return;
	}

	pr_debug("to disable regulator.");
	regulator_disable(kbdev->regulator);
}

static int rk_pm_enable_clk(struct kbase_device *kbdev)
{
	int err = 0;

	if (!(kbdev->clock)) {
		pr_warn("no mali clock control, no need to enable.");
	} else {
		pr_debug("to enable clk.");
		err = clk_enable(kbdev->clock);
		if (err)
			pr_err("failed to enable clk: %d.", err);
	}

	return err;
}

static void rk_pm_disable_clk(struct kbase_device *kbdev)
{
	if (!(kbdev->clock)) {
		pr_warn("no mali clock control, no need to disable.");
	} else {
		pr_debug("to disable clk.");
		clk_disable(kbdev->clock);
	}
}
