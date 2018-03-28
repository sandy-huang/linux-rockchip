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
