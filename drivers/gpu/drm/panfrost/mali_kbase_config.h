/*
 *
 * (C) COPYRIGHT 2010-2016 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained
 * from Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 */

/**
 * @file mali_kbase_config.h
 * Configuration API and Attributes for KBase
 */

#ifndef _KBASE_CONFIG_H_
#define _KBASE_CONFIG_H_

#include <asm/page.h>

#include <mali_malisw.h>
#include <mali_kbase_backend_config.h>

/**
 * @addtogroup base_api
 * @{
 */

/**
 * @addtogroup base_kbase_api
 * @{
 */

/**
 * @addtogroup kbase_config Configuration API and Attributes
 * @{
 */

/**
 * kbase_cpu_clk_speed_func - Type of the function pointer for CPU_SPEED_FUNC
 * @param clock_speed - pointer to store the current CPU clock speed in MHz
 *
 * Returns 0 on success, otherwise negative error code.
 *
 * This is mainly used to implement OpenCL's clGetDeviceInfo().
 */
typedef int (*kbase_cpu_clk_speed_func) (u32 *clock_speed);

/**
 * kbase_gpu_clk_speed_func - Type of the function pointer for GPU_SPEED_FUNC
 * @param clock_speed - pointer to store the current GPU clock speed in MHz
 *
 * Returns 0 on success, otherwise negative error code.
 * When an error is returned the caller assumes maximum GPU speed stored in
 * gpu_freq_khz_max.
 *
 * If the system timer is not available then this function is required
 * for the OpenCL queue profiling to return correct timing information.
 *
 */
typedef int (*kbase_gpu_clk_speed_func) (u32 *clock_speed);

#endif
