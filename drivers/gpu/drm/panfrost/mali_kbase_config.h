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

#include <linux/rbtree.h>

/* Forward declaration of struct kbase_device */
struct kbase_device;

/*
 * @brief Specifies the callbacks for power management
 *
 * By default no callbacks will be made and the GPU must not be powered off.
 */
struct kbase_pm_callback_conf {
	/** Callback for when the GPU is idle and the power to it can be switched off.
	 *
	 * The system integrator can decide whether to either do nothing, just switch off
	 * the clocks to the GPU, or to completely power down the GPU.
	 * The platform specific private pointer kbase_device::platform_context can be accessed and modified in here. It is the
	 * platform \em callbacks responsibility to initialize and terminate this pointer if used.
	 */
	void (*power_off_callback)(struct kbase_device *kbdev);

	/** Callback for when the GPU is about to become active and power must be supplied.
	 *
	 * This function must not return until the GPU is powered and clocked sufficiently for register access to
	 * succeed.  The return value specifies whether the GPU was powered down since the call to power_off_callback.
	 * If the GPU state has been lost then this function must return 1, otherwise it should return 0.
	 * The platform specific private pointer kbase_device::platform_context can be accessed and modified in here. It is the
	 * platform \em callbacks responsibility to initialize and terminate this pointer if used.
	 *
	 * The return value of the first call to this function is ignored.
	 *
	 * @return 1 if the GPU state may have been lost, 0 otherwise.
	 */
	int (*power_on_callback)(struct kbase_device *kbdev);

	/** Callback for when the system is requesting a suspend and GPU power
	 * must be switched off.
	 *
	 * Note that if this callback is present, then this may be called
	 * without a preceding call to power_off_callback. Therefore this
	 * callback must be able to take any action that might otherwise happen
	 * in power_off_callback.
	 *
	 * The platform specific private pointer kbase_device::platform_context
	 * can be accessed and modified in here. It is the platform \em
	 * callbacks responsibility to initialize and terminate this pointer if
	 * used.
	 */
	void (*power_suspend_callback)(struct kbase_device *kbdev);

	/** Callback for when the system is resuming from a suspend and GPU
	 * power must be switched on.
	 *
	 * Note that if this callback is present, then this may be called
	 * without a following call to power_on_callback. Therefore this
	 * callback must be able to take any action that might otherwise happen
	 * in power_on_callback.
	 *
	 * The platform specific private pointer kbase_device::platform_context
	 * can be accessed and modified in here. It is the platform \em
	 * callbacks responsibility to initialize and terminate this pointer if
	 * used.
	 */
	void (*power_resume_callback)(struct kbase_device *kbdev);

	/** Callback for handling runtime power management initialization.
	 *
	 * The runtime power management callbacks @ref power_runtime_off_callback and @ref power_runtime_on_callback
	 * will become active from calls made to the OS from within this function.
	 * The runtime calls can be triggered by calls from @ref power_off_callback and @ref power_on_callback.
	 * Note: for linux the kernel must have CONFIG_PM_RUNTIME enabled to use this feature.
	 *
	 * @return 0 on success, else int error code.
	 */
	 int (*power_runtime_init_callback)(struct kbase_device *kbdev);

	/** Callback for handling runtime power management termination.
	 *
	 * The runtime power management callbacks @ref power_runtime_off_callback and @ref power_runtime_on_callback
	 * should no longer be called by the OS on completion of this function.
	 * Note: for linux the kernel must have CONFIG_PM_RUNTIME enabled to use this feature.
	 */
	void (*power_runtime_term_callback)(struct kbase_device *kbdev);

	/** Callback for runtime power-off power management callback
	 *
	 * For linux this callback will be called by the kernel runtime_suspend callback.
	 * Note: for linux the kernel must have CONFIG_PM_RUNTIME enabled to use this feature.
	 *
	 * @return 0 on success, else OS error code.
	 */
	void (*power_runtime_off_callback)(struct kbase_device *kbdev);

	/** Callback for runtime power-on power management callback
	 *
	 * For linux this callback will be called by the kernel runtime_resume callback.
	 * Note: for linux the kernel must have CONFIG_PM_RUNTIME enabled to use this feature.
	 */
	int (*power_runtime_on_callback)(struct kbase_device *kbdev);

	/*
	 * Optional callback for checking if GPU can be suspended when idle
	 *
	 * This callback will be called by the runtime power management core
	 * when the reference count goes to 0 to provide notification that the
	 * GPU now seems idle.
	 *
	 * If this callback finds that the GPU can't be powered off, or handles
	 * suspend by powering off directly or queueing up a power off, a
	 * non-zero value must be returned to prevent the runtime PM core from
	 * also triggering a suspend.
	 *
	 * Returning 0 will cause the runtime PM core to conduct a regular
	 * autosuspend.
	 *
	 * This callback is optional and if not provided regular autosuspend
	 * will be triggered.
	 *
	 * Note: The Linux kernel must have CONFIG_PM_RUNTIME enabled to use
	 * this feature.
	 *
	 * Return 0 if GPU can be suspended, positive value if it can not be
	 * suspeneded by runtime PM, else OS error code
	 */
	int (*power_runtime_idle_callback)(struct kbase_device *kbdev);
};

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

#ifdef CONFIG_OF
struct kbase_platform_config {
};
#else

/*
 * @brief Specifies start and end of I/O memory region.
 */
struct kbase_io_memory_region {
	u64 start;
	u64 end;
};

/*
 * @brief Specifies I/O related resources like IRQs and memory region for I/O operations.
 */
struct kbase_io_resources {
	u32                      job_irq_number;
	u32                      mmu_irq_number;
	u32                      gpu_irq_number;
	struct kbase_io_memory_region io_memory_region;
};

struct kbase_platform_config {
	const struct kbase_io_resources *io_resources;
};

#endif // ifdef CONFIG_OF

/**
 * @brief Gets the pointer to platform config.
 *
 * @return Pointer to the platform config
 */
struct kbase_platform_config *kbase_get_platform_config(void);

#endif
