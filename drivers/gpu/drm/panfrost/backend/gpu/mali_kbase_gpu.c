/*
 *
 * (C) COPYRIGHT 2014-2016 ARM Limited. All rights reserved.
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

#include <linux/pm_runtime.h>

/*
 * Register-based HW access backend APIs
 */
#include <mali_kbase.h>
#include <mali_kbase_hwaccess_jm.h>
#include <mali_kbase_hwaccess_backend.h>
#include <backend/gpu/mali_kbase_irq_internal.h>
#include <backend/gpu/mali_kbase_jm_internal.h>
#include <backend/gpu/mali_kbase_js_internal.h>
#include <backend/gpu/mali_kbase_pm_internal.h>

int kbase_backend_early_init(struct kbase_device *kbdev)
{
	int err;

	pm_runtime_enable(kbdev->dev);

	/* Ensure we can access the GPU registers */
	kbase_pm_register_access_enable(kbdev);

	/* Find out GPU properties based on the GPU feature registers */
	kbase_gpuprops_set(kbdev);

	/* We're done accessing the GPU registers for now. */
	kbase_pm_register_access_disable(kbdev);

	err = kbase_hwaccess_pm_init(kbdev);
	if (err)
		goto fail_pm;

	err = kbase_install_interrupts(kbdev);
	if (err)
		goto fail_interrupts;

	return 0;

fail_interrupts:
	kbase_hwaccess_pm_term(kbdev);
fail_pm:
	pm_runtime_disable(kbdev->dev);

	return err;
}

void kbase_backend_early_term(struct kbase_device *kbdev)
{
	kbase_release_interrupts(kbdev);
	kbase_hwaccess_pm_term(kbdev);
	pm_runtime_disable(kbdev->dev);
}

int kbase_backend_late_init(struct kbase_device *kbdev)
{
	int err;

	err = kbase_hwaccess_pm_powerup(kbdev, PM_HW_ISSUES_DETECT);
	if (err)
		return err;

	err = kbase_backend_timer_init(kbdev);
	if (err)
		goto fail_timer;

	err = kbase_job_slot_init(kbdev);
	if (err)
		goto fail_job_slot;

	init_waitqueue_head(&kbdev->hwaccess.backend.reset_wait);

	return 0;

fail_job_slot:
	kbase_backend_timer_term(kbdev);
fail_timer:
	kbase_hwaccess_pm_halt(kbdev);

	return err;
}

void kbase_backend_late_term(struct kbase_device *kbdev)
{
	kbase_job_slot_halt(kbdev);
	kbase_job_slot_term(kbdev);
	kbase_backend_timer_term(kbdev);
	kbase_hwaccess_pm_halt(kbdev);
}

