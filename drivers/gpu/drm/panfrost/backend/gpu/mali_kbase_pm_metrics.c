/*
 *
 * (C) COPYRIGHT 2011-2015 ARM Limited. All rights reserved.
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

/*
 * Metrics for power management
 */

#include <mali_kbase.h>
#include <mali_kbase_pm.h>
#include <backend/gpu/mali_kbase_pm_internal.h>
#include <backend/gpu/mali_kbase_jm_rb.h>

/* When VSync is being hit aim for utilisation between 70-90% */
#define KBASE_PM_VSYNC_MIN_UTILISATION          70
#define KBASE_PM_VSYNC_MAX_UTILISATION          90
/* Otherwise aim for 10-40% */
#define KBASE_PM_NO_VSYNC_MIN_UTILISATION       10
#define KBASE_PM_NO_VSYNC_MAX_UTILISATION       40

/* Shift used for kbasep_pm_metrics_data.time_busy/idle - units of (1 << 8) ns
 * This gives a maximum period between samples of 2^(32+8)/100 ns = slightly
 * under 11s. Exceeding this will cause overflow */
#define KBASE_PM_TIME_SHIFT			8

/* Maximum time between sampling of utilization data, without resetting the
 * counters. */
#define MALI_UTILIZATION_MAX_PERIOD 100000 /* ns = 100ms */

int kbasep_pm_metrics_init(struct kbase_device *kbdev)
{
	KBASE_DEBUG_ASSERT(kbdev != NULL);

	kbdev->pm.backend.metrics.kbdev = kbdev;

	kbdev->pm.backend.metrics.time_period_start = ktime_get();
	kbdev->pm.backend.metrics.time_busy = 0;
	kbdev->pm.backend.metrics.time_idle = 0;
	kbdev->pm.backend.metrics.prev_busy = 0;
	kbdev->pm.backend.metrics.prev_idle = 0;
	kbdev->pm.backend.metrics.gpu_active = false;
	kbdev->pm.backend.metrics.active_cl_ctx[0] = 0;
	kbdev->pm.backend.metrics.active_cl_ctx[1] = 0;
	kbdev->pm.backend.metrics.active_gl_ctx[0] = 0;
	kbdev->pm.backend.metrics.active_gl_ctx[1] = 0;
	kbdev->pm.backend.metrics.busy_cl[0] = 0;
	kbdev->pm.backend.metrics.busy_cl[1] = 0;
	kbdev->pm.backend.metrics.busy_gl = 0;

	spin_lock_init(&kbdev->pm.backend.metrics.lock);

	return 0;
}

void kbasep_pm_metrics_term(struct kbase_device *kbdev)
{
}

/* caller needs to hold kbdev->pm.backend.metrics.lock before calling this
 * function
 */
static void kbase_pm_get_dvfs_utilisation_calc(struct kbase_device *kbdev,
								ktime_t now)
{
	ktime_t diff;

	lockdep_assert_held(&kbdev->pm.backend.metrics.lock);

	diff = ktime_sub(now, kbdev->pm.backend.metrics.time_period_start);
	if (ktime_to_ns(diff) < 0)
		return;

	if (kbdev->pm.backend.metrics.gpu_active) {
		u32 ns_time = (u32) (ktime_to_ns(diff) >> KBASE_PM_TIME_SHIFT);

		kbdev->pm.backend.metrics.time_busy += ns_time;
		if (kbdev->pm.backend.metrics.active_cl_ctx[0])
			kbdev->pm.backend.metrics.busy_cl[0] += ns_time;
		if (kbdev->pm.backend.metrics.active_cl_ctx[1])
			kbdev->pm.backend.metrics.busy_cl[1] += ns_time;
		if (kbdev->pm.backend.metrics.active_gl_ctx[0])
			kbdev->pm.backend.metrics.busy_gl += ns_time;
		if (kbdev->pm.backend.metrics.active_gl_ctx[1])
			kbdev->pm.backend.metrics.busy_gl += ns_time;
	} else {
		kbdev->pm.backend.metrics.time_idle += (u32) (ktime_to_ns(diff)
							>> KBASE_PM_TIME_SHIFT);
	}

	kbdev->pm.backend.metrics.time_period_start = now;
}

/**
 * kbase_pm_metrics_active_calc - Update PM active counts based on currently
 *                                running atoms
 * @kbdev: Device pointer
 *
 * The caller must hold kbdev->pm.backend.metrics.lock
 */
static void kbase_pm_metrics_active_calc(struct kbase_device *kbdev)
{
	int js;

	lockdep_assert_held(&kbdev->pm.backend.metrics.lock);

	kbdev->pm.backend.metrics.active_gl_ctx[0] = 0;
	kbdev->pm.backend.metrics.active_gl_ctx[1] = 0;
	kbdev->pm.backend.metrics.active_cl_ctx[0] = 0;
	kbdev->pm.backend.metrics.active_cl_ctx[1] = 0;
	kbdev->pm.backend.metrics.gpu_active = false;

	for (js = 0; js < BASE_JM_MAX_NR_SLOTS; js++) {
		struct kbase_jd_atom *katom = kbase_gpu_inspect(kbdev, js, 0);

		/* Head atom may have just completed, so if it isn't running
		 * then try the next atom */
		if (katom && katom->gpu_rb_state != KBASE_ATOM_GPU_RB_SUBMITTED)
			katom = kbase_gpu_inspect(kbdev, js, 1);

		if (katom && katom->gpu_rb_state ==
				KBASE_ATOM_GPU_RB_SUBMITTED) {
			if (katom->core_req & BASE_JD_REQ_ONLY_COMPUTE) {
				int device_nr = (katom->core_req &
					BASE_JD_REQ_SPECIFIC_COHERENT_GROUP)
						? katom->device_nr : 0;
				WARN_ON(device_nr >= 2);
				kbdev->pm.backend.metrics.active_cl_ctx[
						device_nr] = 1;
			} else {
				/* Slot 2 should not be running non-compute
				 * atoms */
				WARN_ON(js >= 2);
				kbdev->pm.backend.metrics.active_gl_ctx[js] = 1;
			}
			kbdev->pm.backend.metrics.gpu_active = true;
		}
	}
}

/* called when job is submitted to or removed from a GPU slot */
void kbase_pm_metrics_update(struct kbase_device *kbdev, ktime_t *timestamp)
{
	unsigned long flags;
	ktime_t now;

	lockdep_assert_held(&kbdev->js_data.runpool_irq.lock);

	spin_lock_irqsave(&kbdev->pm.backend.metrics.lock, flags);

	if (!timestamp) {
		now = ktime_get();
		timestamp = &now;
	}

	/* Track how long CL and/or GL jobs have been busy for */
	kbase_pm_get_dvfs_utilisation_calc(kbdev, *timestamp);

	kbase_pm_metrics_active_calc(kbdev);

	spin_unlock_irqrestore(&kbdev->pm.backend.metrics.lock, flags);
}
