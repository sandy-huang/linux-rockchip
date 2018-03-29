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

#ifndef _KBASE_H_
#define _KBASE_H_

#include <mali_malisw.h>

#include <asm/page.h>

#include <linux/atomic.h>
#include <linux/highmem.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/mm_types.h>
#include <linux/mutex.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "mali_base_kernel.h"
#include <mali_kbase_uku.h>
#include <mali_kbase_linux.h>

#include "mali_kbase_pm.h"
#include "mali_kbase_mem_lowlevel.h"
#include "mali_kbase_defs.h"
#include "mali_kbase_js.h"
#include "mali_kbase_mem.h"
#include "mali_kbase_utility.h"
#include "mali_kbase_gpuprops.h"
#include "mali_kbase_jm.h"
#include "mali_kbase_vinstr.h"
#include "mali_kbase_ipa.h"

/* Enable GPU reset API */
#define KBASE_GPU_RESET_EN 1

/**
 * @page page_base_kernel_main Kernel-side Base (KBase) APIs
 *
 * The Kernel-side Base (KBase) APIs are divided up as follows:
 * - @subpage page_kbase_js_policy
 */

/**
 * @defgroup base_kbase_api Kernel-side Base (KBase) APIs
 */

struct kbase_device *kbase_device_alloc(void);
/*
* note: configuration attributes member of kbdev needs to have
* been setup before calling kbase_device_init
*/

/*
* API to acquire device list semaphore and return pointer
* to the device list head
*/
const struct list_head *kbase_dev_list_get(void);
/* API to release the device list semaphore */
void kbase_dev_list_put(const struct list_head *dev_list);

int kbase_device_init(struct kbase_device * const kbdev);
void kbase_device_term(struct kbase_device *kbdev);
void kbase_device_free(struct kbase_device *kbdev);
int kbase_device_has_feature(struct kbase_device *kbdev, u32 feature);

/* Needed for gator integration and for reporting vsync information */
struct kbase_device *kbase_find_device(int minor);
void kbase_release_device(struct kbase_device *kbdev);

void kbase_set_profiling_control(struct kbase_device *kbdev, u32 control, u32 value);

u32 kbase_get_profiling_control(struct kbase_device *kbdev, u32 control);

struct kbase_context *
kbase_create_context(struct kbase_device *kbdev, bool is_compat);
void kbase_destroy_context(struct kbase_context *kctx);
int kbase_context_set_create_flags(struct kbase_context *kctx, u32 flags);

int kbase_jd_init(struct kbase_context *kctx);
void kbase_jd_exit(struct kbase_context *kctx);
int kbase_jd_submit(struct kbase_context *kctx,
		const struct kbase_uk_job_submit *submit_data);

/**
 * kbase_jd_done_worker - Handle a job completion
 * @data: a &struct work_struct
 *
 * This function requeues the job from the runpool (if it was soft-stopped or
 * removed from NEXT registers).
 *
 * Removes it from the system if it finished/failed/was cancelled.
 *
 * Resolves dependencies to add dependent jobs to the context, potentially
 * starting them if necessary (which may add more references to the context)
 *
 * Releases the reference to the context from the no-longer-running job.
 *
 * Handles retrying submission outside of IRQ context if it failed from within
 * IRQ context.
 */
void kbase_jd_done_worker(struct work_struct *data);

void kbase_jd_done(struct kbase_jd_atom *katom, int slot_nr, ktime_t *end_timestamp,
		kbasep_js_atom_done_code done_code);
void kbase_jd_cancel(struct kbase_device *kbdev, struct kbase_jd_atom *katom);
void kbase_jd_zap_context(struct kbase_context *kctx);
bool jd_done_nolock(struct kbase_jd_atom *katom,
		struct list_head *completed_jobs_ctx);
void kbase_jd_free_external_resources(struct kbase_jd_atom *katom);
bool jd_submit_atom(struct kbase_context *kctx,
			 const struct base_jd_atom_v2 *user_atom,
			 struct kbase_jd_atom *katom);
void kbase_jd_dep_clear_locked(struct kbase_jd_atom *katom);

void kbase_job_done(struct kbase_device *kbdev, u32 done);

void kbase_gpu_cacheclean(struct kbase_device *kbdev,
					struct kbase_jd_atom *katom);
/**
 * kbase_job_slot_ctx_priority_check_locked(): - Check for lower priority atoms
 *                                               and soft stop them
 * @kctx: Pointer to context to check.
 * @katom: Pointer to priority atom.
 *
 * Atoms from @kctx on the same job slot as @katom, which have lower priority
 * than @katom will be soft stopped and put back in the queue, so that atoms
 * with higher priority can run.
 *
 * The js_data.runpool_irq.lock must be held when calling this function.
 */
void kbase_job_slot_ctx_priority_check_locked(struct kbase_context *kctx,
				struct kbase_jd_atom *katom);

void kbase_job_slot_softstop(struct kbase_device *kbdev, int js,
		struct kbase_jd_atom *target_katom);
void kbase_job_slot_softstop_swflags(struct kbase_device *kbdev, int js,
		struct kbase_jd_atom *target_katom, u32 sw_flags);
void kbase_job_slot_hardstop(struct kbase_context *kctx, int js,
		struct kbase_jd_atom *target_katom);
void kbase_job_check_enter_disjoint(struct kbase_device *kbdev, u32 action,
		u16 core_reqs, struct kbase_jd_atom *target_katom);
void kbase_job_check_leave_disjoint(struct kbase_device *kbdev,
		struct kbase_jd_atom *target_katom);

void kbase_event_post(struct kbase_context *ctx, struct kbase_jd_atom *event);
int kbase_event_dequeue(struct kbase_context *ctx, struct base_jd_event_v2 *uevent);
int kbase_event_pending(struct kbase_context *ctx);
int kbase_event_init(struct kbase_context *kctx);
void kbase_event_close(struct kbase_context *kctx);
void kbase_event_cleanup(struct kbase_context *kctx);
void kbase_event_wakeup(struct kbase_context *kctx);

int kbase_process_soft_job(struct kbase_jd_atom *katom);
int kbase_prepare_soft_job(struct kbase_jd_atom *katom);
void kbase_finish_soft_job(struct kbase_jd_atom *katom);
void kbase_cancel_soft_job(struct kbase_jd_atom *katom);
void kbase_resume_suspended_soft_jobs(struct kbase_device *kbdev);
void kbasep_add_waiting_soft_job(struct kbase_jd_atom *katom);

bool kbase_replay_process(struct kbase_jd_atom *katom);

enum hrtimer_restart kbasep_soft_event_timeout_worker(struct hrtimer *timer);
void kbasep_complete_triggered_soft_events(struct kbase_context *kctx, u64 evt);
int kbasep_read_soft_event_status(
		struct kbase_context *kctx, u64 evt, unsigned char *status);
int kbasep_write_soft_event_status(
		struct kbase_context *kctx, u64 evt, unsigned char new_status);

void kbasep_as_do_poke(struct work_struct *work);

/** Returns the name associated with a Mali exception code
 *
 * This function is called from the interrupt handler when a GPU fault occurs.
 *
 * @param[in] kbdev     The kbase device that the GPU fault occurred from.
 * @param[in] exception_code  exception code
 * @return name associated with the exception code
 */
const char *kbase_exception_name(struct kbase_device *kbdev,
		u32 exception_code);

/**
 * Check whether a system suspend is in progress, or has already been suspended
 *
 * The caller should ensure that either kbdev->pm.active_count_lock is held, or
 * a dmb was executed recently (to ensure the value is most
 * up-to-date). However, without a lock the value could change afterwards.
 *
 * @return false if a suspend is not in progress
 * @return !=false otherwise
 */
static inline bool kbase_pm_is_suspending(struct kbase_device *kbdev)
{
	return kbdev->pm.suspending;
}

/**
 * Return the atom's ID, as was originally supplied by userspace in
 * base_jd_atom_v2::atom_number
 */
static inline int kbase_jd_atom_id(struct kbase_context *kctx, struct kbase_jd_atom *katom)
{
	return katom - &kctx->jctx.atoms[0];
}

/**
 * kbase_jd_atom_from_id - Return the atom structure for the given atom ID
 * @kctx: Context pointer
 * @id:   ID of atom to retrieve
 *
 * Return: Pointer to struct kbase_jd_atom associated with the supplied ID
 */
static inline struct kbase_jd_atom *kbase_jd_atom_from_id(
		struct kbase_context *kctx, int id)
{
	return &kctx->jctx.atoms[id];
}

/**
 * Initialize the disjoint state
 *
 * The disjoint event count and state are both set to zero.
 *
 * Disjoint functions usage:
 *
 * The disjoint event count should be incremented whenever a disjoint event occurs.
 *
 * There are several cases which are regarded as disjoint behavior. Rather than just increment
 * the counter during disjoint events we also increment the counter when jobs may be affected
 * by what the GPU is currently doing. To facilitate this we have the concept of disjoint state.
 *
 * Disjoint state is entered during GPU reset and for the entire time that an atom is replaying
 * (as part of the replay workaround). Increasing the disjoint state also increases the count of
 * disjoint events.
 *
 * The disjoint state is then used to increase the count of disjoint events during job submission
 * and job completion. Any atom submitted or completed while the disjoint state is greater than
 * zero is regarded as a disjoint event.
 *
 * The disjoint event counter is also incremented immediately whenever a job is soft stopped
 * and during context creation.
 *
 * @param kbdev The kbase device
 */
void kbase_disjoint_init(struct kbase_device *kbdev);

/**
 * Increase the count of disjoint events
 * called when a disjoint event has happened
 *
 * @param kbdev The kbase device
 */
void kbase_disjoint_event(struct kbase_device *kbdev);

/**
 * Increase the count of disjoint events only if the GPU is in a disjoint state
 *
 * This should be called when something happens which could be disjoint if the GPU
 * is in a disjoint state. The state refcount keeps track of this.
 *
 * @param kbdev The kbase device
 */
void kbase_disjoint_event_potential(struct kbase_device *kbdev);

/**
 * Returns the count of disjoint events
 *
 * @param kbdev The kbase device
 * @return the count of disjoint events
 */
u32 kbase_disjoint_event_get(struct kbase_device *kbdev);

/**
 * Increment the refcount state indicating that the GPU is in a disjoint state.
 *
 * Also Increment the disjoint event count (calls @ref kbase_disjoint_event)
 * eventually after the disjoint state has completed @ref kbase_disjoint_state_down
 * should be called
 *
 * @param kbdev The kbase device
 */
void kbase_disjoint_state_up(struct kbase_device *kbdev);

/**
 * Decrement the refcount state
 *
 * Also Increment the disjoint event count (calls @ref kbase_disjoint_event)
 *
 * Called after @ref kbase_disjoint_state_up once the disjoint state is over
 *
 * @param kbdev The kbase device
 */
void kbase_disjoint_state_down(struct kbase_device *kbdev);

/**
 * If a job is soft stopped and the number of contexts is >= this value
 * it is reported as a disjoint event
 */
#define KBASE_DISJOINT_STATE_INTERLEAVED_CONTEXT_COUNT_THRESHOLD 2

#ifndef UINT64_MAX
	#define UINT64_MAX ((uint64_t)0xFFFFFFFFFFFFFFFFULL)
#endif

#endif // ifndef _KBASE_H_
