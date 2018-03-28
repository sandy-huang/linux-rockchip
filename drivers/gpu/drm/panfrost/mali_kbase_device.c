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

/*
 * Base kernel device APIs
 */

#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>

#include <mali_kbase.h>
#include <mali_kbase_defs.h>
#include <mali_kbase_hw.h>
#include <mali_kbase_config_defaults.h>

#define DEBUG_MESSAGE_SIZE 256

struct kbase_device *kbase_device_alloc(void)
{
	return kzalloc(sizeof(struct kbase_device), GFP_KERNEL);
}

static int kbase_device_as_init(struct kbase_device *kbdev, int i)
{
	const char format[] = "mali_mmu%d";
	char name[sizeof(format)];
	const char poke_format[] = "mali_mmu%d_poker";
	char poke_name[sizeof(poke_format)];

	if (kbase_hw_has_issue(kbdev, BASE_HW_ISSUE_8316))
		snprintf(poke_name, sizeof(poke_name), poke_format, i);

	snprintf(name, sizeof(name), format, i);

	kbdev->as[i].number = i;
	kbdev->as[i].fault_addr = 0ULL;

	kbdev->as[i].pf_wq = alloc_workqueue(name, 0, 1);
	if (!kbdev->as[i].pf_wq)
		return -EINVAL;

	mutex_init(&kbdev->as[i].transaction_mutex);
	INIT_WORK(&kbdev->as[i].work_pagefault, page_fault_worker);
	INIT_WORK(&kbdev->as[i].work_busfault, bus_fault_worker);

	if (kbase_hw_has_issue(kbdev, BASE_HW_ISSUE_8316)) {
		struct hrtimer *poke_timer = &kbdev->as[i].poke_timer;
		struct work_struct *poke_work = &kbdev->as[i].poke_work;

		kbdev->as[i].poke_wq = alloc_workqueue(poke_name, 0, 1);
		if (!kbdev->as[i].poke_wq) {
			destroy_workqueue(kbdev->as[i].pf_wq);
			return -EINVAL;
		}
		INIT_WORK(poke_work, kbasep_as_do_poke);

		hrtimer_init(poke_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

		poke_timer->function = kbasep_as_poke_timer_callback;

		kbdev->as[i].poke_refcount = 0;
		kbdev->as[i].poke_state = 0u;
	}

	return 0;
}

static void kbase_device_as_term(struct kbase_device *kbdev, int i)
{
	destroy_workqueue(kbdev->as[i].pf_wq);
	if (kbase_hw_has_issue(kbdev, BASE_HW_ISSUE_8316))
		destroy_workqueue(kbdev->as[i].poke_wq);
}

static int kbase_device_all_as_init(struct kbase_device *kbdev)
{
	int i, err;

	for (i = 0; i < kbdev->nr_hw_address_spaces; i++) {
		err = kbase_device_as_init(kbdev, i);
		if (err)
			goto free_workqs;
	}

	return 0;

free_workqs:
	for (; i > 0; i--)
		kbase_device_as_term(kbdev, i);

	return err;
}

static void kbase_device_all_as_term(struct kbase_device *kbdev)
{
	int i;

	for (i = 0; i < kbdev->nr_hw_address_spaces; i++)
		kbase_device_as_term(kbdev, i);
}

int kbase_device_init(struct kbase_device * const kbdev)
{
	int err;
#ifdef CONFIG_ARM64
	struct device_node *np = NULL;
#endif /* CONFIG_ARM64 */

	spin_lock_init(&kbdev->mmu_mask_change);
#ifdef CONFIG_ARM64
	kbdev->cci_snoop_enabled = false;
	np = kbdev->dev->of_node;
	if (np != NULL) {
		if (of_property_read_u32(np, "snoop_enable_smc",
					&kbdev->snoop_enable_smc))
			kbdev->snoop_enable_smc = 0;
		if (of_property_read_u32(np, "snoop_disable_smc",
					&kbdev->snoop_disable_smc))
			kbdev->snoop_disable_smc = 0;
		/* Either both or none of the calls should be provided. */
		if (!((kbdev->snoop_disable_smc == 0
			&& kbdev->snoop_enable_smc == 0)
			|| (kbdev->snoop_disable_smc != 0
			&& kbdev->snoop_enable_smc != 0))) {
			WARN_ON(1);
			err = -EINVAL;
			goto fail;
		}
	}
#endif /* CONFIG_ARM64 */
	/* Get the list of workarounds for issues on the current HW
	 * (identified by the GPU_ID register)
	 */
	err = kbase_hw_set_issues_mask(kbdev);
	if (err)
		goto fail;

	/* Set the list of features available on the current HW
	 * (identified by the GPU_ID register)
	 */
	kbase_hw_set_features_mask(kbdev);

	kbase_gpuprops_set_features(kbdev);

	/* On Linux 4.0+, dma coherency is determined from device tree */
#ifdef CONFIG_ARM64
	set_dma_ops(kbdev->dev, &noncoherent_swiotlb_dma_ops);
#endif

	/* Workaround a pre-3.13 Linux issue, where dma_mask is NULL when our
	 * device structure was created by device-tree
	 */
	if (!kbdev->dev->dma_mask)
		kbdev->dev->dma_mask = &kbdev->dev->coherent_dma_mask;

	err = dma_set_mask(kbdev->dev,
			DMA_BIT_MASK(kbdev->gpu_props.mmu.pa_bits));
	if (err)
		goto dma_set_mask_failed;

	err = dma_set_coherent_mask(kbdev->dev,
			DMA_BIT_MASK(kbdev->gpu_props.mmu.pa_bits));
	if (err)
		goto dma_set_mask_failed;

	kbdev->nr_hw_address_spaces = kbdev->gpu_props.num_address_spaces;

	err = kbase_device_all_as_init(kbdev);
	if (err)
		goto as_init_failed;

	spin_lock_init(&kbdev->hwcnt.lock);

	mutex_init(&kbdev->cacheclean_lock);

#ifdef CONFIG_MALI_TRACE_TIMELINE
	for (i = 0; i < BASE_JM_MAX_NR_SLOTS; ++i)
		kbdev->timeline.slot_atoms_submitted[i] = 0;

	for (i = 0; i <= KBASEP_TIMELINE_PM_EVENT_LAST; ++i)
		atomic_set(&kbdev->timeline.pm_event_uid[i], 0);
#endif /* CONFIG_MALI_TRACE_TIMELINE */

	atomic_set(&kbdev->ctx_num, 0);

	err = kbase_instr_backend_init(kbdev);
	if (err)
		goto term_as;

	kbdev->pm.dvfs_period = DEFAULT_PM_DVFS_PERIOD;

	kbdev->reset_timeout_ms = DEFAULT_RESET_TIMEOUT_MS;

#ifdef CONFIG_MALI_GPU_MMU_AARCH64
	kbdev->mmu_mode = kbase_mmu_mode_get_aarch64();
#else
	kbdev->mmu_mode = kbase_mmu_mode_get_lpae();
#endif /* CONFIG_MALI_GPU_MMU_AARCH64 */

	return 0;
term_as:
	kbase_device_all_as_term(kbdev);
as_init_failed:
dma_set_mask_failed:
fail:
	return err;
}

void kbase_device_term(struct kbase_device *kbdev)
{
	kbase_instr_backend_term(kbdev);
	kbase_device_all_as_term(kbdev);
}

void kbase_device_free(struct kbase_device *kbdev)
{
	kfree(kbdev);
}
