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
#include <linux/interrupt.h>

/*
 * Register-based HW access backend APIs
 */
#include <mali_kbase.h>
#include <mali_kbase_hwaccess_jm.h>
#include <mali_kbase_hwaccess_backend.h>
#include <backend/gpu/mali_kbase_device_internal.h>
#include <backend/gpu/mali_kbase_jm_internal.h>
#include <backend/gpu/mali_kbase_js_internal.h>
#include <backend/gpu/mali_kbase_pm_internal.h>

static irqreturn_t kbase_job_irq_handler(int irq, void *data)
{
	unsigned long flags;
	struct kbase_device *kbdev = data;
	u32 val;

	spin_lock_irqsave(&kbdev->pm.backend.gpu_powered_lock, flags);

	if (!kbdev->pm.backend.gpu_powered) {
		/* GPU is turned off - IRQ is not for us */
		spin_unlock_irqrestore(&kbdev->pm.backend.gpu_powered_lock,
									flags);
		return IRQ_NONE;
	}

	val = kbase_reg_read(kbdev, JOB_CONTROL_REG(JOB_IRQ_STATUS), NULL);

	spin_unlock_irqrestore(&kbdev->pm.backend.gpu_powered_lock, flags);

	if (!val)
		return IRQ_NONE;

	dev_dbg(kbdev->dev, "%s: irq %d irqstatus 0x%x\n", __func__, irq, val);

	kbase_job_done(kbdev, val);

	return IRQ_HANDLED;
}

static irqreturn_t kbase_mmu_irq_handler(int irq, void *data)
{
	unsigned long flags;
	struct kbase_device *kbdev = data;
	u32 val;

	spin_lock_irqsave(&kbdev->pm.backend.gpu_powered_lock, flags);

	if (!kbdev->pm.backend.gpu_powered) {
		/* GPU is turned off - IRQ is not for us */
		spin_unlock_irqrestore(&kbdev->pm.backend.gpu_powered_lock,
									flags);
		return IRQ_NONE;
	}

	atomic_inc(&kbdev->faults_pending);

	val = kbase_reg_read(kbdev, MMU_REG(MMU_IRQ_STATUS), NULL);

	spin_unlock_irqrestore(&kbdev->pm.backend.gpu_powered_lock, flags);

	if (!val) {
		atomic_dec(&kbdev->faults_pending);
		return IRQ_NONE;
	}

	dev_dbg(kbdev->dev, "%s: irq %d irqstatus 0x%x\n", __func__, irq, val);

	kbase_mmu_interrupt(kbdev, val);

	atomic_dec(&kbdev->faults_pending);

	return IRQ_HANDLED;
}

static irqreturn_t kbase_gpu_irq_handler(int irq, void *data)
{
	unsigned long flags;
	struct kbase_device *kbdev = data;
	u32 val;

	spin_lock_irqsave(&kbdev->pm.backend.gpu_powered_lock, flags);

	if (!kbdev->pm.backend.gpu_powered) {
		/* GPU is turned off - IRQ is not for us */
		spin_unlock_irqrestore(&kbdev->pm.backend.gpu_powered_lock,
									flags);
		return IRQ_NONE;
	}

	val = kbase_reg_read(kbdev, GPU_CONTROL_REG(GPU_IRQ_STATUS), NULL);

	spin_unlock_irqrestore(&kbdev->pm.backend.gpu_powered_lock, flags);

	if (!val)
		return IRQ_NONE;

	dev_dbg(kbdev->dev, "%s: irq %d irqstatus 0x%x\n", __func__, irq, val);

	kbase_gpu_interrupt(kbdev, val);

	return IRQ_HANDLED;
}

static int kbase_install_interrupts(struct kbase_device *kbdev)
{
	int err;

	err = request_irq(kbdev->irq_job, kbase_job_irq_handler,
				kbdev->irq_job_flags | IRQF_SHARED,
				dev_name(kbdev->dev), kbdev);
	if (err) {
		dev_err(kbdev->dev, "Can't request job interrupt\n");
		return err;
	}

	err = request_irq(kbdev->irq_gpu, kbase_gpu_irq_handler,
				kbdev->irq_gpu_flags | IRQF_SHARED,
				dev_name(kbdev->dev), kbdev);
	if (err) {
		dev_err(kbdev->dev, "Can't request gpu interrupt\n");
		goto release_job;
	}

	err = request_irq(kbdev->irq_mmu, kbase_mmu_irq_handler,
				kbdev->irq_mmu_flags | IRQF_SHARED,
				dev_name(kbdev->dev), kbdev);
	if (err) {
		dev_err(kbdev->dev, "Can't request mmu interrupt\n");
		goto release_gpu;
	}

	return 0;

release_gpu:
	free_irq(kbdev->irq_gpu, kbdev);
release_job:
	free_irq(kbdev->irq_job, kbdev);

	return err;
}

static void kbase_release_interrupts(struct kbase_device *kbdev)
{
	free_irq(kbdev->irq_job, kbdev);
	free_irq(kbdev->irq_gpu, kbdev);
	free_irq(kbdev->irq_mmu, kbdev);
}

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

