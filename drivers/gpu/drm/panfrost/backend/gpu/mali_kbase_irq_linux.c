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

#include <mali_kbase.h>
#include <backend/gpu/mali_kbase_device_internal.h>
#include <backend/gpu/mali_kbase_irq_internal.h>

#include <linux/interrupt.h>

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

int kbase_install_interrupts(struct kbase_device *kbdev)
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

void kbase_release_interrupts(struct kbase_device *kbdev)
{
	free_irq(kbdev->irq_job, kbdev);
	free_irq(kbdev->irq_gpu, kbdev);
	free_irq(kbdev->irq_mmu, kbdev);
}

void kbase_synchronize_irqs(struct kbase_device *kbdev)
{
	synchronize_irq(kbdev->irq_job);
	synchronize_irq(kbdev->irq_gpu);
	synchronize_irq(kbdev->irq_mmu);
}
