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

/* GPU IRQ Tags */
#define	JOB_IRQ_TAG	0
#define MMU_IRQ_TAG	1
#define GPU_IRQ_TAG	2

static void *kbase_tag(void *ptr, u32 tag)
{
	return (void *)(((uintptr_t) ptr) | tag);
}

static void *kbase_untag(void *ptr)
{
	return (void *)(((uintptr_t) ptr) & ~3);
}

static irqreturn_t kbase_job_irq_handler(int irq, void *data)
{
	unsigned long flags;
	struct kbase_device *kbdev = kbase_untag(data);
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
	struct kbase_device *kbdev = kbase_untag(data);
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
	struct kbase_device *kbdev = kbase_untag(data);
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

static irq_handler_t kbase_handler_table[] = {
	[JOB_IRQ_TAG] = kbase_job_irq_handler,
	[MMU_IRQ_TAG] = kbase_mmu_irq_handler,
	[GPU_IRQ_TAG] = kbase_gpu_irq_handler,
};

int kbase_install_interrupts(struct kbase_device *kbdev)
{
	u32 nr = ARRAY_SIZE(kbase_handler_table);
	int err;
	u32 i;

	for (i = 0; i < nr; i++) {
		err = request_irq(kbdev->irqs[i].irq, kbase_handler_table[i],
				kbdev->irqs[i].flags | IRQF_SHARED,
				dev_name(kbdev->dev),
				kbase_tag(kbdev, i));
		if (err) {
			dev_err(kbdev->dev, "Can't request interrupt %d (index %d)\n",
							kbdev->irqs[i].irq, i);
#ifdef CONFIG_SPARSE_IRQ
			dev_err(kbdev->dev, "You have CONFIG_SPARSE_IRQ support enabled - is the interrupt number correct for this configuration?\n");
#endif
			goto release;
		}
	}

	return 0;

 release:
	while (i-- > 0)
		free_irq(kbdev->irqs[i].irq, kbase_tag(kbdev, i));

	return err;
}

void kbase_release_interrupts(struct kbase_device *kbdev)
{
	u32 nr = ARRAY_SIZE(kbase_handler_table);
	u32 i;

	for (i = 0; i < nr; i++) {
		if (kbdev->irqs[i].irq)
			free_irq(kbdev->irqs[i].irq, kbase_tag(kbdev, i));
	}
}

void kbase_synchronize_irqs(struct kbase_device *kbdev)
{
	u32 nr = ARRAY_SIZE(kbase_handler_table);
	u32 i;

	for (i = 0; i < nr; i++) {
		if (kbdev->irqs[i].irq)
			synchronize_irq(kbdev->irqs[i].irq);
	}
}

