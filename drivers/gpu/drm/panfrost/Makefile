#
# (C) COPYRIGHT 2012,2014 ARM Limited. All rights reserved.
#
# This program is free software and is provided to you under the terms of the
# GNU General Public License version 2 as published by the Free Software
# Foundation, and any use by you of this program is subject to the terms
# of such GNU licence.
#
# A copy of the licence is included with the program, and can also be obtained
# from Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
# Boston, MA  02110-1301, USA.
#
#

# Use our defines when compiling
ccflags-y += -I$(src) -I$(src)/backend/gpu

panfrost-y := \
	mali_kbase_device.o \
	mali_kbase_cache_policy.o \
	mali_kbase_mem.o \
	mali_kbase_mmu.o \
	mali_kbase_ipa.o \
	mali_kbase_jd.o \
	mali_kbase_jm.o \
	mali_kbase_gpuprops.o \
	mali_kbase_js.o \
	mali_kbase_js_ctx_attr.o \
	mali_kbase_js_policy_cfs.o \
	mali_kbase_event.o \
	mali_kbase_context.o \
	mali_kbase_pm.o \
	mali_kbase_instr.o \
	mali_kbase_vinstr.o \
	mali_kbase_softjobs.o \
	mali_kbase_10969_workaround.o \
	mali_kbase_hw.o \
	mali_kbase_utility.o \
	mali_kbase_mem_linux.o \
	mali_kbase_core_linux.o \
	mali_kbase_replay.o \
	mali_kbase_mmu_mode_lpae.o \
	mali_kbase_disjoint_events.o \
	mali_kbase_smc.o \
	mali_kbase_mem_pool.o \
	mali_kbase_tlstream.o \
	mali_kbase_dma_fence.o \
	backend/gpu/mali_kbase_cache_policy_backend.o \
	backend/gpu/mali_kbase_device_hw.o \
	backend/gpu/mali_kbase_gpu.o \
	backend/gpu/mali_kbase_gpuprops_backend.o \
	backend/gpu/mali_kbase_instr_backend.o \
	backend/gpu/mali_kbase_jm_as.o \
	backend/gpu/mali_kbase_jm_hw.o \
	backend/gpu/mali_kbase_jm_rb.o \
	backend/gpu/mali_kbase_js_affinity.o \
	backend/gpu/mali_kbase_js_backend.o \
	backend/gpu/mali_kbase_mmu_hw_direct.o \
	backend/gpu/mali_kbase_pm_backend.o \
	backend/gpu/mali_kbase_pm_driver.o \
	backend/gpu/mali_kbase_pm_metrics.o \
	backend/gpu/mali_kbase_pm_ca.o \
	backend/gpu/mali_kbase_pm_ca_fixed.o \
	backend/gpu/mali_kbase_pm_always_on.o \
	backend/gpu/mali_kbase_pm_coarse_demand.o \
	backend/gpu/mali_kbase_pm_demand.o \
	backend/gpu/mali_kbase_pm_policy.o \
	backend/gpu/mali_kbase_time.o

# Tell the Linux build system from which .o file to create the kernel module
obj-$(CONFIG_DRM_PANFROST) += panfrost.o
