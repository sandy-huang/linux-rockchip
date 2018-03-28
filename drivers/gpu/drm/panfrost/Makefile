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



# Driver version string which is returned to userspace via an ioctl
MALI_RELEASE_NAME ?= "r12p0-04rel0"

# Paths required for build
KBASE_PATH = $(src)

# Set up our defines, which will be passed to gcc
DEFINES = -DMALI_RELEASE_NAME=\"$(MALI_RELEASE_NAME)\"

# Use our defines when compiling
ccflags-y += $(DEFINES) -I$(KBASE_PATH) -I$(srctree)/include/linux -I$(KBASE_PATH)/backend/gpu
#subdir-ccflags-y += $(DEFINES) -I$(KBASE_PATH) -I$(srctree)/include/linux

SRC := \
	mali_kbase_device.c \
	mali_kbase_cache_policy.c \
	mali_kbase_mem.c \
	mali_kbase_mmu.c \
	mali_kbase_ipa.c \
	mali_kbase_jd.c \
	mali_kbase_jm.c \
	mali_kbase_gpuprops.c \
	mali_kbase_js.c \
	mali_kbase_js_ctx_attr.c \
	mali_kbase_event.c \
	mali_kbase_context.c \
	mali_kbase_pm.c \
	mali_kbase_instr.c \
	mali_kbase_vinstr.c \
	mali_kbase_softjobs.c \
	mali_kbase_10969_workaround.c \
	mali_kbase_hw.c \
	mali_kbase_utility.c \
	mali_kbase_debug.c \
	mali_kbase_trace_timeline.c \
	mali_kbase_mem_linux.c \
	mali_kbase_core_linux.c \
	mali_kbase_sync.c \
	mali_kbase_sync_user.c \
	mali_kbase_replay.c \
	mali_kbase_mmu_mode_lpae.c \
	mali_kbase_disjoint_events.c \
	mali_kbase_smc.c \
	mali_kbase_mem_pool.c \
	mali_kbase_tlstream.c

# Job Scheduler Policy: Completely Fair Scheduler
SRC += mali_kbase_js_policy_cfs.c


# Tell the Linux build system from which .o file to create the kernel module
obj-$(CONFIG_MALI_MIDGARD) += mali_kbase.o

# Tell the Linux build system to enable building of our .c files
mali_kbase-y := $(SRC:.c=.o)

mali_kbase-$(CONFIG_MALI_DMA_FENCE) += mali_kbase_dma_fence.o

BACKEND += \
	backend/gpu/mali_kbase_cache_policy_backend.c \
	backend/gpu/mali_kbase_device_hw.c \
	backend/gpu/mali_kbase_gpu.c \
	backend/gpu/mali_kbase_gpuprops_backend.c \
	backend/gpu/mali_kbase_irq_linux.c \
	backend/gpu/mali_kbase_instr_backend.c \
	backend/gpu/mali_kbase_jm_as.c \
	backend/gpu/mali_kbase_jm_hw.c \
	backend/gpu/mali_kbase_jm_rb.c \
	backend/gpu/mali_kbase_js_affinity.c \
	backend/gpu/mali_kbase_js_backend.c \
	backend/gpu/mali_kbase_mmu_hw_direct.c \
	backend/gpu/mali_kbase_pm_backend.c \
	backend/gpu/mali_kbase_pm_driver.c \
	backend/gpu/mali_kbase_pm_metrics.c \
	backend/gpu/mali_kbase_pm_ca.c \
	backend/gpu/mali_kbase_pm_ca_fixed.c \
	backend/gpu/mali_kbase_pm_always_on.c \
	backend/gpu/mali_kbase_pm_coarse_demand.c \
	backend/gpu/mali_kbase_pm_demand.c \
	backend/gpu/mali_kbase_pm_policy.c \
	backend/gpu/mali_kbase_time.c

ifeq ($(CONFIG_MALI_DEVFREQ),y)
BACKEND += backend/gpu/mali_kbase_devfreq.c
endif

ifeq ($(CONFIG_DEVFREQ_THERMAL),y)
	BACKEND += backend/gpu/mali_kbase_power_model_simple.c
endif

mali_kbase-y += $(BACKEND:.c=.o)
