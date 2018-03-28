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
KBASE_PLATFORM_PATH = $(KBASE_PATH)/platform_dummy
UMP_PATH = $(src)/../../../base

# Set up our defines, which will be passed to gcc
DEFINES = -DMALI_RELEASE_NAME=\"$(MALI_RELEASE_NAME)\"

ifeq ($(KBUILD_EXTMOD),)
# in-tree
DEFINES +=-DMALI_KBASE_THIRDPARTY_PATH=../../$(src)/platform/$(CONFIG_MALI_PLATFORM_THIRDPARTY_NAME)
else
# out-of-tree
DEFINES +=-DMALI_KBASE_THIRDPARTY_PATH=$(src)/platform/$(CONFIG_MALI_PLATFORM_THIRDPARTY_NAME)
endif

DEFINES += -I$(srctree)/drivers/staging/android

# Use our defines when compiling
ccflags-y += $(DEFINES) -I$(KBASE_PATH)   -I$(KBASE_PLATFORM_PATH) -I$(UMP_PATH) -I$(srctree)/include/linux
subdir-ccflags-y += $(DEFINES) -I$(KBASE_PATH)   -I$(KBASE_PLATFORM_PATH) -I$(OSK_PATH) -I$(UMP_PATH) -I$(srctree)/include/linux

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
	mali_kbase_config.c \
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

ccflags-y += -I$(KBASE_PATH)

ifeq ($(CONFIG_MALI_PLATFORM_THIRDPARTY),y)
# remove begin and end quotes from the Kconfig string type
platform_name := $(shell echo $(CONFIG_MALI_PLATFORM_THIRDPARTY_NAME))
MALI_PLATFORM_THIRDPARTY_DIR := platform/$(platform_name)
ccflags-y += -I$(src)/$(MALI_PLATFORM_THIRDPARTY_DIR)
ifeq ($(CONFIG_MALI_MIDGARD),m)
include  $(src)/platform/$(platform_name)/Makefile
else ifeq ($(CONFIG_MALI_MIDGARD),y)
obj-$(CONFIG_MALI_MIDGARD) += platform/
endif
endif

# Tell the Linux build system from which .o file to create the kernel module
obj-$(CONFIG_MALI_MIDGARD) += mali_kbase.o

# Tell the Linux build system to enable building of our .c files
mali_kbase-y := $(SRC:.c=.o)

mali_kbase-$(CONFIG_MALI_DMA_FENCE) += mali_kbase_dma_fence.o

MALI_BACKEND_PATH ?= backend
CONFIG_MALI_BACKEND ?= gpu
CONFIG_MALI_BACKEND_REAL ?= $(CONFIG_MALI_BACKEND)

include  $(src)/$(MALI_BACKEND_PATH)/$(CONFIG_MALI_BACKEND_REAL)/Kbuild
mali_kbase-y += $(BACKEND:.c=.o)

ccflags-y += -I$(src)/$(MALI_BACKEND_PATH)/$(CONFIG_MALI_BACKEND_REAL)
subdir-ccflags-y += -I$(src)/$(MALI_BACKEND_PATH)/$(CONFIG_MALI_BACKEND_REAL)

# Default to devicetree platform if a thirdparty platform is not configured.
ifeq ($(CONFIG_MALI_PLATFORM_THIRDPARTY),)
CONFIG_MALI_PLATFORM_DEVICETREE := y
endif
