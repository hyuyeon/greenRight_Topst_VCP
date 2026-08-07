# SPDX-License-Identifier: Apache-2.0
###################################################################################################
#   FileName : rules.mk
#   Copyright (c) Telechips Inc.
###################################################################################################

MCU_BSP_APP_PED_FLAG_PATH := $(MCU_BSP_BUILD_CURDIR)

# Flags
COMMON_FLAGS += -DMCU_BSP_SUPPORT_APP_PED_FLAG=1

# Paths
VPATH += $(MCU_BSP_APP_PED_FLAG_PATH)

# Includes
INCLUDES += -I$(MCU_BSP_APP_PED_FLAG_PATH)

# Sources
SRCS += ped_flag_isr.c
