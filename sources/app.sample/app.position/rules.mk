# SPDX-License-Identifier: Apache-2.0
###################################################################################################
#   FileName : rules.mk
#   Copyright (c) Telechips Inc.
###################################################################################################

MCU_BSP_APP_POSITION_PATH := $(MCU_BSP_BUILD_CURDIR)

# Flags
COMMON_FLAGS += -DMCU_BSP_SUPPORT_APP_POSITION=1

# Paths
VPATH += $(MCU_BSP_APP_POSITION_PATH)

# Includes
INCLUDES += -I$(MCU_BSP_APP_POSITION_PATH)

# Sources
SRCS += position.c