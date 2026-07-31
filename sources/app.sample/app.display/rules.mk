# SPDX-License-Identifier: Apache-2.0
###################################################################################################
#   FileName : rules.mk
#   Copyright (c) Telechips Inc.
###################################################################################################

MCU_BSP_APP_DISPLAY_PATH := $(MCU_BSP_BUILD_CURDIR)

# Flags
COMMON_FLAGS += -DMCU_BSP_SUPPORT_APP_DISPLAY=1

# Paths
VPATH += $(MCU_BSP_APP_DISPLAY_PATH)

# Includes
INCLUDES += -I$(MCU_BSP_APP_DISPLAY_PATH)

# Sources
SRCS += lcd.c
