# SPDX-License-Identifier: Apache-2.0
###################################################################################################
#   FileName : rules.mk
#   Copyright (c) Telechips Inc.
###################################################################################################

MCU_BSP_APP_TCRT5000_PATH := $(MCU_BSP_BUILD_CURDIR)

# Flags
COMMON_FLAGS += -DMCU_BSP_SUPPORT_APP_TCRT5000=1

# Paths
VPATH += $(MCU_BSP_APP_TCRT5000_PATH)

# Includes
INCLUDES += -I$(MCU_BSP_APP_TCRT5000_PATH)

# Sources
SRCS += speed_sensor.c