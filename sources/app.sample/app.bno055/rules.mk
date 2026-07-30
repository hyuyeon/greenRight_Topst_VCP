# SPDX-License-Identifier: Apache-2.0
###################################################################################################
#   FileName : rules.mk
#   Copyright (c) Telechips Inc.
###################################################################################################

MCU_BSP_APP_BNO055_PATH := $(MCU_BSP_BUILD_CURDIR)

# Flags
COMMON_FLAGS += -DMCU_BSP_SUPPORT_APP_BNO055=1

# Paths
VPATH += $(MCU_BSP_APP_BNO055_PATH)

# Includes
INCLUDES += -I$(MCU_BSP_APP_BNO055_PATH)

# Sources
SRCS += bno055.c
SRCS += bno055_app.c
SRCS += i2c1_bus.c

