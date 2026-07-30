# SPDX-License-Identifier: Apache-2.0
###################################################################################################
#   FileName : rules.mk
#   Description : VCP-G ADXL345 standalone test application
###################################################################################################

MCU_BSP_APP_ADXL345_PATH := $(MCU_BSP_BUILD_CURDIR)

COMMON_FLAGS += -DMCU_BSP_SUPPORT_APP_ADXL345=1

VPATH += $(MCU_BSP_APP_ADXL345_PATH)

INCLUDES += -I$(MCU_BSP_APP_ADXL345_PATH)

SRCS += adxl345.c
SRCS += led.c
SRCS += exti_button.c
