# SPDX-License-Identifier: Apache-2.0

###################################################################################################
#
#   FileName : ruls.mk
#
#   Copyright (c) Telechips Inc.
#
#   Description :
#
#
###################################################################################################

MCU_BSP_APP_SAMPLE_PATH := $(MCU_BSP_BUILD_CURDIR)

# Main
include $(MCU_BSP_APP_SAMPLE_PATH)/app.base/rules.mk

# CAN Demo Application
ifeq ($(MCU_BSP_BUILD_FLAGS_APP_CAN_DEMO), 1)
    include $(MCU_BSP_APP_SAMPLE_PATH)/app.can.demo/rules.mk
endif

# BNO055 IMU Application
ifeq ($(MCU_BSP_BUILD_FLAGS_APP_BNO055), 1)
    include $(MCU_BSP_APP_SAMPLE_PATH)/app.bno055/rules.mk
endif

# ADXL345 Application
ifeq ($(MCU_BSP_BUILD_FLAGS_APP_ADXL345), 1)
    include $(MCU_BSP_APP_SAMPLE_PATH)/app.adxl345/rules.mk
endif

# TCRT5000 Speed Sensor
ifeq ($(MCU_BSP_BUILD_FLAGS_APP_TCRT5000), 1)
    include $(MCU_BSP_APP_SAMPLE_PATH)/app.tcrt5000/rules.mk
endif

# SENSOR Application
ifeq ($(MCU_BSP_BUILD_FLAGS_APP_SENSOR), 1)
    include $(MCU_BSP_APP_SAMPLE_PATH)/app.sensor/rules.mk
endif

# POSITION Application
ifeq ($(MCU_BSP_BUILD_FLAGS_APP_POSITION), 1)
    include $(MCU_BSP_APP_SAMPLE_PATH)/app.position/rules.mk
endif

# DISPLAY Application
ifeq ($(MCU_BSP_BUILD_FLAGS_APP_DISPLAY), 1)
    include $(MCU_BSP_APP_SAMPLE_PATH)/app.display/rules.mk
endif

# TURN_JUDGE Application
ifeq ($(MCU_BSP_BUILD_FLAGS_APP_TURN_JUDGE), 1)
    include $(MCU_BSP_APP_SAMPLE_PATH)/app.turnjudge/rules.mk
endif

# QoS Application
ifeq ($(MCU_BSP_BUILD_FLAGS_APP_QOS), 1)
    include $(MCU_BSP_APP_SAMPLE_PATH)/app.qos/rules.mk
endif

# Buzzer Application
ifeq ($(MCU_BSP_BUILD_FLAGS_APP_BUZZER), 1)
    include $(MCU_BSP_APP_SAMPLE_PATH)/app.buzzer/rules.mk
endif

# CAN Test Application
ifeq ($(MCU_BSP_BUILD_FLAGS_TEST_APP_CAN), 1)
    include $(MCU_BSP_APP_SAMPLE_PATH)/test.app.can/rules.mk
endif

# Pedestrian Flag (AI-G camera UART link)
ifeq ($(MCU_BSP_BUILD_FLAGS_APP_PED_FLAG), 1)
    include $(MCU_BSP_APP_SAMPLE_PATH)/app.ped_flag/rules.mk
endif
