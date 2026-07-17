#===============================================================================
# @brief    cmake file
# Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
#===============================================================================
if(CONFIG_SAMPLE_SUPPORT_BLE_SAMPLE)
    add_subdirectory_if_exist(ble)
endif()
if(CONFIG_SAMPLE_SUPPORT_SLE_SAMPLE)
    add_subdirectory_if_exist(sle)
endif()

set(SOURCES "${SOURCES}" PARENT_SCOPE)
set(PUBLIC_HEADER "${PUBLIC_HEADER}" PARENT_SCOPE)
