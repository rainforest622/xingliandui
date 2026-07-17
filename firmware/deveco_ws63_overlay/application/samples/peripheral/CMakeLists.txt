#===============================================================================
# @brief    cmake file
# Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
#===============================================================================
if(CONFIG_SAMPLE_SUPPORT_AMIC)
    add_subdirectory_if_exist(amic)
endif()

if(CONFIG_SAMPLE_SUPPORT_BLINKY)
    add_subdirectory_if_exist(blinky)
endif()

if(CONFIG_SAMPLE_SUPPORT_HELLOWORLD)
    add_subdirectory_if_exist(helloworld)
endif()

if(CONFIG_SAMPLE_SUPPORT_BUTTON)
    add_subdirectory_if_exist(button)
endif()

if(CONFIG_SAMPLE_SUPPORT_TASKS)
    add_subdirectory_if_exist(tasks)
endif()

if(CONFIG_SAMPLE_SUPPORT_TCXO)
    add_subdirectory_if_exist(tcxo)
endif()

if(CONFIG_SAMPLE_SUPPORT_DMA)
    add_subdirectory_if_exist(dma)
endif()

if(CONFIG_SAMPLE_SUPPORT_I2C)
    add_subdirectory_if_exist(i2c)
endif()

if(CONFIG_SAMPLE_SUPPORT_I2S)
    add_subdirectory_if_exist(i2s)
endif()

if(CONFIG_SAMPLE_SUPPORT_KEYSCAN)
    add_subdirectory_if_exist(keyscan)
endif()

if(CONFIG_SAMPLE_SUPPORT_PDM)
    add_subdirectory_if_exist(pdm)
endif()

if(CONFIG_SAMPLE_SUPPORT_PDM_AMIC)
    add_subdirectory_if_exist(pdm_amic)
endif()

if(CONFIG_SAMPLE_SUPPORT_PINCTRL)
    add_subdirectory_if_exist(pinctrl)
endif()

if(CONFIG_SAMPLE_SUPPORT_QDEC)
    add_subdirectory_if_exist(qdec)
endif()

if(CONFIG_SAMPLE_SUPPORT_SFC)
    add_subdirectory_if_exist(sfc)
endif()

if(CONFIG_SAMPLE_SUPPORT_SFC_PERF)
    add_subdirectory_if_exist(sfc_perf)
endif()

if(CONFIG_SAMPLE_SUPPORT_SPI)
    add_subdirectory_if_exist(spi)
endif()

if(CONFIG_SAMPLE_SUPPORT_SYSTICK)
    add_subdirectory_if_exist(systick)
endif()

if(CONFIG_SAMPLE_SUPPORT_WDT)
    add_subdirectory_if_exist(watchdog)
endif()

if(CONFIG_SAMPLE_SUPPORT_PWM)
    add_subdirectory_if_exist(pwm)
endif()

if(CONFIG_SAMPLE_SUPPORT_UART)
    add_subdirectory_if_exist(uart)
endif()

if(CONFIG_SAMPLE_SUPPORT_ROBOT_MVP)
    add_subdirectory_if_exist(robot_mvp)
endif()

if(CONFIG_SAMPLE_SUPPORT_ADC)
    add_subdirectory_if_exist(adc)
endif()

if(CONFIG_SAMPLE_SUPPORT_RTC)
    add_subdirectory_if_exist(rtc)
endif()

if(CONFIG_SAMPLE_SUPPORT_TIMER)
    add_subdirectory_if_exist(timer)
endif()

if(CONFIG_SAMPLE_SUPPORT_EFLASH)
    add_subdirectory_if_exist(eflash)
endif()

if(CONFIG_SAMPLE_SUPPORT_CALENDAR)
    add_subdirectory_if_exist(calendar)
endif()

if(CONFIG_SAMPLE_SUPPORT_I2S_DMA)
    add_subdirectory_if_exist(i2s_dma)
endif()

if(CONFIG_SAMPLE_SUPPORT_I2S_DMA_LLI)
    add_subdirectory_if_exist(i2s_dma_lli)
endif()

if(CONFIG_SAMPLE_SUPPORT_CAN)
    add_subdirectory_if_exist(can)
endif()

if(CONFIG_SAMPLE_SUPPORT_CLOCKS)
    add_subdirectory_if_exist(clocks)
endif()

if(CONFIG_SAMPLE_SUPPORT_HELLOWORLD_OLED)
    add_subdirectory_if_exist(helloworld_oled)
endif()

set(SOURCES "${SOURCES}" PARENT_SCOPE)
