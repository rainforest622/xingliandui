set(ROBOT_MVP_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/robot_mvp.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot_protocol.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot_buzzer.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot_i2c.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot_env.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot_obstacle.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot_motor.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot_rover_link.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot_oled.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../helloworld_oled/ssd1306.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../helloworld_oled/ssd1306_fonts.c"
)

if(CONFIG_ROBOT_MVP_ENABLE_SLE)
    list(APPEND ROBOT_MVP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/robot_sle_server.c")
endif()

set(SOURCES
    "${SOURCES}"
    ${ROBOT_MVP_SOURCES}
    PARENT_SCOPE
)
