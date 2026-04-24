add_library(UserLibs STATIC
    c610/c610.cpp
    FP/FP.cpp
    pid/pid.cpp
    solenoid/solenoid.cpp
)

target_include_directories(UserLibs PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/c610
    ${CMAKE_CURRENT_SOURCE_DIR}/FP
    ${CMAKE_CURRENT_SOURCE_DIR}/pid
    ${CMAKE_CURRENT_SOURCE_DIR}/solenoid
    ${CMAKE_CURRENT_SOURCE_DIR}/vesc
    ${CMAKE_CURRENT_SOURCE_DIR}/can_utils
)

target_link_libraries(UserLibs PUBLIC stm32cubemx)
