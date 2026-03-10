cmake_minimum_required(VERSION 3.22)

project(CANopenApp LANGUAGES C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/Platform.cmake)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/extern/CANopenNode)

add_executable(${PROJECT_NAME})

target_sources(${PROJECT_NAME}
    PRIVATE
        src/main.cpp
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(${PROJECT_NAME}
    PRIVATE
        cxx_std_20
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        threadx
        Platform
        CANopen
)

set(APP_ARTIFACT_BASE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}")

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> "${APP_ARTIFACT_BASE}.hex"
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> "${APP_ARTIFACT_BASE}.bin"
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Generating firmware images"
)