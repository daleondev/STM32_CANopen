cmake_minimum_required(VERSION 3.22)

project(CANopenApp LANGUAGES CXX C ASM)
set(CMAKE_PROJECT_NAME CANopenApp)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

include("Platform.cmake")

# --- Executable ---
add_executable(CANopenApp src/main.cpp)

# --- Sources ---
target_sources(CANopenApp 
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src/main.cpp
)

# --- Includes ---
target_include_directories(CANopenApp 
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(CANopenApp 
    PRIVATE
        threadx
        Platform
)

target_compile_features(CANopenApp
    PUBLIC
        cxx_std_20
)

# --- Post-build steps ---
add_custom_command(TARGET CANopenApp POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:CANopenApp> CANopenApp.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:CANopenApp> CANopenApp.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:CANopenApp>
    COMMENT "Generating hex and bin files, and printing size"
)