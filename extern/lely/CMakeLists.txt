cmake_minimum_required(VERSION 3.20)

set(LELY_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lely-core)
set(LELY_PORT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lely-core_port)

file(GLOB_RECURSE LELY_SOURCES
    CONFIGURE_DEPENDS
    "${LELY_CORE_DIR}/src/libc/*.c"
    "${LELY_CORE_DIR}/src/util/*.c"
    "${LELY_CORE_DIR}/src/util/*.cpp"
    "${LELY_CORE_DIR}/src/can/*.c"
    "${LELY_CORE_DIR}/src/ev/*.c"
    "${LELY_CORE_DIR}/src/co/*.c"
    "${LELY_CORE_DIR}/src/io2/*.c"
    "${LELY_CORE_DIR}/src/coapp/*.cpp"
)

list(FILTER LELY_SOURCES EXCLUDE REGEX "/src/io2/(linux|posix|win32)/.*\\.c$")

add_library(lely STATIC)
add_library(Lely::lely ALIAS lely)
add_library(lely::lely ALIAS lely)

target_sources(lely
    PRIVATE
        ${LELY_SOURCES}
)

target_include_directories(lely
    PUBLIC
        $<BUILD_INTERFACE:${LELY_CORE_DIR}/include>
    PRIVATE
        $<BUILD_INTERFACE:${LELY_CORE_DIR}/src>
)

target_compile_features(lely
    PUBLIC
        cxx_std_17
)

target_compile_definitions(lely
    PUBLIC
        LELY_NO_DAEMON=1
        LELY_NO_COAPP_SLAVE=1
        LELY_HAVE_VCI=0
)

target_compile_options(lely
    PRIVATE
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wno-unused-parameter>
)

if(CMAKE_SYSTEM_NAME STREQUAL "Generic")
    target_sources(lely
        PRIVATE
            ${LELY_PORT_DIR}/osal/threadx_threads.c
            ${LELY_PORT_DIR}/osal/threadx_time.c
    )

    target_include_directories(lely
        BEFORE PUBLIC
            $<BUILD_INTERFACE:${LELY_PORT_DIR}/include>
    )

    target_compile_definitions(lely
        PUBLIC
            LELY_NO_STDIO=1
            LELY_HAVE_SOCKET_CAN=0
            LELY_HAVE_THREADS_H=1
            LELY_THREADX_STACK_SIZE=16384
    )

    target_link_libraries(lely
        PUBLIC
            threadx
    )
else()
    find_package(Threads REQUIRED)

    target_link_libraries(lely
        PUBLIC
            Threads::Threads
    )
endif()