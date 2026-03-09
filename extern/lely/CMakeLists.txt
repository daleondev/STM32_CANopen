set(LELY_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lely-core)
set(LELY_PORT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lely-core_port)

add_library(CANopen)

file(GLOB_RECURSE LELY_CORE_SOURCES
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
list(FILTER LELY_CORE_SOURCES EXCLUDE REGEX "/src/io2/(linux|posix|win32)/.*\\.c$")
list(FILTER LELY_CORE_SOURCES EXCLUDE REGEX "/src/libc/time\\.c$")

target_sources(CANopen
    PRIVATE
        ${LELY_PORT_DIR}/osal/threadx_threads.c
        ${LELY_PORT_DIR}/osal/threadx_time.c
        ${LELY_CORE_SOURCES}
)

target_include_directories(CANopen
    BEFORE
    PRIVATE
        ${LELY_PORT_DIR}/cxxshim
        ${LELY_CORE_DIR}/src
    PUBLIC
        ${LELY_PORT_DIR}/include
    PUBLIC
        ${LELY_CORE_DIR}/include
)

target_compile_definitions(CANopen
    PUBLIC
        LELY_NO_DAEMON=1
        LELY_NO_COAPP_SLAVE=1
        LELY_HAVE_VCI=0
        LELY_NO_STDIO=1
        LELY_HAVE_SOCKET_CAN=0
        LELY_HAVE_THREADS_H=1
        LELY_THREADX_STACK_SIZE=16384
    NDEBUG=1
)

target_link_libraries(CANopen
    PUBLIC
        threadx
)