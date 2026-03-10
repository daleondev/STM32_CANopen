add_library(
    CANopen
)

target_sources(CANopen
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_ODinterface.c
        ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_NMT_Heartbeat.c
        ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_HBconsumer.c
        ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_Emergency.c
        ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_SDOserver.c
        ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_TIME.c
        ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_SYNC.c
        ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_PDO.c
        ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/303/CO_LEDs.c
        ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/305/CO_LSSslave.c
        ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/storage/CO_storage.c
        ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/CANopen.c

        ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode_port/OD.c
        ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode_port/CO_driver.cpp
)

target_include_directories(CANopen
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode
        ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode_port
)


target_link_libraries(CANopen
    PRIVATE
        Platform
)