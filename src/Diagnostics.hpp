#pragma once

#include <cstdint>

namespace Interfaces::LLDriver
{
    class ICAN;
}
namespace Interfaces::HLDriver
{
    class ICANopen;
    class ICiA402;
}

/**
 * @brief Advanced commissioning diagnostics for the CANopen motor system.
 *
 * Provides deep introspection into the CAN bus, CANopen stack internals,
 * PDO/SYNC timing, heartbeat monitoring, emergency history, and drive-side
 * error readback.  All output goes to the serial console (printf).
 *
 * Thread safety: functions that access OD_RAM or CO_t internals use
 * CO_LOCK_OD / interrupt-disable as appropriate.  SDO-based drive reads
 * must only be called from the serial thread.
 */
namespace Diagnostics
{

    /**
     * @brief One-time initialisation — call after CO_CANopenInit succeeds.
     *
     * @param co        Opaque pointer to the CANopenNode CO_t stack object.
     * @param ican      Pointer to the low-level CAN driver (for error counters).
     * @param canopen   Pointer to the high-level CANopen interface (for SDO).
     */
    void init(void *co,
              Interfaces::LLDriver::ICAN *ican,
              Interfaces::HLDriver::ICANopen *canopen);

    /* ---------------------------------------------------------------------- */
    /* Individual diagnostic views                                            */
    /* ---------------------------------------------------------------------- */

    /** CAN bus health: error counters, bus state, overflow, error flags. */
    void printBusStatus();

    /** CANopen stack: local NMT state, error register, active error bits. */
    void printStackStatus();

    /** Local emergency history (OD 0x1003) — decoded error codes. */
    void printEmergencyHistory();

    /** Heartbeat consumer: per-monitored-node state, NMT state, timer. */
    void printHeartbeatStatus();

    /** Live PDO data: all OD_RAM 0x2000–0x2006 values + mapping summary. */
    void printPDOStatus();

    /** SYNC producer: enabled, period, window, timer, counter, late flag. */
    void printSyncStatus();

    /** Drive-side errors via SDO: error register, fault code, error history. */
    void printDriveErrors(uint8_t driveNodeId);

    /** CiA 402 state transition history from the motor driver. */
    void printStateLog(const Interfaces::HLDriver::ICiA402 *motor);

    /** Full diagnostic dump — calls all of the above. */
    void printAll(uint8_t driveNodeId, const Interfaces::HLDriver::ICiA402 *motor = nullptr);

    /**
     * @brief Continuous compact monitoring — prints one status line per interval.
     *
     * @param driveNodeId  Node ID of the drive to monitor.
     * @param count        Number of lines to print (0 = infinite until key press).
     * @param interval_ms  Milliseconds between each line.
     */
    void watch(uint8_t driveNodeId, uint32_t count, uint32_t interval_ms);

    /** Decode a CiA 301 SDO abort code to a human-readable string. */
    const char *decodeSDOAbort(uint32_t abortCode);

    /** Decode a CiA 301 emergency error code to a human-readable string. */
    const char *decodeEmergencyCode(uint16_t errorCode);

} // namespace Diagnostics
