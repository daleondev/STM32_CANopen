#pragma once

#include "Interfaces/HLDriver/ICiA402.hpp"
#include "Interfaces/HLDriver/ICANopen.hpp"

#include <atomic>

extern "C"
{
#include "CANopen.h"
#include "OD.h"
}

namespace Implementations::HLDriver
{

    /**
     * @brief CiA 402 drive profile implementation (single drive).
     *
     * Uses PDO (via local OD variables 0x2000–0x2006) for all real-time
     * data exchange: controlword, statusword, target/actual position,
     * modes of operation, and target velocity.
     * SDO is used only for one-shot configuration (heartbeat, etc.).
     *
     * Thread safety: all access to OD_RAM and the cached status_ struct
     * is protected by CO_LOCK_OD / CO_UNLOCK_OD (ThreadX mutex).
     */
    class CiA402 final : public Interfaces::HLDriver::ICiA402
    {
    public:
        /**
         * @brief Construct with a reference to the CANopen stack driver.
         * @param canopen  Reference used for SDO transfers and NMT commands.
         */
        explicit CiA402(Interfaces::HLDriver::ICANopen &canopen);

        bool init(uint8_t driveNodeId) override;
        Interfaces::HLDriver::DriveStatus getStatus() const override;
        bool enable() override;
        bool disable() override;
        bool quickStop() override;
        bool faultReset() override;
        bool setOperationMode(Interfaces::HLDriver::OperationMode mode) override;
        bool moveToPosition(int32_t position, bool relative = false) override;
        bool setTargetVelocity(int32_t velocity) override;
        void update() override;

    private:
        Interfaces::HLDriver::ICANopen &canopen_;
        uint8_t driveNodeId_{0};
        std::atomic<bool> initialized_{false};

        /* Cached status — updated every cycle via update() */
        Interfaces::HLDriver::DriveStatus status_{};

        /* ------------------------------------------------------------------ */
        /* CiA 402 controlword bit masks                                      */
        /* ------------------------------------------------------------------ */
        static constexpr uint16_t CW_SWITCH_ON = 1U << 0;
        static constexpr uint16_t CW_ENABLE_VOLTAGE = 1U << 1;
        static constexpr uint16_t CW_QUICK_STOP = 1U << 2;
        static constexpr uint16_t CW_ENABLE_OPERATION = 1U << 3;
        static constexpr uint16_t CW_NEW_SETPOINT = 1U << 4; /* PP mode */
        static constexpr uint16_t CW_ABS_REL = 1U << 6;      /* PP mode: 1=relative */
        static constexpr uint16_t CW_FAULT_RESET = 1U << 7;

        /* ------------------------------------------------------------------ */
        /* CiA 402 statusword bit masks                                       */
        /* ------------------------------------------------------------------ */
        static constexpr uint16_t SW_READY_TO_SWITCH_ON = 1U << 0;
        static constexpr uint16_t SW_SWITCHED_ON = 1U << 1;
        static constexpr uint16_t SW_OPERATION_ENABLED = 1U << 2;
        static constexpr uint16_t SW_FAULT = 1U << 3;
        static constexpr uint16_t SW_VOLTAGE_ENABLED = 1U << 4;
        static constexpr uint16_t SW_QUICK_STOP = 1U << 5;
        static constexpr uint16_t SW_SWITCH_ON_DISABLED = 1U << 6;
        static constexpr uint16_t SW_WARNING = 1U << 7;
        static constexpr uint16_t SW_TARGET_REACHED = 1U << 10;

        /* Timeouts */
        static constexpr uint32_t STATE_TRANSITION_TIMEOUT_MS = 3000;
        static constexpr uint32_t POLL_INTERVAL_MS = 10; /* matches SYNC/thread cycle */

        /* ------------------------------------------------------------------ */
        /* Helpers                                                            */
        /* ------------------------------------------------------------------ */
        /** Decode CiA 402 state from statusword. */
        static Interfaces::HLDriver::CiA402State decodeState(uint16_t sw);

        /** Write controlword to local OD (picked up by TPDO). */
        void setControlword(uint16_t cw);

        /** Read statusword from local OD (populated by RPDO). */
        uint16_t readStatusword() const;

        /** Block until a target state is reached or timeout. */
        bool waitForState(Interfaces::HLDriver::CiA402State target, uint32_t timeout_ms);

        /** SDO helpers — write a typed value to the remote drive. */
        bool sdoWriteU8(uint16_t index, uint8_t sub, uint8_t val);
        bool sdoWriteU16(uint16_t index, uint8_t sub, uint16_t val);
        bool sdoWriteU32(uint16_t index, uint8_t sub, uint32_t val);
    };

} // namespace Implementations::HLDriver
