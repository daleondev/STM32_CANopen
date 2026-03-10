#pragma once

#include "Interfaces/HLDriver/INanotecPnD.hpp"
#include "Interfaces/HLDriver/ICANopen.hpp"

namespace Implementations::HLDriver
{

    /**
     * @brief Minimal Nanotec Plug & Drive implementation — pure SDO, no PDOs.
     *
     * Every read/write goes over the SDO client.  Simple and suitable for
     * bringup / commissioning; not designed for cyclic real-time control.
     */
    class NanotecPnD final : public Interfaces::HLDriver::INanotecPnD
    {
    public:
        explicit NanotecPnD(Interfaces::HLDriver::ICANopen &canopen);

        bool configure(uint8_t nodeId,
                       uint32_t peakCurrent_mA,
                       uint32_t nomCurrent_mA) override;
        bool autoSetup() override;
        bool enable() override;
        bool disable() override;
        bool quickStop() override;
        bool faultReset() override;
        bool setMode(int8_t mode) override;
        bool setProfile(uint32_t velocity,
                        uint32_t acceleration,
                        uint32_t deceleration) override;
        bool moveAbsolute(int32_t position) override;
        bool moveRelative(int32_t distance) override;
        bool setVelocity(int32_t velocity) override;
        bool halt() override;
        Interfaces::HLDriver::NanotecStatus readStatus() override;
        bool save() override;

    private:
        Interfaces::HLDriver::ICANopen &canopen_;
        uint8_t nodeId_{0};
        bool configured_{false};

        /* ---- Nanotec-specific OD indices (drive-side) ---- */
        static constexpr uint16_t OD_PEAK_CURRENT = 0x2031;
        static constexpr uint16_t OD_NOMINAL_CURRENT = 0x2032;

        /* ---- CiA 402 standard OD indices ---- */
        static constexpr uint16_t OD_CONTROLWORD = 0x6040;
        static constexpr uint16_t OD_STATUSWORD = 0x6041;
        static constexpr uint16_t OD_MODES_OF_OPERATION = 0x6060;
        static constexpr uint16_t OD_MODES_DISPLAY = 0x6061;
        static constexpr uint16_t OD_POSITION_ACTUAL = 0x6064;
        static constexpr uint16_t OD_VELOCITY_ACTUAL = 0x606C;
        static constexpr uint16_t OD_TARGET_POSITION = 0x607A;
        static constexpr uint16_t OD_PROFILE_VELOCITY = 0x6081;
        static constexpr uint16_t OD_PROFILE_ACCEL = 0x6083;
        static constexpr uint16_t OD_PROFILE_DECEL = 0x6084;
        static constexpr uint16_t OD_TARGET_VELOCITY = 0x60FF;
        static constexpr uint16_t OD_STORE_PARAMS = 0x1010;

        /* ---- Nanotec auto setup mode ---- */
        static constexpr int8_t MODE_AUTO_SETUP = -1;

        /* ---- CiA 402 controlword bits ---- */
        static constexpr uint16_t CW_SWITCH_ON = 1U << 0;
        static constexpr uint16_t CW_ENABLE_VOLTAGE = 1U << 1;
        static constexpr uint16_t CW_QUICK_STOP = 1U << 2;
        static constexpr uint16_t CW_ENABLE_OPERATION = 1U << 3;
        static constexpr uint16_t CW_NEW_SETPOINT = 1U << 4;
        static constexpr uint16_t CW_ABS_REL = 1U << 6;
        static constexpr uint16_t CW_FAULT_RESET = 1U << 7;
        static constexpr uint16_t CW_HALT = 1U << 8;

        /* ---- CiA 402 statusword bits ---- */
        static constexpr uint16_t SW_READY_TO_SWITCH_ON = 1U << 0;
        static constexpr uint16_t SW_SWITCHED_ON = 1U << 1;
        static constexpr uint16_t SW_OPERATION_ENABLED = 1U << 2;
        static constexpr uint16_t SW_FAULT = 1U << 3;
        static constexpr uint16_t SW_QUICK_STOP = 1U << 5;
        static constexpr uint16_t SW_SWITCH_ON_DISABLED = 1U << 6;
        static constexpr uint16_t SW_TARGET_REACHED = 1U << 10;

        /* ---- Timeouts ---- */
        static constexpr uint32_t STATE_TIMEOUT_MS = 3000;
        static constexpr uint32_t AUTO_SETUP_TIMEOUT_MS = 60000;
        static constexpr uint32_t POLL_MS = 50;

        /* ---- SDO helpers ---- */
        bool sdoWriteU8(uint16_t index, uint8_t sub, uint8_t val);
        bool sdoWriteU16(uint16_t index, uint8_t sub, uint16_t val);
        bool sdoWriteU32(uint16_t index, uint8_t sub, uint32_t val);
        bool sdoWriteI8(uint16_t index, uint8_t sub, int8_t val);
        bool sdoWriteI32(uint16_t index, uint8_t sub, int32_t val);

        bool sdoReadU16(uint16_t index, uint8_t sub, uint16_t &out);
        bool sdoReadI32(uint16_t index, uint8_t sub, int32_t &out);
        bool sdoReadI8(uint16_t index, uint8_t sub, int8_t &out);

        /** Write controlword via SDO. */
        bool writeControlword(uint16_t cw);

        /** Read statusword via SDO. */
        bool readStatusword(uint16_t &sw);

        /**
         * @brief Walk CiA 402 state machine to OperationEnabled via SDO.
         * Handles: SwitchOnDisabled → ReadyToSwitchOn → SwitchedOn → OperationEnabled.
         */
        bool walkToOperationEnabled();

        /** Block-poll statusword until a bit mask matches, or timeout. */
        bool pollStatusword(uint16_t mask, uint16_t expected,
                            uint32_t timeout_ms);

        /** ThreadX delay helper. */
        void delayMs(uint32_t ms);
    };

} // namespace Implementations::HLDriver
