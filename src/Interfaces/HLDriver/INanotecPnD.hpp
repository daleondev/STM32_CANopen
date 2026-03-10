#pragma once

#include <cstdint>

namespace Interfaces::HLDriver
{

    /**
     * @brief Snapshot of a Nanotec drive's current status (read via SDO).
     */
    struct NanotecStatus
    {
        uint16_t statusword{0};
        int32_t position{0};
        int32_t velocity{0};
        int8_t modeDisplay{0};
    };

    /**
     * @brief Very minimal Nanotec "Plug & Drive" interface for bringup tests.
     *
     * All operations are SDO-based (blocking).  No PDO configuration required.
     * Designed to be called from a low-priority CLI thread.
     *
     * Typical bringup sequence:
     *   1. configure()  — set node ID + current limits
     *   2. autoSetup()  — Nanotec auto-tuning (motor identification)
     *   3. save()       — persist parameters in drive NVM
     *   4. setMode()    — select Profile Position (1) or Profile Velocity (3)
     *   5. setProfile() — set velocity / acceleration / deceleration
     *   6. enable()     — CiA 402 → OperationEnabled
     *   7. moveAbsolute() / setVelocity()
     *   8. readStatus() — poll actual position / velocity
     */
    class INanotecPnD
    {
    public:
        virtual ~INanotecPnD() = default;

        /**
         * @brief Configure basic motor parameters via SDO.
         *
         * Sends NMT Pre-Operational, writes 0x2031 (peak current) and
         * 0x2032 (nominal current), then NMT Start.
         *
         * @param nodeId          CAN node ID of the Nanotec drive (1..127).
         * @param peakCurrent_mA  Peak current limit in mA (0x2031).
         * @param nomCurrent_mA   Nominal / rated current in mA (0x2032).
         * @return true on success.
         */
        virtual bool configure(uint8_t nodeId,
                               uint32_t peakCurrent_mA,
                               uint32_t nomCurrent_mA) = 0;

        /**
         * @brief Run Nanotec Auto Setup (motor identification).
         *
         * Sets modes-of-operation to −1, walks the CiA 402 state machine to
         * OperationEnabled, and blocks until the drive signals completion
         * (statusword bit 10) or a timeout of ~60 s is reached.
         *
         * The motor **will move** during auto setup.
         *
         * @return true if auto setup completed successfully.
         */
        virtual bool autoSetup() = 0;

        /** @brief CiA 402 state machine → OperationEnabled (via SDO). */
        virtual bool enable() = 0;

        /** @brief CiA 402 → SwitchedOn (disable operation). */
        virtual bool disable() = 0;

        /** @brief CiA 402 quick stop. */
        virtual bool quickStop() = 0;

        /** @brief Reset a latched fault. */
        virtual bool faultReset() = 0;

        /**
         * @brief Set the CiA 402 mode of operation via SDO (0x6060).
         * @param mode  e.g. 1 = Profile Position, 3 = Profile Velocity,
         *              −1 = Auto Setup (Nanotec).
         */
        virtual bool setMode(int8_t mode) = 0;

        /**
         * @brief Set profile motion parameters.
         * @param velocity      Profile velocity      (0x6081, counts/s).
         * @param acceleration  Profile acceleration  (0x6083, counts/s²).
         * @param deceleration  Profile deceleration  (0x6084, counts/s²).
         */
        virtual bool setProfile(uint32_t velocity,
                                uint32_t acceleration,
                                uint32_t deceleration) = 0;

        /**
         * @brief Absolute position move (Profile Position mode).
         * @param position  Target position in encoder counts (0x607A).
         */
        virtual bool moveAbsolute(int32_t position) = 0;

        /**
         * @brief Relative position move (Profile Position mode).
         * @param distance  Distance in encoder counts (0x607A).
         */
        virtual bool moveRelative(int32_t distance) = 0;

        /**
         * @brief Set target velocity (Profile Velocity mode, 0x60FF).
         * @param velocity  Target velocity in counts/s (signed).
         */
        virtual bool setVelocity(int32_t velocity) = 0;

        /** @brief Halt motion (set controlword halt bit). */
        virtual bool halt() = 0;

        /**
         * @brief Read drive status via SDO.
         * @return NanotecStatus with statusword, position, velocity, mode.
         */
        virtual NanotecStatus readStatus() = 0;

        /**
         * @brief Save all parameters to drive NVM (0x1010:01 = "save").
         * @return true on success.
         */
        virtual bool save() = 0;
    };

} // namespace Interfaces::HLDriver
