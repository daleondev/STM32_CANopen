#pragma once

#include <cstdint>

namespace Interfaces::HLDriver
{

    /**
     * @brief CiA 402 drive state machine states.
     *
     * Decoded from the statusword bits [6:0] per IEC 61800-7-201.
     */
    enum class CiA402State : uint8_t
    {
        NotReadyToSwitchOn,
        SwitchOnDisabled,
        ReadyToSwitchOn,
        SwitchedOn,
        OperationEnabled,
        QuickStopActive,
        FaultReactionActive,
        Fault,
        Unknown
    };

    /**
     * @brief CiA 402 modes of operation.
     */
    enum class OperationMode : int8_t
    {
        ProfilePosition = 1,
        ProfileVelocity = 3
    };

    /**
     * @brief Snapshot of the drive's current status.
     *
     * Populated from PDO data (statusword + actual position) — essentially free
     * to query since no CAN traffic is generated.
     */
    struct DriveStatus
    {
        CiA402State state{CiA402State::Unknown};
        uint16_t rawStatusword{0};
        int32_t actualPosition{0};
        int8_t activeModeDisplay{0};
        bool targetReached{false};
        bool fault{false};
        bool warning{false};
    };

    /**
     * @brief High-level CiA 402 drive profile interface.
     *
     * Commands a single servo / stepper drive over CANopen using the CiA 402
     * state machine.  Configuration and mode changes use SDO (blocking);
     * real-time controlword / statusword exchange uses PDO at the stack cycle
     * rate (~1 ms).
     */
    class ICiA402
    {
    public:
        virtual ~ICiA402() = default;

        /**
         * @brief Initialise the drive.
         *
         * Configures the remote node's PDO mapping via SDO, sets up heartbeat
         * monitoring, and transitions the node to Operational (NMT Start).
         *
         * @param driveNodeId  CAN node ID of the target drive (1..127).
         * @return true on success.
         *
         * @note Blocking — must be called from a low-priority thread (e.g. CLI).
         */
        virtual bool init(uint8_t driveNodeId) = 0;

        /**
         * @brief Return the current drive status (from PDO-fed OD variables).
         *
         * This is a non-blocking read of local RAM only.
         */
        virtual DriveStatus getStatus() const = 0;

        /**
         * @brief Walk the state machine to OperationEnabled.
         *
         * Transitions: SwitchOnDisabled → ReadyToSwitchOn → SwitchedOn →
         * OperationEnabled.  Polls statusword between transitions.
         *
         * @return true if OperationEnabled was reached.
         *
         * @note Blocking — waits for state transitions via PDO.
         */
        virtual bool enable() = 0;

        /**
         * @brief Transition to SwitchOnDisabled (servo off).
         */
        virtual bool disable() = 0;

        /**
         * @brief Issue a quick-stop (state → QuickStopActive).
         */
        virtual bool quickStop() = 0;

        /**
         * @brief Clear a fault condition (Fault → SwitchOnDisabled).
         */
        virtual bool faultReset() = 0;

        /**
         * @brief Set the operating mode on the drive (Profile Position, Profile
         *        Velocity, etc.) via PDO (local OD → TPDO2 → drive 0x6060).
         *
         * Blocks until the drive confirms the mode via modesOfOperationDisplay
         * (RPDO2) or times out.
         *
         * @note Blocking — polls confirmation for up to 2 s.
         */
        virtual bool setOperationMode(OperationMode mode) = 0;

        /**
         * @brief Command a profile-position move.
         *
         * Sets target position in the local OD (TPDO) and toggles the
         * new-setpoint bit in the controlword.
         *
         * @param position  Target position in drive units.
         * @param relative  If true, position is relative to current.
         * @return true if the command was accepted (drive in OperationEnabled).
         */
        virtual bool moveToPosition(int32_t position, bool relative = false) = 0;

        /**
         * @brief Set the target velocity for Profile Velocity mode.
         *
         * @param velocity  Target velocity in drive units.
         * @return true if the command was accepted (drive in OperationEnabled).
         */
        virtual bool setTargetVelocity(int32_t velocity) = 0;

        /**
         * @brief Cyclic update — call once per CANopen processing cycle.
         *
         * Reads statusword / actual position from the local OD (fed by RPDO),
         * and updates the internal DriveStatus cache.  This is the fast path
         * and must be non-blocking.
         */
        virtual void update() = 0;
    };

} // namespace Interfaces::HLDriver
