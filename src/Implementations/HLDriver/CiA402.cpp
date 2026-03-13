#include "Implementations/HLDriver/CiA402.hpp"

#include <cstdio>
#include <cstring>

extern "C"
{
#include <tx_api.h>
}

namespace Implementations::HLDriver
{

    using State = Interfaces::HLDriver::CiA402State;
    using Mode = Interfaces::HLDriver::OperationMode;

    /* ---------------------------------------------------------------------- */
    /* Construction                                                           */
    /* ---------------------------------------------------------------------- */
    CiA402::CiA402(Interfaces::HLDriver::ICANopen &canopen)
        : canopen_{canopen}
    {
    }

    /* ---------------------------------------------------------------------- */
    /* State decode                                                           */
    /* ---------------------------------------------------------------------- */
    State CiA402::decodeState(uint16_t sw)
    {
        /* Mask bits 6..0 per CiA 402 state encoding table */
        const uint16_t masked = sw & 0x006FU;

        if ((masked & 0x004F) == 0x0000)
        {
            return State::NotReadyToSwitchOn;
        }
        if ((masked & 0x004F) == 0x0040)
        {
            return State::SwitchOnDisabled;
        }
        if ((masked & 0x006F) == 0x0021)
        {
            return State::ReadyToSwitchOn;
        }
        if ((masked & 0x006F) == 0x0023)
        {
            return State::SwitchedOn;
        }
        if ((masked & 0x006F) == 0x0027)
        {
            return State::OperationEnabled;
        }
        if ((masked & 0x006F) == 0x0007)
        {
            return State::QuickStopActive;
        }
        if ((masked & 0x004F) == 0x000F)
        {
            return State::FaultReactionActive;
        }
        if ((masked & 0x004F) == 0x0008)
        {
            return State::Fault;
        }
        return State::Unknown;
    }

    /* ---------------------------------------------------------------------- */
    /* Local OD accessors (PDO data)                                          */
    /* All OD_RAM fields mapped to PDOs must be accessed under CO_LOCK_OD     */
    /* to prevent torn reads/writes vs the CANopen processing thread.         */
    /* ---------------------------------------------------------------------- */
    void CiA402::setControlword(uint16_t cw)
    {
        CO_LOCK_OD(nullptr);
        OD_RAM.x2000_controlword = cw;
        CO_UNLOCK_OD(nullptr);
    }

    uint16_t CiA402::readStatusword() const
    {
        CO_LOCK_OD(nullptr);
        uint16_t sw = OD_RAM.x2004_statusword;
        CO_UNLOCK_OD(nullptr);
        return sw;
    }

    /* ---------------------------------------------------------------------- */
    /* SDO typed helpers                                                      */
    /* ---------------------------------------------------------------------- */
    bool CiA402::sdoWriteU8(uint16_t index, uint8_t sub, uint8_t val)
    {
        return canopen_.sdoWrite(driveNodeId_, index, sub, {&val, 1}).success;
    }

    bool CiA402::sdoWriteU16(uint16_t index, uint8_t sub, uint16_t val)
    {
        uint8_t buf[2];
        std::memcpy(buf, &val, 2);
        return canopen_.sdoWrite(driveNodeId_, index, sub, {buf, 2}).success;
    }

    bool CiA402::sdoWriteU32(uint16_t index, uint8_t sub, uint32_t val)
    {
        uint8_t buf[4];
        std::memcpy(buf, &val, 4);
        return canopen_.sdoWrite(driveNodeId_, index, sub, {buf, 4}).success;
    }

    /* ---------------------------------------------------------------------- */
    /* Wait for a specific CiA 402 state                                      */
    /* ---------------------------------------------------------------------- */
    bool CiA402::waitForState(State target, uint32_t timeout_ms)
    {
        uint32_t elapsed = 0;
        while (elapsed < timeout_ms)
        {
            tx_thread_sleep(POLL_INTERVAL_MS);
            elapsed += POLL_INTERVAL_MS;

            State cur = decodeState(readStatusword());
            if (cur == target)
            {
                return true;
            }
            /* Abort early on fault */
            if (cur == State::Fault || cur == State::FaultReactionActive)
            {
                printf("CiA402: fault detected during state transition\r\n");
                return false;
            }
        }
        printf("CiA402: state transition timeout (wanted %u, got %u)\r\n",
               static_cast<unsigned>(target),
               static_cast<unsigned>(decodeState(readStatusword())));
        return false;
    }

    /* ---------------------------------------------------------------------- */
    /* init()                                                                 */
    /* ---------------------------------------------------------------------- */
    bool CiA402::init(uint8_t driveNodeId)
    {
        driveNodeId_ = driveNodeId;
        initialized_ = false;

        printf("CiA402: initialising drive node %u\r\n", driveNodeId);

        /* Put drive in pre-operational for SDO configuration */
        canopen_.sendNMTCommand(128, driveNodeId_); /* CO_NMT_ENTER_PRE_OPERATIONAL */
        tx_thread_sleep(50);

        /*
         * Master-side PDO mapping (TPDO1/2, RPDO1/2) is fully defined in the
         * OD generated from CiA402_Master.xpd — no runtime setup needed here.
         *
         * Drive-side PDO mapping relies on standard CiA 402 factory defaults:
         *   Drive RPDO1 (0x200+id): controlword (0x6040) + target position (0x607A)
         *   Drive RPDO2 (0x300+id): modes of operation (0x6060) + target velocity (0x60FF)
         *   Drive TPDO1 (0x180+id): statusword (0x6041) + actual position (0x6064)
         *   Drive TPDO2 (0x280+id): modes of operation display (0x6061)
         *
         * If a drive ships with non-standard defaults, add explicit
         * configureSlavePDO() calls here to override them.
         */

        // https://www.nanotec.com/fileadmin/files/Handbuecher/Handbuecher_Archiv/Plug_Drive/PD4C_CANopen_Technical-Manual_V2.0.1.pdf?1656012592

        /* ---- Configure heartbeat on the slave (produce every 500 ms) ---- */
        {
            uint16_t hbTime = 500;
            uint8_t buf[2];
            std::memcpy(buf, &hbTime, 2);
            canopen_.sdoWrite(driveNodeId_, 0x1017, 0, {buf, 2});
        }

        /* ---- Start the drive node ---- */
        canopen_.sendNMTCommand(1, driveNodeId_); /* CO_NMT_ENTER_OPERATIONAL */
        tx_thread_sleep(50);

        /* Clear local controlword */
        setControlword(0);

        initialized_ = true;
        printf("CiA402: drive node %u initialised\r\n", driveNodeId_);
        return true;
    }

    /* ---------------------------------------------------------------------- */
    /* getStatus()                                                            */
    /* ---------------------------------------------------------------------- */
    Interfaces::HLDriver::DriveStatus CiA402::getStatus() const
    {
        CO_LOCK_OD(nullptr);
        auto snapshot = status_;
        CO_UNLOCK_OD(nullptr);
        return snapshot;
    }

    /* ---------------------------------------------------------------------- */
    /* getStateLog()                                                          */
    /* ---------------------------------------------------------------------- */
    Interfaces::HLDriver::StateLog CiA402::getStateLog() const
    {
        CO_LOCK_OD(nullptr);
        auto snapshot = stateLog_;
        CO_UNLOCK_OD(nullptr);
        return snapshot;
    }

    /* ---------------------------------------------------------------------- */
    /* update()  — called each CANopen cycle (~1 ms)                          */
    /* ---------------------------------------------------------------------- */
    void CiA402::update()
    {
        if (!initialized_)
        {
            return;
        }

        /*
         * Take a consistent snapshot of all PDO-fed OD variables and update
         * the cached status under a single lock hold.  The lock is shared
         * with the serial thread (setControlword, readStatusword, getStatus,
         * moveToPosition, setTargetVelocity, setOperationMode).
         */
        CO_LOCK_OD(nullptr);
        uint16_t sw = OD_RAM.x2004_statusword;
        int32_t pos = OD_RAM.x2005_actualPosition;
        int8_t modeDisp = OD_RAM.x2006_modesOfOperationDisplay;

        State prevState = status_.state;

        status_.rawStatusword = sw;
        status_.state = decodeState(sw);
        status_.actualPosition = pos;
        status_.activeModeDisplay = modeDisp;
        status_.targetReached = (sw & SW_TARGET_REACHED) != 0;
        status_.fault = (sw & SW_FAULT) != 0;
        status_.warning = (sw & SW_WARNING) != 0;

        /* Log state transitions */
        if (status_.state != prevState)
        {
            auto &entry = stateLog_.entries[stateLog_.writeIdx];
            entry.timestamp_ms = tx_time_get();
            entry.fromState = prevState;
            entry.toState = status_.state;
            entry.statusword = sw;

            stateLog_.writeIdx = (stateLog_.writeIdx + 1U) %
                                 Interfaces::HLDriver::STATE_LOG_CAPACITY;
            if (stateLog_.count < Interfaces::HLDriver::STATE_LOG_CAPACITY)
            {
                stateLog_.count++;
            }
        }
        CO_UNLOCK_OD(nullptr);
    }

    /* ---------------------------------------------------------------------- */
    /* enable()  — walk state machine to OperationEnabled                     */
    /* ---------------------------------------------------------------------- */
    bool CiA402::enable()
    {
        if (!initialized_)
        {
            return false;
        }

        State cur = decodeState(readStatusword());

        /* From Fault: must reset first */
        if (cur == State::Fault)
        {
            printf("CiA402: drive in Fault — call faultReset() first\r\n");
            return false;
        }

        /* From QuickStopActive: disable voltage first → SwitchOnDisabled */
        if (cur == State::QuickStopActive)
        {
            setControlword(0);
            if (!waitForState(State::SwitchOnDisabled, STATE_TRANSITION_TIMEOUT_MS))
            {
                return false;
            }
            cur = State::SwitchOnDisabled;
        }

        /* Step 1: Shutdown — transition to ReadyToSwitchOn */
        if (cur == State::SwitchOnDisabled || cur == State::ReadyToSwitchOn ||
            cur == State::SwitchedOn || cur == State::OperationEnabled)
        {
            if (cur != State::ReadyToSwitchOn && cur != State::SwitchedOn &&
                cur != State::OperationEnabled)
            {
                setControlword(CW_ENABLE_VOLTAGE | CW_QUICK_STOP);
                if (!waitForState(State::ReadyToSwitchOn, STATE_TRANSITION_TIMEOUT_MS))
                {
                    return false;
                }
            }
        }

        cur = decodeState(readStatusword());

        /* Step 2: Switch On */
        if (cur == State::ReadyToSwitchOn)
        {
            setControlword(CW_SWITCH_ON | CW_ENABLE_VOLTAGE | CW_QUICK_STOP);
            if (!waitForState(State::SwitchedOn, STATE_TRANSITION_TIMEOUT_MS))
            {
                return false;
            }
        }

        cur = decodeState(readStatusword());

        /* Step 3: Enable Operation */
        if (cur == State::SwitchedOn)
        {
            setControlword(CW_SWITCH_ON | CW_ENABLE_VOLTAGE | CW_QUICK_STOP | CW_ENABLE_OPERATION);
            if (!waitForState(State::OperationEnabled, STATE_TRANSITION_TIMEOUT_MS))
            {
                return false;
            }
        }

        printf("CiA402: drive enabled (OperationEnabled)\r\n");
        return true;
    }

    /* ---------------------------------------------------------------------- */
    /* disable()                                                              */
    /* ---------------------------------------------------------------------- */
    bool CiA402::disable()
    {
        if (!initialized_)
        {
            return false;
        }

        /* Disable voltage → SwitchOnDisabled */
        setControlword(0);
        return waitForState(State::SwitchOnDisabled, STATE_TRANSITION_TIMEOUT_MS);
    }

    /* ---------------------------------------------------------------------- */
    /* quickStop()                                                            */
    /* ---------------------------------------------------------------------- */
    bool CiA402::quickStop()
    {
        if (!initialized_)
        {
            return false;
        }

        setControlword(CW_ENABLE_VOLTAGE); /* Quick stop = deassert bit 2 */
        return waitForState(State::QuickStopActive, STATE_TRANSITION_TIMEOUT_MS);
    }

    /* ---------------------------------------------------------------------- */
    /* faultReset()                                                           */
    /* ---------------------------------------------------------------------- */
    bool CiA402::faultReset()
    {
        if (!initialized_)
        {
            return false;
        }

        /* Rising edge on bit 7 — each step must persist for at least 2 SYNC cycles */
        setControlword(0);
        tx_thread_sleep(20);
        setControlword(CW_FAULT_RESET);
        tx_thread_sleep(20);
        setControlword(0);

        return waitForState(State::SwitchOnDisabled, STATE_TRANSITION_TIMEOUT_MS);
    }

    /* ---------------------------------------------------------------------- */
    /* setOperationMode()                                                     */
    /* ---------------------------------------------------------------------- */
    bool CiA402::setOperationMode(Interfaces::HLDriver::OperationMode mode)
    {
        if (!initialized_)
        {
            return false;
        }

        auto modeVal = static_cast<int8_t>(mode);

        /* Write to local OD — TPDO2 delivers it to the drive's 0x6060 */
        CO_LOCK_OD(nullptr);
        OD_RAM.x2002_modesOfOperation = modeVal;
        CO_UNLOCK_OD(nullptr);

        /* Wait for the drive to confirm via its TPDO2 → our RPDO2 */
        constexpr uint32_t timeout_ms = 2000;
        uint32_t elapsed = 0;
        while (elapsed < timeout_ms)
        {
            tx_thread_sleep(POLL_INTERVAL_MS);
            elapsed += POLL_INTERVAL_MS;

            CO_LOCK_OD(nullptr);
            int8_t confirmed = OD_RAM.x2006_modesOfOperationDisplay;
            CO_UNLOCK_OD(nullptr);

            if (confirmed == modeVal)
            {
                printf("CiA402: mode %d confirmed\r\n", modeVal);
                return true;
            }
        }

        printf("CiA402: mode change timeout (wanted %d)\r\n", modeVal);
        return false;
    }

    /* ---------------------------------------------------------------------- */
    /* moveToPosition()                                                       */
    /* ---------------------------------------------------------------------- */
    bool CiA402::moveToPosition(int32_t position, bool relative)
    {
        if (!initialized_)
        {
            return false;
        }

        {
            CO_LOCK_OD(nullptr);
            State cur = status_.state;
            CO_UNLOCK_OD(nullptr);
            if (cur != State::OperationEnabled)
            {
                return false;
            }
        }

        /* Write target position to local OD (picked up by TPDO1) */
        CO_LOCK_OD(nullptr);
        OD_RAM.x2001_targetPosition = position;
        CO_UNLOCK_OD(nullptr);

        /* Build controlword with new-setpoint bit */
        uint16_t cw = CW_SWITCH_ON | CW_ENABLE_VOLTAGE | CW_QUICK_STOP | CW_ENABLE_OPERATION | CW_NEW_SETPOINT;
        if (relative)
        {
            cw |= CW_ABS_REL;
        }
        setControlword(cw);

        /* After at least 2 SYNC cycles, clear new-setpoint so the drive doesn't restart */
        tx_thread_sleep(20);
        cw &= ~CW_NEW_SETPOINT;
        setControlword(cw);

        printf("CiA402: move to %ld %s\r\n",
               static_cast<long>(position),
               relative ? "(relative)" : "(absolute)");
        return true;
    }

    /* ---------------------------------------------------------------------- */
    /* setTargetVelocity()                                                    */
    /* ---------------------------------------------------------------------- */
    bool CiA402::setTargetVelocity(int32_t velocity)
    {
        if (!initialized_)
        {
            return false;
        }

        {
            CO_LOCK_OD(nullptr);
            State cur = status_.state;
            CO_UNLOCK_OD(nullptr);
            if (cur != State::OperationEnabled)
            {
                return false;
            }
        }

        /* Write target velocity to local OD (picked up by TPDO2) */
        CO_LOCK_OD(nullptr);
        OD_RAM.x2003_targetVelocity = velocity;
        CO_UNLOCK_OD(nullptr);

        printf("CiA402: target velocity = %ld\r\n", static_cast<long>(velocity));
        return true;
    }

} // namespace Implementations::HLDriver
