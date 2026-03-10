#include "Implementations/HLDriver/NanotecPnD.hpp"

#include <cstdio>
#include <cstring>

extern "C"
{
#include <tx_api.h>
}

namespace Implementations::HLDriver
{

    /* ---------------------------------------------------------------------- */
    /* Construction                                                           */
    /* ---------------------------------------------------------------------- */
    NanotecPnD::NanotecPnD(Interfaces::HLDriver::ICANopen &canopen)
        : canopen_{canopen}
    {
    }

    /* ---------------------------------------------------------------------- */
    /* SDO typed helpers                                                      */
    /* ---------------------------------------------------------------------- */
    bool NanotecPnD::sdoWriteU8(uint16_t index, uint8_t sub, uint8_t val)
    {
        return canopen_.sdoWrite(nodeId_, index, sub, {&val, 1}).success;
    }

    bool NanotecPnD::sdoWriteU16(uint16_t index, uint8_t sub, uint16_t val)
    {
        uint8_t buf[2];
        std::memcpy(buf, &val, 2);
        return canopen_.sdoWrite(nodeId_, index, sub, {buf, 2}).success;
    }

    bool NanotecPnD::sdoWriteU32(uint16_t index, uint8_t sub, uint32_t val)
    {
        uint8_t buf[4];
        std::memcpy(buf, &val, 4);
        return canopen_.sdoWrite(nodeId_, index, sub, {buf, 4}).success;
    }

    bool NanotecPnD::sdoWriteI8(uint16_t index, uint8_t sub, int8_t val)
    {
        uint8_t u;
        std::memcpy(&u, &val, 1);
        return canopen_.sdoWrite(nodeId_, index, sub, {&u, 1}).success;
    }

    bool NanotecPnD::sdoWriteI32(uint16_t index, uint8_t sub, int32_t val)
    {
        uint8_t buf[4];
        std::memcpy(buf, &val, 4);
        return canopen_.sdoWrite(nodeId_, index, sub, {buf, 4}).success;
    }

    bool NanotecPnD::sdoReadU16(uint16_t index, uint8_t sub, uint16_t &out)
    {
        uint8_t buf[2]{};
        auto r = canopen_.sdoRead(nodeId_, index, sub, buf);
        if (!r.success)
        {
            return false;
        }
        std::memcpy(&out, buf, 2);
        return true;
    }

    bool NanotecPnD::sdoReadI32(uint16_t index, uint8_t sub, int32_t &out)
    {
        uint8_t buf[4]{};
        auto r = canopen_.sdoRead(nodeId_, index, sub, buf);
        if (!r.success)
        {
            return false;
        }
        std::memcpy(&out, buf, 4);
        return true;
    }

    bool NanotecPnD::sdoReadI8(uint16_t index, uint8_t sub, int8_t &out)
    {
        uint8_t buf[1]{};
        auto r = canopen_.sdoRead(nodeId_, index, sub, buf);
        if (!r.success)
        {
            return false;
        }
        std::memcpy(&out, buf, 1);
        return true;
    }

    bool NanotecPnD::writeControlword(uint16_t cw)
    {
        return sdoWriteU16(OD_CONTROLWORD, 0, cw);
    }

    bool NanotecPnD::readStatusword(uint16_t &sw)
    {
        return sdoReadU16(OD_STATUSWORD, 0, sw);
    }

    void NanotecPnD::delayMs(uint32_t ms)
    {
        constexpr auto ticks_per_ms = TX_TIMER_TICKS_PER_SECOND / 1000;
        tx_thread_sleep(ms * ticks_per_ms);
    }

    /* ---------------------------------------------------------------------- */
    /* Poll statusword for a bit pattern                                      */
    /* ---------------------------------------------------------------------- */
    bool NanotecPnD::pollStatusword(uint16_t mask, uint16_t expected,
                                    uint32_t timeout_ms)
    {
        uint32_t elapsed = 0;
        while (elapsed < timeout_ms)
        {
            uint16_t sw = 0;
            if (readStatusword(sw) && (sw & mask) == expected)
            {
                return true;
            }
            delayMs(POLL_MS);
            elapsed += POLL_MS;
        }
        return false;
    }

    /* ---------------------------------------------------------------------- */
    /* CiA 402 state machine walk (SDO-based)                                 */
    /* ---------------------------------------------------------------------- */
    bool NanotecPnD::walkToOperationEnabled()
    {
        uint16_t sw = 0;
        if (!readStatusword(sw))
        {
            printf("  [PnD] cannot read statusword\r\n");
            return false;
        }
        printf("  [PnD] statusword = 0x%04X\r\n", sw);

        /* If fault, attempt reset first */
        if (sw & SW_FAULT)
        {
            printf("  [PnD] fault detected — resetting\r\n");
            if (!faultReset())
            {
                return false;
            }
            if (!readStatusword(sw))
            {
                return false;
            }
        }

        /* Transition: → ReadyToSwitchOn  (Shutdown command) */
        printf("  [PnD] → Shutdown (0x0006)\r\n");
        if (!writeControlword(CW_QUICK_STOP | CW_ENABLE_VOLTAGE))
        {
            return false;
        }
        if (!pollStatusword(0x006F, 0x0021, STATE_TIMEOUT_MS))
        {
            printf("  [PnD] timeout waiting for ReadyToSwitchOn\r\n");
            return false;
        }

        /* Transition: → SwitchedOn  (Switch On command) */
        printf("  [PnD] → SwitchOn (0x0007)\r\n");
        if (!writeControlword(CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON))
        {
            return false;
        }
        if (!pollStatusword(0x006F, 0x0023, STATE_TIMEOUT_MS))
        {
            printf("  [PnD] timeout waiting for SwitchedOn\r\n");
            return false;
        }

        /* Transition: → OperationEnabled  (Enable Operation command) */
        printf("  [PnD] → EnableOp (0x000F)\r\n");
        if (!writeControlword(CW_QUICK_STOP | CW_ENABLE_VOLTAGE |
                              CW_SWITCH_ON | CW_ENABLE_OPERATION))
        {
            return false;
        }
        if (!pollStatusword(0x006F, 0x0027, STATE_TIMEOUT_MS))
        {
            printf("  [PnD] timeout waiting for OperationEnabled\r\n");
            return false;
        }

        printf("  [PnD] OperationEnabled reached\r\n");
        return true;
    }

    /* ---------------------------------------------------------------------- */
    /* Public API                                                             */
    /* ---------------------------------------------------------------------- */

    bool NanotecPnD::configure(uint8_t nodeId,
                               uint32_t peakCurrent_mA,
                               uint32_t nomCurrent_mA)
    {
        nodeId_ = nodeId;

        printf("[PnD] configuring node %u (peak=%lu mA, nom=%lu mA)\r\n",
               nodeId,
               static_cast<unsigned long>(peakCurrent_mA),
               static_cast<unsigned long>(nomCurrent_mA));

        /* NMT → Pre-Operational (for configuration) */
        if (!canopen_.sendNMTCommand(128, nodeId_))
        {
            printf("[PnD] NMT pre-op failed\r\n");
            return false;
        }
        delayMs(100);

        /* Write motor current limits */
        if (!sdoWriteU32(OD_PEAK_CURRENT, 0, peakCurrent_mA))
        {
            printf("[PnD] failed to write peak current (0x2031)\r\n");
            return false;
        }
        if (!sdoWriteU32(OD_NOMINAL_CURRENT, 0, nomCurrent_mA))
        {
            printf("[PnD] failed to write nominal current (0x2032)\r\n");
            return false;
        }

        /* NMT → Operational */
        if (!canopen_.sendNMTCommand(1, nodeId_))
        {
            printf("[PnD] NMT start failed\r\n");
            return false;
        }
        delayMs(100);

        configured_ = true;
        printf("[PnD] configuration OK\r\n");
        return true;
    }

    bool NanotecPnD::autoSetup()
    {
        if (!configured_)
        {
            printf("[PnD] call configure() first\r\n");
            return false;
        }

        printf("[PnD] starting auto setup — motor WILL move!\r\n");

        /* Set mode of operation to Auto Setup (-1) */
        if (!sdoWriteI8(OD_MODES_OF_OPERATION, 0, MODE_AUTO_SETUP))
        {
            printf("[PnD] failed to set auto setup mode\r\n");
            return false;
        }
        delayMs(50);

        /* Walk state machine to OperationEnabled → triggers auto setup */
        if (!walkToOperationEnabled())
        {
            printf("[PnD] failed to enable drive for auto setup\r\n");
            return false;
        }

        /* Poll until target-reached (bit 10) signals completion */
        printf("[PnD] waiting for auto setup to complete (up to 60 s)...\r\n");
        if (!pollStatusword(SW_TARGET_REACHED, SW_TARGET_REACHED,
                            AUTO_SETUP_TIMEOUT_MS))
        {
            printf("[PnD] auto setup timed out!\r\n");
            return false;
        }

        printf("[PnD] auto setup complete\r\n");

        /* Disable drive after auto setup */
        writeControlword(0x0000);
        delayMs(100);

        return true;
    }

    bool NanotecPnD::enable()
    {
        if (!configured_)
        {
            printf("[PnD] call configure() first\r\n");
            return false;
        }
        return walkToOperationEnabled();
    }

    bool NanotecPnD::disable()
    {
        /* Shutdown command → ReadyToSwitchOn */
        printf("[PnD] disabling\r\n");
        return writeControlword(CW_QUICK_STOP | CW_ENABLE_VOLTAGE);
    }

    bool NanotecPnD::quickStop()
    {
        printf("[PnD] quick stop\r\n");
        return writeControlword(CW_ENABLE_VOLTAGE); /* Quick stop = QS bit low */
    }

    bool NanotecPnD::faultReset()
    {
        printf("[PnD] fault reset\r\n");
        if (!writeControlword(CW_FAULT_RESET))
        {
            return false;
        }
        delayMs(100);
        /* Clear the fault reset bit */
        writeControlword(0x0000);
        delayMs(100);

        uint16_t sw = 0;
        if (readStatusword(sw) && !(sw & SW_FAULT))
        {
            printf("[PnD] fault cleared\r\n");
            return true;
        }
        printf("[PnD] fault still present (sw=0x%04X)\r\n", sw);
        return false;
    }

    bool NanotecPnD::setMode(int8_t mode)
    {
        printf("[PnD] setting mode %d\r\n", mode);
        if (!sdoWriteI8(OD_MODES_OF_OPERATION, 0, mode))
        {
            printf("[PnD] failed to set mode\r\n");
            return false;
        }
        delayMs(50);

        /* Verify */
        int8_t actual = 0;
        if (sdoReadI8(OD_MODES_DISPLAY, 0, actual))
        {
            printf("[PnD] modes display = %d\r\n", actual);
        }
        return true;
    }

    bool NanotecPnD::setProfile(uint32_t velocity,
                                uint32_t acceleration,
                                uint32_t deceleration)
    {
        printf("[PnD] profile: vel=%lu acc=%lu dec=%lu\r\n",
               static_cast<unsigned long>(velocity),
               static_cast<unsigned long>(acceleration),
               static_cast<unsigned long>(deceleration));

        bool ok = true;
        ok = sdoWriteU32(OD_PROFILE_VELOCITY, 0, velocity) && ok;
        ok = sdoWriteU32(OD_PROFILE_ACCEL, 0, acceleration) && ok;
        ok = sdoWriteU32(OD_PROFILE_DECEL, 0, deceleration) && ok;
        return ok;
    }

    bool NanotecPnD::moveAbsolute(int32_t position)
    {
        printf("[PnD] move absolute → %ld\r\n", static_cast<long>(position));

        if (!sdoWriteI32(OD_TARGET_POSITION, 0, position))
        {
            return false;
        }

        /* Controlword: enabled + new setpoint + absolute */
        uint16_t cw = CW_SWITCH_ON | CW_ENABLE_VOLTAGE | CW_QUICK_STOP |
                      CW_ENABLE_OPERATION | CW_NEW_SETPOINT;
        if (!writeControlword(cw))
        {
            return false;
        }

        /* Release new-setpoint bit after brief delay */
        delayMs(10);
        cw &= ~CW_NEW_SETPOINT;
        return writeControlword(cw);
    }

    bool NanotecPnD::moveRelative(int32_t distance)
    {
        printf("[PnD] move relative → %ld\r\n", static_cast<long>(distance));

        if (!sdoWriteI32(OD_TARGET_POSITION, 0, distance))
        {
            return false;
        }

        /* Controlword: enabled + new setpoint + relative */
        uint16_t cw = CW_SWITCH_ON | CW_ENABLE_VOLTAGE | CW_QUICK_STOP |
                      CW_ENABLE_OPERATION | CW_NEW_SETPOINT | CW_ABS_REL;
        if (!writeControlword(cw))
        {
            return false;
        }

        delayMs(10);
        cw &= ~CW_NEW_SETPOINT;
        return writeControlword(cw);
    }

    bool NanotecPnD::setVelocity(int32_t velocity)
    {
        printf("[PnD] velocity → %ld\r\n", static_cast<long>(velocity));
        return sdoWriteI32(OD_TARGET_VELOCITY, 0, velocity);
    }

    bool NanotecPnD::halt()
    {
        printf("[PnD] halt\r\n");
        uint16_t cw = CW_SWITCH_ON | CW_ENABLE_VOLTAGE | CW_QUICK_STOP |
                      CW_ENABLE_OPERATION | CW_HALT;
        return writeControlword(cw);
    }

    Interfaces::HLDriver::NanotecStatus NanotecPnD::readStatus()
    {
        Interfaces::HLDriver::NanotecStatus st{};
        sdoReadU16(OD_STATUSWORD, 0, st.statusword);
        sdoReadI32(OD_POSITION_ACTUAL, 0, st.position);
        sdoReadI32(OD_VELOCITY_ACTUAL, 0, st.velocity);
        sdoReadI8(OD_MODES_DISPLAY, 0, st.modeDisplay);
        return st;
    }

    bool NanotecPnD::save()
    {
        printf("[PnD] saving parameters to NVM...\r\n");

        /* Magic signature "save" = 0x65766173 */
        constexpr uint32_t SAVE_SIGNATURE = 0x65766173U;
        if (!sdoWriteU32(OD_STORE_PARAMS, 1, SAVE_SIGNATURE))
        {
            printf("[PnD] save FAILED\r\n");
            return false;
        }

        /* Give the drive time to write flash */
        delayMs(500);
        printf("[PnD] save OK\r\n");
        return true;
    }

} // namespace Implementations::HLDriver
