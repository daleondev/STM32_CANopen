#include "SerialCLI.hpp"

#include <array>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "Diagnostics.hpp"

extern "C"
{
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"
}

namespace SerialCLI
{

    /* -------------------------------------------------------------------------- */
    /* Module state                                                               */
    /* -------------------------------------------------------------------------- */
    static Interfaces::HLDriver::ICANopen *s_canopen = nullptr;
    static Interfaces::HLDriver::ICiA402 *s_motor = nullptr;
    static Interfaces::HLDriver::INanotecPnD *s_nanotec = nullptr;

    /* -------------------------------------------------------------------------- */
    /* UART helpers                                                               */
    /* -------------------------------------------------------------------------- */
    static constexpr size_t MAX_CMD_LEN = 128;

    /**
     * @brief Read a line from UART (blocking, with echo).
     * @return Number of characters read (excluding null terminator).
     */
    static size_t readLine(char *buf, size_t maxLen)
    {
        size_t pos = 0;
        while (pos < maxLen - 1)
        {
            uint8_t ch = 0;

            /* Blocking receive, one byte at a time */
            if (HAL_UART_Receive(&hcom_uart[COM1], &ch, 1, HAL_MAX_DELAY) != HAL_OK)
            {
                continue;
            }

            /* Echo */
            HAL_UART_Transmit(&hcom_uart[COM1], &ch, 1, HAL_MAX_DELAY);

            if (ch == '\r' || ch == '\n')
            {
                printf("\r\n");
                break;
            }
            else if (ch == '\b' || ch == 127) /* Backspace / DEL */
            {
                if (pos > 0)
                {
                    pos--;
                    printf("\b \b");
                }
            }
            else
            {
                buf[pos++] = static_cast<char>(ch);
            }
        }
        buf[pos] = '\0';
        return pos;
    }

    /**
     * @brief Parse a hex or decimal number from string.
     */
    static uint32_t parseNumber(const char *str)
    {
        if (str == nullptr)
        {
            return 0;
        }
        if (str[0] == '0' && (str[1] == 'x' || str[1] == 'X'))
        {
            return static_cast<uint32_t>(strtoul(str, nullptr, 16));
        }
        return static_cast<uint32_t>(strtoul(str, nullptr, 10));
    }

    /* -------------------------------------------------------------------------- */
    /* Command handlers                                                           */
    /* -------------------------------------------------------------------------- */
    static void cmdHelp()
    {
        printf("Available commands:\r\n");
        printf("  help                              - Show this help\r\n");
        printf("  status                            - Show CANopen status\r\n");
        printf("  nmt <start|stop|preop|reset> <id> - Send NMT command (id=0 for all)\r\n");
        printf("  sdo read <nodeId> <index> <sub>   - SDO upload (read from remote)\r\n");
        printf("  sdo write <nodeId> <idx> <sub> <byte0> [byte1..] - SDO download\r\n");
        if (s_motor != nullptr)
        {
            printf("Motor commands:\r\n");
            printf("  motor init                        - Init drive (configure PDOs, go operational)\r\n");
            printf("  motor enable                      - Enable drive (OperationEnabled)\r\n");
            printf("  motor disable                     - Disable drive\r\n");
            printf("  motor stop                        - Quick stop\r\n");
            printf("  motor status                      - Show drive status\r\n");
            printf("  motor mode <pp|pv>                - Set operation mode\r\n");
            printf("  motor move <pos> [rel]            - Move to position (optional: rel)\r\n");
            printf("  motor velocity <vel>              - Set target velocity\r\n");
            printf("  motor fault-reset                 - Reset fault\r\n");
        }
        if (s_nanotec != nullptr)
        {
            printf("Nanotec Plug & Drive commands:\r\n");
            printf("  nt config <nodeId> <peakI> <nomI> - Configure (current in mA)\r\n");
            printf("  nt autosetup                      - Run auto setup (motor moves!)\r\n");
            printf("  nt enable                         - Enable drive\r\n");
            printf("  nt disable                        - Disable drive\r\n");
            printf("  nt stop                           - Quick stop\r\n");
            printf("  nt reset                          - Fault reset\r\n");
            printf("  nt mode <mode>                    - Set mode (-1=auto,1=pp,3=pv)\r\n");
            printf("  nt profile <vel> <acc> <dec>      - Set profile params\r\n");
            printf("  nt move <pos> [rel]               - Position move\r\n");
            printf("  nt vel <velocity>                 - Set target velocity\r\n");
            printf("  nt halt                           - Halt motion\r\n");
            printf("  nt status                         - Read drive status\r\n");
            printf("  nt save                           - Save params to NVM\r\n");
        }
        printf("Diagnostics commands:\r\n");
        printf("  diag bus                          - CAN bus health\r\n");
        printf("  diag stack                        - CANopen stack status\r\n");
        printf("  diag emcy                         - Emergency history\r\n");
        printf("  diag hb                           - Heartbeat consumer status\r\n");
        printf("  diag pdo                          - PDO values & mapping\r\n");
        printf("  diag sync                         - SYNC producer status\r\n");
        printf("  diag drive [nodeId]               - Drive errors via SDO (default: 1)\r\n");
        printf("  diag all [nodeId]                 - Full diagnostic dump\r\n");
        printf("  diag statelog                     - CiA 402 state transition history\r\n");
        printf("  diag watch [nodeId] [N] [ms]      - Continuous monitoring\r\n");
    }

    static void cmdStatus()
    {
        if (s_canopen == nullptr)
        {
            printf("CANopen: not initialized\r\n");
            return;
        }
        printf("CANopen running: %s\r\n", s_canopen->isRunning() ? "yes" : "no");
    }

    static void cmdNMT(char *args)
    {
        if (s_canopen == nullptr)
        {
            printf("CANopen not initialized\r\n");
            return;
        }

        char *cmd = strtok(args, " ");
        char *idStr = strtok(nullptr, " ");

        if (cmd == nullptr || idStr == nullptr)
        {
            printf("Usage: nmt <start|stop|preop|reset|resetcomm> <nodeId>\r\n");
            return;
        }

        uint8_t nodeId = static_cast<uint8_t>(parseNumber(idStr));
        uint8_t nmtCmd = 0;

        if (strcmp(cmd, "start") == 0)
        {
            nmtCmd = 1; /* CO_NMT_ENTER_OPERATIONAL */
        }
        else if (strcmp(cmd, "stop") == 0)
        {
            nmtCmd = 2; /* CO_NMT_ENTER_STOPPED */
        }
        else if (strcmp(cmd, "preop") == 0)
        {
            nmtCmd = 128; /* CO_NMT_ENTER_PRE_OPERATIONAL */
        }
        else if (strcmp(cmd, "reset") == 0)
        {
            nmtCmd = 129; /* CO_NMT_RESET_NODE */
        }
        else if (strcmp(cmd, "resetcomm") == 0)
        {
            nmtCmd = 130; /* CO_NMT_RESET_COMMUNICATION */
        }
        else
        {
            printf("Unknown NMT command: %s\r\n", cmd);
            return;
        }

        bool ok = s_canopen->sendNMTCommand(nmtCmd, nodeId);
        printf("NMT %s to node %u: %s\r\n", cmd, nodeId, ok ? "OK" : "FAIL");
    }

    static void cmdSDORead(char *args)
    {
        if (s_canopen == nullptr)
        {
            printf("CANopen not initialized\r\n");
            return;
        }

        char *nodeStr = strtok(args, " ");
        char *indexStr = strtok(nullptr, " ");
        char *subStr = strtok(nullptr, " ");

        if (nodeStr == nullptr || indexStr == nullptr || subStr == nullptr)
        {
            printf("Usage: sdo read <nodeId> <index> <subIndex>\r\n");
            return;
        }

        uint8_t nodeId = static_cast<uint8_t>(parseNumber(nodeStr));
        uint16_t index = static_cast<uint16_t>(parseNumber(indexStr));
        uint8_t subIndex = static_cast<uint8_t>(parseNumber(subStr));

        std::array<uint8_t, 64> buf{};
        auto result = s_canopen->sdoRead(nodeId, index, subIndex, buf);

        if (result.success)
        {
            printf("SDO read OK (%zu bytes): ", result.bytesTransferred);
            for (size_t i = 0; i < result.bytesTransferred; i++)
            {
                printf("%02X ", buf[i]);
            }
            printf("\r\n");
        }
        else
        {
            printf("SDO read FAILED (abort=0x%08lX)\r\n", static_cast<unsigned long>(result.abortCode));
        }
    }

    static void cmdSDOWrite(char *args)
    {
        if (s_canopen == nullptr)
        {
            printf("CANopen not initialized\r\n");
            return;
        }

        char *nodeStr = strtok(args, " ");
        char *indexStr = strtok(nullptr, " ");
        char *subStr = strtok(nullptr, " ");

        if (nodeStr == nullptr || indexStr == nullptr || subStr == nullptr)
        {
            printf("Usage: sdo write <nodeId> <index> <subIndex> <byte0> [byte1..]\r\n");
            return;
        }

        uint8_t nodeId = static_cast<uint8_t>(parseNumber(nodeStr));
        uint16_t index = static_cast<uint16_t>(parseNumber(indexStr));
        uint8_t subIndex = static_cast<uint8_t>(parseNumber(subStr));

        /* Parse data bytes */
        std::array<uint8_t, 64> data{};
        size_t dataLen = 0;

        char *byteStr = strtok(nullptr, " ");
        while (byteStr != nullptr && dataLen < data.size())
        {
            data[dataLen++] = static_cast<uint8_t>(parseNumber(byteStr));
            byteStr = strtok(nullptr, " ");
        }

        if (dataLen == 0)
        {
            printf("No data bytes provided\r\n");
            return;
        }

        auto result = s_canopen->sdoWrite(nodeId, index, subIndex, std::span{data.data(), dataLen});

        if (result.success)
        {
            printf("SDO write OK (%zu bytes)\r\n", result.bytesTransferred);
        }
        else
        {
            printf("SDO write FAILED (abort=0x%08lX)\r\n", static_cast<unsigned long>(result.abortCode));
        }
    }

    /* -------------------------------------------------------------------------- */
    /* Motor command handlers                                                     */
    /* -------------------------------------------------------------------------- */
    static constexpr uint8_t DRIVE_NODE_ID = 1;

    static const char *stateToString(Interfaces::HLDriver::CiA402State state)
    {
        using S = Interfaces::HLDriver::CiA402State;
        switch (state)
        {
        case S::NotReadyToSwitchOn:
            return "NotReadyToSwitchOn";
        case S::SwitchOnDisabled:
            return "SwitchOnDisabled";
        case S::ReadyToSwitchOn:
            return "ReadyToSwitchOn";
        case S::SwitchedOn:
            return "SwitchedOn";
        case S::OperationEnabled:
            return "OperationEnabled";
        case S::QuickStopActive:
            return "QuickStopActive";
        case S::FaultReactionActive:
            return "FaultReactionActive";
        case S::Fault:
            return "Fault";
        default:
            return "Unknown";
        }
    }

    static void cmdMotor(char *args)
    {
        if (s_motor == nullptr)
        {
            printf("Motor driver not available\r\n");
            return;
        }

        char *subcmd = strtok(args, " ");
        if (subcmd == nullptr)
        {
            printf("Usage: motor <init|enable|disable|stop|status|mode|move|velocity|fault-reset>\r\n");
            return;
        }

        if (strcmp(subcmd, "init") == 0)
        {
            bool ok = s_motor->init(DRIVE_NODE_ID);
            printf("Motor init: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "enable") == 0)
        {
            bool ok = s_motor->enable();
            printf("Motor enable: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "disable") == 0)
        {
            bool ok = s_motor->disable();
            printf("Motor disable: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "stop") == 0)
        {
            bool ok = s_motor->quickStop();
            printf("Motor quick stop: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "status") == 0)
        {
            auto st = s_motor->getStatus();
            printf("Drive state:    %s\r\n", stateToString(st.state));
            printf("Statusword:     0x%04X\r\n", st.rawStatusword);
            printf("Actual pos:     %ld\r\n", static_cast<long>(st.actualPosition));
            printf("Active mode:    %d\r\n", st.activeModeDisplay);
            printf("Target reached: %s\r\n", st.targetReached ? "yes" : "no");
            printf("Fault:          %s\r\n", st.fault ? "YES" : "no");
            printf("Warning:        %s\r\n", st.warning ? "YES" : "no");
        }
        else if (strcmp(subcmd, "mode") == 0)
        {
            char *modeStr = strtok(nullptr, " ");
            if (modeStr == nullptr)
            {
                printf("Usage: motor mode <pp|pv>\r\n");
                return;
            }

            using Mode = Interfaces::HLDriver::OperationMode;
            Mode mode;
            if (strcmp(modeStr, "pp") == 0)
            {
                mode = Mode::ProfilePosition;
            }
            else if (strcmp(modeStr, "pv") == 0)
            {
                mode = Mode::ProfileVelocity;
            }
            else
            {
                printf("Unknown mode: %s (use pp or pv)\r\n", modeStr);
                return;
            }

            bool ok = s_motor->setOperationMode(mode);
            printf("Set mode: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "move") == 0)
        {
            char *posStr = strtok(nullptr, " ");
            char *relStr = strtok(nullptr, " ");
            if (posStr == nullptr)
            {
                printf("Usage: motor move <position> [rel]\r\n");
                return;
            }

            auto pos = static_cast<int32_t>(strtol(posStr, nullptr, 10));
            bool relative = (relStr != nullptr && strcmp(relStr, "rel") == 0);

            bool ok = s_motor->moveToPosition(pos, relative);
            printf("Move: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "velocity") == 0)
        {
            char *velStr = strtok(nullptr, " ");
            if (velStr == nullptr)
            {
                printf("Usage: motor velocity <value>\r\n");
                return;
            }

            auto vel = static_cast<int32_t>(strtol(velStr, nullptr, 10));
            bool ok = s_motor->setTargetVelocity(vel);
            printf("Velocity: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "fault-reset") == 0)
        {
            bool ok = s_motor->faultReset();
            printf("Fault reset: %s\r\n", ok ? "OK" : "FAIL");
        }
        else
        {
            printf("Unknown motor command: %s\r\n", subcmd);
        }
    }

    /* -------------------------------------------------------------------------- */
    /* Nanotec Plug & Drive command handlers                                    */
    /* -------------------------------------------------------------------------- */
    static void cmdNanotec(char *args)
    {
        if (s_nanotec == nullptr)
        {
            printf("Nanotec driver not available\r\n");
            return;
        }

        char *subcmd = strtok(args, " ");
        if (subcmd == nullptr)
        {
            printf("Usage: nt <config|autosetup|enable|disable|stop|reset|mode|"
                   "profile|move|vel|halt|status|save>\r\n");
            return;
        }

        if (strcmp(subcmd, "config") == 0)
        {
            char *nodeStr = strtok(nullptr, " ");
            char *peakStr = strtok(nullptr, " ");
            char *nomStr = strtok(nullptr, " ");
            if (nodeStr == nullptr || peakStr == nullptr || nomStr == nullptr)
            {
                printf("Usage: nt config <nodeId> <peakCurrent_mA> <nomCurrent_mA>\r\n");
                return;
            }
            auto nodeId = static_cast<uint8_t>(parseNumber(nodeStr));
            auto peak = parseNumber(peakStr);
            auto nom = parseNumber(nomStr);
            bool ok = s_nanotec->configure(nodeId, peak, nom);
            printf("nt config: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "autosetup") == 0)
        {
            bool ok = s_nanotec->autoSetup();
            printf("nt autosetup: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "enable") == 0)
        {
            bool ok = s_nanotec->enable();
            printf("nt enable: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "disable") == 0)
        {
            bool ok = s_nanotec->disable();
            printf("nt disable: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "stop") == 0)
        {
            bool ok = s_nanotec->quickStop();
            printf("nt stop: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "reset") == 0)
        {
            bool ok = s_nanotec->faultReset();
            printf("nt reset: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "mode") == 0)
        {
            char *modeStr = strtok(nullptr, " ");
            if (modeStr == nullptr)
            {
                printf("Usage: nt mode <mode> (-1=auto, 1=pp, 3=pv)\r\n");
                return;
            }
            auto mode = static_cast<int8_t>(strtol(modeStr, nullptr, 10));
            bool ok = s_nanotec->setMode(mode);
            printf("nt mode: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "profile") == 0)
        {
            char *velStr = strtok(nullptr, " ");
            char *accStr = strtok(nullptr, " ");
            char *decStr = strtok(nullptr, " ");
            if (velStr == nullptr || accStr == nullptr || decStr == nullptr)
            {
                printf("Usage: nt profile <velocity> <accel> <decel>\r\n");
                return;
            }
            auto vel = parseNumber(velStr);
            auto acc = parseNumber(accStr);
            auto dec = parseNumber(decStr);
            bool ok = s_nanotec->setProfile(vel, acc, dec);
            printf("nt profile: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "move") == 0)
        {
            char *posStr = strtok(nullptr, " ");
            char *relStr = strtok(nullptr, " ");
            if (posStr == nullptr)
            {
                printf("Usage: nt move <position> [rel]\r\n");
                return;
            }
            auto pos = static_cast<int32_t>(strtol(posStr, nullptr, 10));
            if (relStr != nullptr && strcmp(relStr, "rel") == 0)
            {
                bool ok = s_nanotec->moveRelative(pos);
                printf("nt move rel: %s\r\n", ok ? "OK" : "FAIL");
            }
            else
            {
                bool ok = s_nanotec->moveAbsolute(pos);
                printf("nt move abs: %s\r\n", ok ? "OK" : "FAIL");
            }
        }
        else if (strcmp(subcmd, "vel") == 0)
        {
            char *velStr = strtok(nullptr, " ");
            if (velStr == nullptr)
            {
                printf("Usage: nt vel <velocity>\r\n");
                return;
            }
            auto vel = static_cast<int32_t>(strtol(velStr, nullptr, 10));
            bool ok = s_nanotec->setVelocity(vel);
            printf("nt vel: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "halt") == 0)
        {
            bool ok = s_nanotec->halt();
            printf("nt halt: %s\r\n", ok ? "OK" : "FAIL");
        }
        else if (strcmp(subcmd, "status") == 0)
        {
            auto st = s_nanotec->readStatus();
            printf("Statusword:  0x%04X\r\n", st.statusword);

            /* Decode state from statusword */
            const uint16_t sw = st.statusword;
            const char *state = "Unknown";
            uint16_t masked = sw & 0x006FU;
            if ((masked & 0x004F) == 0x0000)
                state = "NotReadyToSwitchOn";
            else if ((masked & 0x004F) == 0x0040)
                state = "SwitchOnDisabled";
            else if ((masked & 0x006F) == 0x0021)
                state = "ReadyToSwitchOn";
            else if ((masked & 0x006F) == 0x0023)
                state = "SwitchedOn";
            else if ((masked & 0x006F) == 0x0027)
                state = "OperationEnabled";
            else if ((masked & 0x006F) == 0x0007)
                state = "QuickStopActive";
            else if ((masked & 0x004F) == 0x000F)
                state = "FaultReactionActive";
            else if ((masked & 0x004F) == 0x0008)
                state = "Fault";

            printf("State:       %s\r\n", state);
            printf("Position:    %ld\r\n", static_cast<long>(st.position));
            printf("Velocity:    %ld\r\n", static_cast<long>(st.velocity));
            printf("Mode:        %d\r\n", st.modeDisplay);
            printf("Tgt reached: %s\r\n", (sw & (1U << 10)) ? "yes" : "no");
            printf("Fault:       %s\r\n", (sw & (1U << 3)) ? "YES" : "no");
            printf("Warning:     %s\r\n", (sw & (1U << 7)) ? "YES" : "no");
        }
        else if (strcmp(subcmd, "save") == 0)
        {
            bool ok = s_nanotec->save();
            printf("nt save: %s\r\n", ok ? "OK" : "FAIL");
        }
        else
        {
            printf("Unknown nt command: %s\r\n", subcmd);
        }
    }

    /* -------------------------------------------------------------------------- */
    /* Diagnostics command handler                                              */
    /* -------------------------------------------------------------------------- */
    static void cmdDiag(char *args)
    {
        char *subcmd = strtok(args, " ");
        if (subcmd == nullptr)
        {
            printf("Usage: diag <bus|stack|emcy|hb|pdo|sync|drive|all|watch>\r\n");
            return;
        }

        if (strcmp(subcmd, "bus") == 0)
        {
            Diagnostics::printBusStatus();
        }
        else if (strcmp(subcmd, "stack") == 0)
        {
            Diagnostics::printStackStatus();
        }
        else if (strcmp(subcmd, "emcy") == 0)
        {
            Diagnostics::printEmergencyHistory();
        }
        else if (strcmp(subcmd, "hb") == 0)
        {
            Diagnostics::printHeartbeatStatus();
        }
        else if (strcmp(subcmd, "pdo") == 0)
        {
            Diagnostics::printPDOStatus();
        }
        else if (strcmp(subcmd, "sync") == 0)
        {
            Diagnostics::printSyncStatus();
        }
        else if (strcmp(subcmd, "drive") == 0)
        {
            char *nodeStr = strtok(nullptr, " ");
            uint8_t nodeId = (nodeStr != nullptr)
                                 ? static_cast<uint8_t>(parseNumber(nodeStr))
                                 : DRIVE_NODE_ID;
            Diagnostics::printDriveErrors(nodeId);
        }
        else if (strcmp(subcmd, "all") == 0)
        {
            char *nodeStr = strtok(nullptr, " ");
            uint8_t nodeId = (nodeStr != nullptr)
                                 ? static_cast<uint8_t>(parseNumber(nodeStr))
                                 : DRIVE_NODE_ID;
            Diagnostics::printAll(nodeId, s_motor);
        }
        else if (strcmp(subcmd, "statelog") == 0)
        {
            Diagnostics::printStateLog(s_motor);
        }
        else if (strcmp(subcmd, "watch") == 0)
        {
            char *nodeStr = strtok(nullptr, " ");
            char *countStr = strtok(nullptr, " ");
            char *intervalStr = strtok(nullptr, " ");

            uint8_t nodeId = (nodeStr != nullptr)
                                 ? static_cast<uint8_t>(parseNumber(nodeStr))
                                 : DRIVE_NODE_ID;
            uint32_t count = (countStr != nullptr)
                                 ? parseNumber(countStr)
                                 : 10U;
            uint32_t interval = (intervalStr != nullptr)
                                    ? parseNumber(intervalStr)
                                    : 1000U;

            Diagnostics::watch(nodeId, count, interval);
        }
        else
        {
            printf("Unknown diag command: %s\r\n", subcmd);
        }
    }

    /* -------------------------------------------------------------------------- */
    /* Command dispatcher                                                         */
    /* -------------------------------------------------------------------------- */
    static void processCommand(char *line)
    {
        /* Skip leading whitespace */
        while (*line != '\0' && std::isspace(static_cast<unsigned char>(*line)))
        {
            line++;
        }

        if (*line == '\0')
        {
            return;
        }

        /* Extract first token */
        char *cmd = strtok(line, " ");
        char *rest = strtok(nullptr, ""); /* remainder */

        if (strcmp(cmd, "help") == 0)
        {
            cmdHelp();
        }
        else if (strcmp(cmd, "status") == 0)
        {
            cmdStatus();
        }
        else if (strcmp(cmd, "nmt") == 0)
        {
            cmdNMT(rest);
        }
        else if (strcmp(cmd, "sdo") == 0)
        {
            /* Next token: read or write */
            char *subcmd = strtok(rest, " ");
            char *subrest = strtok(nullptr, "");

            if (subcmd != nullptr && strcmp(subcmd, "read") == 0)
            {
                cmdSDORead(subrest);
            }
            else if (subcmd != nullptr && strcmp(subcmd, "write") == 0)
            {
                cmdSDOWrite(subrest);
            }
            else
            {
                printf("Usage: sdo <read|write> ...\r\n");
            }
        }
        else if (strcmp(cmd, "motor") == 0)
        {
            cmdMotor(rest);
        }
        else if (strcmp(cmd, "nt") == 0)
        {
            cmdNanotec(rest);
        }
        else if (strcmp(cmd, "diag") == 0)
        {
            cmdDiag(rest);
        }
        else
        {
            printf("Unknown command: %s (type 'help')\r\n", cmd);
        }
    }

    /* -------------------------------------------------------------------------- */
    /* Public API                                                                 */
    /* -------------------------------------------------------------------------- */
    void init(Interfaces::HLDriver::ICANopen &canopen,
              Interfaces::HLDriver::ICiA402 *motor,
              Interfaces::HLDriver::INanotecPnD *nanotec)
    {
        s_canopen = &canopen;
        s_motor = motor;
        s_nanotec = nanotec;
    }

    [[noreturn]] void run()
    {
        printf("\r\n=== CANopen Master CLI ===\r\n");
        printf("Type 'help' for available commands.\r\n");

        std::array<char, MAX_CMD_LEN> cmdBuf{};

        while (true)
        {
            printf("> ");
            fflush(stdout);

            size_t len = readLine(cmdBuf.data(), cmdBuf.size());
            if (len > 0)
            {
                processCommand(cmdBuf.data());
            }
        }
    }

} // namespace SerialCLI
