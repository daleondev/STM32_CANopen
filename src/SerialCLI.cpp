#include "SerialCLI.hpp"

#include <array>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

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
        else
        {
            printf("Unknown command: %s (type 'help')\r\n", cmd);
        }
    }

    /* -------------------------------------------------------------------------- */
    /* Public API                                                                 */
    /* -------------------------------------------------------------------------- */
    void init(Interfaces::HLDriver::ICANopen &canopen)
    {
        s_canopen = &canopen;
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
