#include "Diagnostics.hpp"

#include <array>
#include <cstdio>
#include <cstring>

extern "C"
{
#include "CANopen.h"
#include "OD.h"
#include "301/CO_Emergency.h"
#include "301/CO_NMT_Heartbeat.h"
#include "301/CO_HBconsumer.h"
#include "301/CO_SYNC.h"
#include <tx_api.h>
}

#include "Interfaces/LLDriver/ICAN.hpp"
#include "Interfaces/HLDriver/ICANopen.hpp"
#include "Interfaces/HLDriver/ICiA402.hpp"

namespace Diagnostics
{

    /* ---------------------------------------------------------------------- */
    /* Module state                                                           */
    /* ---------------------------------------------------------------------- */
    static CO_t *s_co = nullptr;
    static Interfaces::LLDriver::ICAN *s_ican = nullptr;
    static Interfaces::HLDriver::ICANopen *s_canopen = nullptr;

    void init(void *co,
              Interfaces::LLDriver::ICAN *ican,
              Interfaces::HLDriver::ICANopen *canopen)
    {
        s_co = static_cast<CO_t *>(co);
        s_ican = ican;
        s_canopen = canopen;
    }

    /* ---------------------------------------------------------------------- */
    /* Helpers                                                                */
    /* ---------------------------------------------------------------------- */
    static const char *nmtStateStr(CO_NMT_internalState_t s)
    {
        switch (s)
        {
        case CO_NMT_INITIALIZING:
            return "Initializing";
        case CO_NMT_PRE_OPERATIONAL:
            return "Pre-Operational";
        case CO_NMT_OPERATIONAL:
            return "Operational";
        case CO_NMT_STOPPED:
            return "Stopped";
        default:
            return "Unknown";
        }
    }

    static const char *hbStateStr(CO_HBconsumer_state_t s)
    {
        switch (s)
        {
        case CO_HBconsumer_UNCONFIGURED:
            return "UNCONFIGURED";
        case CO_HBconsumer_UNKNOWN:
            return "UNKNOWN";
        case CO_HBconsumer_ACTIVE:
            return "ACTIVE";
        case CO_HBconsumer_TIMEOUT:
            return "TIMEOUT";
        default:
            return "???";
        }
    }

    static bool sdoReadU8(uint8_t nodeId, uint16_t idx, uint8_t sub, uint8_t &out)
    {
        if (s_canopen == nullptr)
        {
            return false;
        }
        uint8_t buf[1]{};
        auto r = s_canopen->sdoRead(nodeId, idx, sub, buf);
        if (!r.success)
        {
            return false;
        }
        out = buf[0];
        return true;
    }

    static bool sdoReadU16(uint8_t nodeId, uint16_t idx, uint8_t sub, uint16_t &out)
    {
        if (s_canopen == nullptr)
        {
            return false;
        }
        uint8_t buf[2]{};
        auto r = s_canopen->sdoRead(nodeId, idx, sub, buf);
        if (!r.success)
        {
            return false;
        }
        std::memcpy(&out, buf, 2);
        return true;
    }

    static bool sdoReadU32(uint8_t nodeId, uint16_t idx, uint8_t sub, uint32_t &out)
    {
        if (s_canopen == nullptr)
        {
            return false;
        }
        uint8_t buf[4]{};
        auto r = s_canopen->sdoRead(nodeId, idx, sub, buf);
        if (!r.success)
        {
            return false;
        }
        std::memcpy(&out, buf, 4);
        return true;
    }

    /* ====================================================================== */
    /* Decode helpers (must precede print* functions that reference them)      */
    /* ====================================================================== */

    const char *decodeSDOAbort(uint32_t abortCode)
    {
        switch (abortCode)
        {
        case 0x05030000:
            return "Toggle bit not alternated";
        case 0x05040000:
            return "SDO protocol timed out";
        case 0x05040001:
            return "Client/server command specifier invalid";
        case 0x05040002:
            return "Invalid block size";
        case 0x05040003:
            return "Invalid sequence number";
        case 0x05040004:
            return "CRC error";
        case 0x05040005:
            return "Out of memory";
        case 0x06010000:
            return "Unsupported access to object";
        case 0x06010001:
            return "Attempt to read write-only object";
        case 0x06010002:
            return "Attempt to write read-only object";
        case 0x06020000:
            return "Object does not exist in OD";
        case 0x06040041:
            return "Object cannot be mapped to PDO";
        case 0x06040042:
            return "PDO mapping exceeds PDO length";
        case 0x06040043:
            return "General parameter incompatibility";
        case 0x06040047:
            return "General internal incompatibility";
        case 0x06060000:
            return "Access failed due to hardware error";
        case 0x06070010:
            return "Data type mismatch, length mismatch";
        case 0x06070012:
            return "Data type mismatch, length too high";
        case 0x06070013:
            return "Data type mismatch, length too low";
        case 0x06090011:
            return "Sub-index does not exist";
        case 0x06090030:
            return "Invalid value for parameter";
        case 0x06090031:
            return "Value too high";
        case 0x06090032:
            return "Value too low";
        case 0x06090036:
            return "Maximum value less than minimum";
        case 0x060A0023:
            return "Resource not available: SDO connection";
        case 0x08000000:
            return "General error";
        case 0x08000020:
            return "Data cannot be transferred or stored";
        case 0x08000021:
            return "Data cannot be transferred (local control)";
        case 0x08000022:
            return "Data cannot be transferred (device state)";
        case 0x08000023:
            return "Object dictionary dynamic generation fails";
        case 0x08000024:
            return "No data available";
        default:
            return "Unknown abort code";
        }
    }

    const char *decodeEmergencyCode(uint16_t errorCode)
    {
        switch (errorCode)
        {
        case 0x0000:
            return "No error / Error reset";
        case 0x1000:
            return "Generic error";
        case 0x2000:
            return "Current";
        case 0x2100:
            return "Current, device input side";
        case 0x2200:
            return "Current, inside device";
        case 0x2300:
            return "Current, device output side";
        case 0x2310:
            return "Output current too high (overload)";
        case 0x2320:
            return "Short circuit at outputs";
        case 0x3000:
            return "Voltage";
        case 0x3100:
            return "Mains voltage";
        case 0x3110:
            return "Input voltage too high";
        case 0x3120:
            return "Input voltage too low";
        case 0x3200:
            return "Voltage inside device";
        case 0x3300:
            return "Output voltage";
        case 0x4000:
            return "Temperature";
        case 0x4100:
            return "Ambient temperature";
        case 0x4200:
            return "Device temperature";
        case 0x5000:
            return "Device hardware";
        case 0x5100:
            return "Device hardware (sub)";
        case 0x5200:
            return "Device hardware (sub)";
        case 0x5300:
            return "Device hardware (sub)";
        case 0x5400:
            return "Device hardware (sub)";
        case 0x5500:
            return "Device hardware (sub)";
        case 0x6000:
            return "Device software";
        case 0x6100:
            return "Internal software";
        case 0x6200:
            return "User software";
        case 0x6300:
            return "Data set";
        case 0x7000:
            return "Additional modules";
        case 0x8000:
            return "Monitoring";
        case 0x8100:
            return "Communication";
        case 0x8110:
            return "CAN overrun (objects lost)";
        case 0x8120:
            return "CAN in Error Passive Mode";
        case 0x8130:
            return "Heartbeat error";
        case 0x8140:
            return "Recovered from bus off";
        case 0x8150:
            return "CAN-ID collision";
        case 0x8200:
            return "Protocol error";
        case 0x8210:
            return "PDO not processed (length error)";
        case 0x8220:
            return "PDO length exceeded";
        case 0x8250:
            return "RPDO timeout";
        case 0xFF00:
            return "Device specific";
        default:
        {
            uint8_t category = static_cast<uint8_t>(errorCode >> 8U);
            switch (category)
            {
            case 0x10:
                return "Generic error (sub)";
            case 0x20:
            case 0x21:
            case 0x22:
            case 0x23:
                return "Current error (sub)";
            case 0x30:
            case 0x31:
            case 0x32:
            case 0x33:
                return "Voltage error (sub)";
            case 0x40:
            case 0x41:
            case 0x42:
                return "Temperature error (sub)";
            case 0x50:
            case 0x51:
            case 0x52:
            case 0x53:
            case 0x54:
            case 0x55:
                return "Hardware error (sub)";
            case 0x60:
            case 0x61:
            case 0x62:
            case 0x63:
                return "Software error (sub)";
            case 0x80:
            case 0x81:
            case 0x82:
                return "Communication/monitoring error (sub)";
            case 0x90:
                return "External error";
            case 0xF0:
                return "Additional functions";
            case 0xFF:
                return "Device specific";
            default:
                return "Unknown error code";
            }
        }
        }
    }

    static const char *decodeErrorBit(uint8_t bit)
    {
        switch (bit)
        {
        case CO_EM_NO_ERROR:
            return "No error / reset";
        case CO_EM_CAN_BUS_WARNING:
            return "CAN bus warning limit";
        case CO_EM_RXMSG_WRONG_LENGTH:
            return "RX message wrong length";
        case CO_EM_RXMSG_OVERFLOW:
            return "RX message overflow";
        case CO_EM_RPDO_WRONG_LENGTH:
            return "RPDO wrong length";
        case CO_EM_RPDO_OVERFLOW:
            return "RPDO overflow";
        case CO_EM_CAN_RX_BUS_PASSIVE:
            return "CAN RX bus passive";
        case CO_EM_CAN_TX_BUS_PASSIVE:
            return "CAN TX bus passive";
        case CO_EM_NMT_WRONG_COMMAND:
            return "Wrong NMT command";
        case CO_EM_TIME_TIMEOUT:
            return "TIME message timeout";
        case CO_EM_CAN_TX_BUS_OFF:
            return "CAN TX bus off";
        case CO_EM_CAN_RXB_OVERFLOW:
            return "CAN RX buffer overflow";
        case CO_EM_CAN_TX_OVERFLOW:
            return "CAN TX overflow";
        case CO_EM_TPDO_OUTSIDE_WINDOW:
            return "TPDO outside SYNC window";
        case CO_EM_RPDO_TIME_OUT:
            return "RPDO timeout";
        case CO_EM_SYNC_TIME_OUT:
            return "SYNC timeout";
        case CO_EM_SYNC_LENGTH:
            return "SYNC data length";
        case CO_EM_PDO_WRONG_MAPPING:
            return "PDO mapping error";
        case CO_EM_HEARTBEAT_CONSUMER:
            return "Heartbeat consumer timeout";
        case CO_EM_HB_CONSUMER_REMOTE_RESET:
            return "HB consumer remote node reset";
        case CO_EM_EMERGENCY_BUFFER_FULL:
            return "Emergency buffer full";
        case CO_EM_MICROCONTROLLER_RESET:
            return "Microcontroller reset";
        case CO_EM_WRONG_ERROR_REPORT:
            return "Wrong error report params";
        case CO_EM_ISR_TIMER_OVERFLOW:
            return "Timer ISR overflow";
        case CO_EM_MEMORY_ALLOCATION_ERROR:
            return "Memory allocation error";
        case CO_EM_GENERIC_ERROR:
            return "Generic error";
        case CO_EM_GENERIC_SOFTWARE_ERROR:
            return "Generic software error";
        case CO_EM_INCONSISTENT_OBJECT_DICT:
            return "Inconsistent object dictionary";
        case CO_EM_CALCULATION_OF_PARAMETERS:
            return "Calculation of parameters error";
        case CO_EM_NON_VOLATILE_MEMORY:
            return "Non-volatile memory error";
        default:
            if (bit >= CO_EM_MANUFACTURER_START)
            {
                return "Manufacturer-specific";
            }
            return "Unknown";
        }
    }

    static const char *decodeCiA402State(uint16_t sw)
    {
        uint16_t masked = sw & 0x006FU;
        if ((masked & 0x004F) == 0x0000)
            return "NotReadyToSwitchOn";
        if ((masked & 0x004F) == 0x0040)
            return "SwitchOnDisabled";
        if ((masked & 0x006F) == 0x0021)
            return "ReadyToSwitchOn";
        if ((masked & 0x006F) == 0x0023)
            return "SwitchedOn";
        if ((masked & 0x006F) == 0x0027)
            return "OperationEnabled";
        if ((masked & 0x006F) == 0x0007)
            return "QuickStopActive";
        if ((masked & 0x004F) == 0x000F)
            return "FaultReactionActive";
        if ((masked & 0x004F) == 0x0008)
            return "Fault";
        return "Unknown";
    }

    static const char *decodeModeStr(int8_t mode)
    {
        switch (mode)
        {
        case 0:
            return " (none)";
        case 1:
            return " (PP)";
        case 3:
            return " (PV)";
        case 6:
            return " (Homing)";
        case 7:
            return " (IP)";
        case 8:
            return " (CSP)";
        case 9:
            return " (CSV)";
        case 10:
            return " (CST)";
        case -1:
            return " (AutoSetup)";
        case -2:
            return " (ClockDir)";
        default:
            return "";
        }
    }

    /* ====================================================================== */
    /* CAN Bus Status                                                         */
    /* ====================================================================== */
    void printBusStatus()
    {
        printf("=== CAN Bus Status ===\r\n");

        if (s_ican == nullptr || s_co == nullptr)
        {
            printf("  (not initialised)\r\n");
            return;
        }

        /* Hardware error counters from FDCAN peripheral */
        uint16_t rxErr = 0, txErr = 0, overflow = 0;
        s_ican->getErrorCounters(rxErr, txErr, overflow);

        printf("  RX error count:  %u\r\n", rxErr);
        printf("  TX error count:  %u\r\n", txErr);
        printf("  RX FIFO lost:    %u\r\n", overflow);

        /* Determine bus state from error counts */
        const char *busState = "Normal (Error-Active)";
        if (txErr >= 256)
        {
            busState = "BUS-OFF";
        }
        else if (rxErr >= 128 || txErr >= 128)
        {
            busState = "Error-Passive";
        }
        else if (rxErr >= 96 || txErr >= 96)
        {
            busState = "Warning";
        }
        printf("  Bus state:       %s\r\n", busState);

        /* CANopenNode CAN error status flags */
        uint16_t canErrStatus = s_co->CANmodule->CANerrorStatus;
        printf("  CAN error flags: 0x%04X", canErrStatus);
        if (canErrStatus == 0)
        {
            printf(" (none)");
        }
        else
        {
            if (canErrStatus & CO_CAN_ERRTX_BUS_OFF)
            {
                printf(" [TX_BUS_OFF]");
            }
            if (canErrStatus & CO_CAN_ERRTX_OVERFLOW)
            {
                printf(" [TX_OVERFLOW]");
            }
            if (canErrStatus & CO_CAN_ERRTX_PASSIVE)
            {
                printf(" [TX_PASSIVE]");
            }
            if (canErrStatus & CO_CAN_ERRTX_WARNING)
            {
                printf(" [TX_WARNING]");
            }
            if (canErrStatus & CO_CAN_ERRRX_PASSIVE)
            {
                printf(" [RX_PASSIVE]");
            }
            if (canErrStatus & CO_CAN_ERRRX_WARNING)
            {
                printf(" [RX_WARNING]");
            }
            if (canErrStatus & CO_CAN_ERRRX_OVERFLOW)
            {
                printf(" [RX_OVERFLOW]");
            }
            if (canErrStatus & CO_CAN_ERRTX_PDO_LATE)
            {
                printf(" [TPDO_LATE]");
            }
        }
        printf("\r\n");
        printf("  CAN normal:      %s\r\n", s_co->CANmodule->CANnormal ? "yes" : "NO");
        printf("  TX pending:      %u\r\n", s_co->CANmodule->CANtxCount);
    }

    /* ====================================================================== */
    /* CANopen Stack Status                                                   */
    /* ====================================================================== */
    void printStackStatus()
    {
        printf("=== CANopen Stack Status ===\r\n");

        if (s_co == nullptr)
        {
            printf("  (not initialised)\r\n");
            return;
        }

        /* NMT state */
        CO_NMT_internalState_t nmt = CO_NMT_getInternalState(s_co->NMT);
        printf("  NMT state:       %s (%d)\r\n", nmtStateStr(nmt), static_cast<int>(nmt));
        printf("  Node ID:         %u\r\n", s_co->NMT->nodeId);

        /* Error register (OD 0x1001) */
        uint8_t errReg = CO_getErrorRegister(s_co->em);
        printf("  Error register:  0x%02X", errReg);
        if (errReg == 0)
        {
            printf(" (clean)");
        }
        else
        {
            if (errReg & CO_ERR_REG_GENERIC_ERR)
            {
                printf(" [GENERIC]");
            }
            if (errReg & CO_ERR_REG_CURRENT)
            {
                printf(" [CURRENT]");
            }
            if (errReg & CO_ERR_REG_VOLTAGE)
            {
                printf(" [VOLTAGE]");
            }
            if (errReg & CO_ERR_REG_TEMPERATURE)
            {
                printf(" [TEMP]");
            }
            if (errReg & CO_ERR_REG_COMMUNICATION)
            {
                printf(" [COMM]");
            }
            if (errReg & CO_ERR_REG_DEV_PROFILE)
            {
                printf(" [PROFILE]");
            }
            if (errReg & CO_ERR_REG_MANUFACTURER)
            {
                printf(" [MFR]");
            }
        }
        printf("\r\n");

        /* Active error status bits */
        printf("  Active errors:   ");
        bool anyError = false;
        constexpr size_t numBytes = CO_CONFIG_EM_ERR_STATUS_BITS_COUNT / 8U;
        for (size_t byteIdx = 0; byteIdx < numBytes; byteIdx++)
        {
            uint8_t b = s_co->em->errorStatusBits[byteIdx];
            if (b != 0)
            {
                for (uint8_t bit = 0; bit < 8; bit++)
                {
                    if (b & (1U << bit))
                    {
                        uint8_t errBit = static_cast<uint8_t>(byteIdx * 8U + bit);
                        if (anyError)
                        {
                            printf("                   ");
                        }
                        printf("[0x%02X] %s\r\n", errBit, decodeErrorBit(errBit));
                        anyError = true;
                    }
                }
            }
        }
        if (!anyError)
        {
            printf("(none)\r\n");
        }
    }

    /* ====================================================================== */
    /* Emergency History                                                      */
    /* ====================================================================== */
    void printEmergencyHistory()
    {
        printf("=== Emergency History (local 0x1003) ===\r\n");

        if (s_co == nullptr || s_co->em == nullptr)
        {
            printf("  (not initialised)\r\n");
            return;
        }

#if (CO_CONFIG_EM) & CO_CONFIG_EM_HISTORY
        uint8_t count = s_co->em->fifoCount;
        if (count == 0)
        {
            printf("  (no errors recorded)\r\n");
            return;
        }

        printf("  %u error(s) recorded:\r\n", count);
        uint8_t fifoSize = s_co->em->fifoSize;

        /* Walk the fifo from oldest to newest */
        for (uint8_t i = 0; i < count && i < fifoSize; i++)
        {
            /* fifo entries: msg = (errorCode | (errorRegister << 16) | (errorBit << 24)) */
            uint32_t msg = s_co->em->fifo[i].msg;
            uint16_t errorCode = static_cast<uint16_t>(msg & 0xFFFFU);
            uint8_t errorReg = static_cast<uint8_t>((msg >> 16U) & 0xFFU);
            uint8_t errorBit = static_cast<uint8_t>((msg >> 24U) & 0xFFU);

#if (CO_CONFIG_EM) & CO_CONFIG_EM_PRODUCER
            uint32_t info = s_co->em->fifo[i].info;
            printf("  [%2u] Code=0x%04X  Reg=0x%02X  Bit=0x%02X  Info=0x%08lX\r\n",
                   i + 1, errorCode, errorReg, errorBit,
                   static_cast<unsigned long>(info));
#else
            printf("  [%2u] Code=0x%04X  Reg=0x%02X  Bit=0x%02X\r\n",
                   i + 1, errorCode, errorReg, errorBit);
#endif
            printf("       → %s\r\n", decodeEmergencyCode(errorCode));
        }
#else
        printf("  (EM history not enabled in build)\r\n");
#endif
    }

    /* ====================================================================== */
    /* Heartbeat Consumer                                                     */
    /* ====================================================================== */
    void printHeartbeatStatus()
    {
        printf("=== Heartbeat Consumer ===\r\n");

        if (s_co == nullptr || s_co->HBcons == nullptr)
        {
            printf("  (not initialised)\r\n");
            return;
        }

#if (CO_CONFIG_HB_CONS) & CO_CONFIG_HB_CONS_ENABLE
        CO_HBconsumer_t *hb = s_co->HBcons;
        uint8_t numNodes = hb->numberOfMonitoredNodes;

        for (uint8_t i = 0; i < numNodes; i++)
        {
            CO_HBconsNode_t *node = &hb->monitoredNodes[i];
            if (node->nodeId == 0)
            {
                continue; /* unconfigured slot */
            }

            CO_HBconsumer_state_t hbState = CO_HBconsumer_getState(hb, i);
            CO_NMT_internalState_t nmtState = CO_NMT_UNKNOWN;
            CO_HBconsumer_getNmtState(hb, i, &nmtState);

            printf("  Node %3u: %-13s  NMT=%-16s  timer=%lu/%lu us\r\n",
                   node->nodeId,
                   hbStateStr(hbState),
                   nmtStateStr(nmtState),
                   static_cast<unsigned long>(node->timeoutTimer),
                   static_cast<unsigned long>(node->time_us));
        }

        printf("  All monitored active:      %s\r\n",
               hb->allMonitoredActive ? "yes" : "NO");
        printf("  All monitored operational: %s\r\n",
               hb->allMonitoredOperational ? "yes" : "NO");
#else
        printf("  (HB consumer not enabled in build)\r\n");
#endif
    }

    /* ====================================================================== */
    /* PDO Status                                                             */
    /* ====================================================================== */
    void printPDOStatus()
    {
        printf("=== PDO Status ===\r\n");

        /* Snapshot local OD values under lock */
        CO_LOCK_OD(nullptr);
        uint16_t cw = OD_RAM.x2000_controlword;
        int32_t tgtPos = OD_RAM.x2001_targetPosition;
        int8_t modesOp = OD_RAM.x2002_modesOfOperation;
        int32_t tgtVel = OD_RAM.x2003_targetVelocity;
        uint16_t sw = OD_RAM.x2004_statusword;
        int32_t actPos = OD_RAM.x2005_actualPosition;
        int8_t modeDisp = OD_RAM.x2006_modesOfOperationDisplay;
        CO_UNLOCK_OD(nullptr);

        printf("  --- Local OD (0x2000-0x2006) ---\r\n");
        printf("  Controlword  (0x2000): 0x%04X\r\n", cw);
        printf("  Target pos   (0x2001): %ld\r\n", static_cast<long>(tgtPos));
        printf("  Modes of op  (0x2002): %d%s\r\n", modesOp, decodeModeStr(modesOp));
        printf("  Target vel   (0x2003): %ld\r\n", static_cast<long>(tgtVel));
        printf("  Statusword   (0x2004): 0x%04X\r\n", sw);
        printf("  Actual pos   (0x2005): %ld\r\n", static_cast<long>(actPos));
        printf("  Mode display (0x2006): %d%s\r\n", modeDisp, decodeModeStr(modeDisp));

        /* CiA 402 state from statusword */
        printf("  CiA 402 state:         %s\r\n", decodeCiA402State(sw));
        printf("  Target reached:        %s\r\n", (sw & (1U << 10)) ? "yes" : "no");
        printf("  Fault:                 %s\r\n", (sw & (1U << 3)) ? "YES" : "no");
        printf("  Warning:               %s\r\n", (sw & (1U << 7)) ? "YES" : "no");

        /* PDO mapping summary from OD_PERSIST_COMM */
        printf("  --- TPDO Mapping (master → drive) ---\r\n");
        printf("  TPDO1: COB-ID=0x%04lX  txType=%u  maps=[0x%08lX, 0x%08lX]\r\n",
               static_cast<unsigned long>(OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.COB_IDUsedByTPDO),
               OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.transmissionType,
               static_cast<unsigned long>(OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject1),
               static_cast<unsigned long>(OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject2));
        printf("  TPDO2: COB-ID=0x%04lX  txType=%u  maps=[0x%08lX, 0x%08lX]\r\n",
               static_cast<unsigned long>(OD_PERSIST_COMM.x1801_TPDOCommunicationParameter.COB_IDUsedByTPDO),
               OD_PERSIST_COMM.x1801_TPDOCommunicationParameter.transmissionType,
               static_cast<unsigned long>(OD_PERSIST_COMM.x1A01_TPDOMappingParameter.applicationObject1),
               static_cast<unsigned long>(OD_PERSIST_COMM.x1A01_TPDOMappingParameter.applicationObject2));

        printf("  --- RPDO Mapping (drive → master) ---\r\n");
        printf("  RPDO1: COB-ID=0x%04lX  txType=%u  evtTmr=%ums  maps=[0x%08lX, 0x%08lX]\r\n",
               static_cast<unsigned long>(OD_PERSIST_COMM.x1400_RPDOCommunicationParameter.COB_IDUsedByRPDO),
               OD_PERSIST_COMM.x1400_RPDOCommunicationParameter.transmissionType,
               OD_PERSIST_COMM.x1400_RPDOCommunicationParameter.eventTimer,
               static_cast<unsigned long>(OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject1),
               static_cast<unsigned long>(OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject2));
        printf("  RPDO2: COB-ID=0x%04lX  txType=%u  evtTmr=%ums  maps=[0x%08lX]\r\n",
               static_cast<unsigned long>(OD_PERSIST_COMM.x1401_RPDOCommunicationParameter.COB_IDUsedByRPDO),
               OD_PERSIST_COMM.x1401_RPDOCommunicationParameter.transmissionType,
               OD_PERSIST_COMM.x1401_RPDOCommunicationParameter.eventTimer,
               static_cast<unsigned long>(OD_PERSIST_COMM.x1601_RPDOMappingParameter.applicationObject1));
    }

    /* ====================================================================== */
    /* SYNC Producer                                                          */
    /* ====================================================================== */
    void printSyncStatus()
    {
        printf("=== SYNC Status ===\r\n");

        if (s_co == nullptr)
        {
            printf("  (not initialised)\r\n");
            return;
        }

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
        CO_SYNC_t *sync = s_co->SYNC;
        if (sync == nullptr)
        {
            printf("  (SYNC object null)\r\n");
            return;
        }

        uint32_t cobId = OD_PERSIST_COMM.x1005_COB_ID_SYNCMessage;
        bool isProducer = (cobId & 0x40000000UL) != 0;
        uint32_t period = OD_PERSIST_COMM.x1006_communicationCyclePeriod;
        uint32_t window = OD_PERSIST_COMM.x1007_synchronousWindowLength;

        printf("  COB-ID:          0x%08lX\r\n", static_cast<unsigned long>(cobId));
        printf("  Producer:        %s\r\n", isProducer ? "YES" : "no (consumer only)");
        printf("  Period:          %lu µs (%lu ms)\r\n",
               static_cast<unsigned long>(period),
               static_cast<unsigned long>(period / 1000));
        printf("  Window:          %lu µs (%lu ms)\r\n",
               static_cast<unsigned long>(window),
               static_cast<unsigned long>(window / 1000));
        printf("  Timer:           %lu µs\r\n", static_cast<unsigned long>(sync->timer));
        printf("  Counter:         %u (overflow at %u)\r\n",
               sync->counter, sync->counterOverflowValue);
        printf("  Outside window:  %s\r\n", sync->syncIsOutsideWindow ? "YES" : "no");

        const char *timeoutStr = "not started";
        if (sync->timeoutError == 1)
        {
            timeoutStr = "running (OK)";
        }
        else if (sync->timeoutError == 2)
        {
            timeoutStr = "TIMEOUT ERROR";
        }
        printf("  Timeout state:   %s\r\n", timeoutStr);
#else
        printf("  (SYNC not enabled in build)\r\n");
#endif
    }

    /* ====================================================================== */
    /* Drive-Side Errors (via SDO)                                            */
    /* ====================================================================== */
    void printDriveErrors(uint8_t driveNodeId)
    {
        printf("=== Drive Node %u Errors (via SDO) ===\r\n", driveNodeId);

        if (s_canopen == nullptr)
        {
            printf("  (CANopen not initialised)\r\n");
            return;
        }

        /* 0x1001:00 — Error register */
        uint8_t errReg = 0;
        if (sdoReadU8(driveNodeId, 0x1001, 0, errReg))
        {
            printf("  Error register (0x1001): 0x%02X", errReg);
            if (errReg == 0)
            {
                printf(" (clean)\r\n");
            }
            else
            {
                if (errReg & 0x01)
                {
                    printf(" [GENERIC]");
                }
                if (errReg & 0x02)
                {
                    printf(" [CURRENT]");
                }
                if (errReg & 0x04)
                {
                    printf(" [VOLTAGE]");
                }
                if (errReg & 0x08)
                {
                    printf(" [TEMP]");
                }
                if (errReg & 0x10)
                {
                    printf(" [COMM]");
                }
                if (errReg & 0x20)
                {
                    printf(" [PROFILE]");
                }
                if (errReg & 0x80)
                {
                    printf(" [MFR]");
                }
                printf("\r\n");
            }
        }
        else
        {
            printf("  Error register (0x1001): READ FAILED\r\n");
        }

        /* 0x603F:00 — CiA 402 error code */
        uint16_t errorCode402 = 0;
        if (sdoReadU16(driveNodeId, 0x603F, 0, errorCode402))
        {
            printf("  CiA 402 error  (0x603F): 0x%04X → %s\r\n",
                   errorCode402, decodeEmergencyCode(errorCode402));
        }
        else
        {
            printf("  CiA 402 error  (0x603F): READ FAILED (may not be supported)\r\n");
        }

        /* 0x1003 — Pre-defined error field (error history) */
        uint8_t errHistCount = 0;
        if (!sdoReadU8(driveNodeId, 0x1003, 0, errHistCount))
        {
            printf("  Error history  (0x1003): READ FAILED\r\n");
            return;
        }

        if (errHistCount == 0)
        {
            printf("  Error history  (0x1003): (empty)\r\n");
        }
        else
        {
            printf("  Error history  (0x1003): %u entries\r\n", errHistCount);
            uint8_t maxRead = (errHistCount > 10) ? 10 : errHistCount;
            for (uint8_t i = 1; i <= maxRead; i++)
            {
                uint32_t entry = 0;
                if (sdoReadU32(driveNodeId, 0x1003, i, entry))
                {
                    uint16_t code = static_cast<uint16_t>(entry & 0xFFFFU);
                    uint16_t extra = static_cast<uint16_t>((entry >> 16U) & 0xFFFFU);
                    printf("    [%u] 0x%08lX  code=0x%04X extra=0x%04X → %s\r\n",
                           i, static_cast<unsigned long>(entry),
                           code, extra, decodeEmergencyCode(code));
                }
            }
        }

        /* 0x6041:00 — Statusword (direct read) */
        uint16_t swDirect = 0;
        if (sdoReadU16(driveNodeId, 0x6041, 0, swDirect))
        {
            printf("  Statusword     (0x6041): 0x%04X → %s\r\n",
                   swDirect, decodeCiA402State(swDirect));
        }

        /* 0x6061:00 — Modes of operation display */
        uint8_t modeRaw = 0;
        if (sdoReadU8(driveNodeId, 0x6061, 0, modeRaw))
        {
            printf("  Mode display   (0x6061): %d%s\r\n",
                   static_cast<int8_t>(modeRaw),
                   decodeModeStr(static_cast<int8_t>(modeRaw)));
        }

        /* 0x60FD:00 — Digital inputs (Nanotec: limit switches, home, etc.) */
        uint32_t digitalInputs = 0;
        if (sdoReadU32(driveNodeId, 0x60FD, 0, digitalInputs))
        {
            printf("  Digital inputs (0x60FD): 0x%08lX", static_cast<unsigned long>(digitalInputs));
            if (digitalInputs & (1UL << 0))
            {
                printf(" [NegLimit]");
            }
            if (digitalInputs & (1UL << 1))
            {
                printf(" [PosLimit]");
            }
            if (digitalInputs & (1UL << 2))
            {
                printf(" [HomeSw]");
            }
            printf("\r\n");
        }
    }

    /* ====================================================================== */
    /* CiA 402 State Transition Log                                           */
    /* ====================================================================== */

    static const char *cia402StateStr(Interfaces::HLDriver::CiA402State s)
    {
        using S = Interfaces::HLDriver::CiA402State;
        switch (s)
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

    void printStateLog(const Interfaces::HLDriver::ICiA402 *motor)
    {
        printf("=== CiA 402 State Transition Log ===\r\n");

        if (motor == nullptr)
        {
            printf("  (motor driver not available)\r\n");
            return;
        }

        auto log = motor->getStateLog();
        if (log.count == 0)
        {
            printf("  (no transitions recorded)\r\n");
            return;
        }

        printf("  %u transition(s) recorded:\r\n", log.count);

        /* Print from oldest to newest */
        uint8_t startIdx = (log.count < Interfaces::HLDriver::STATE_LOG_CAPACITY)
                               ? 0
                               : log.writeIdx;
        uint8_t capacity = static_cast<uint8_t>(Interfaces::HLDriver::STATE_LOG_CAPACITY);

        for (uint8_t i = 0; i < log.count; i++)
        {
            uint8_t idx = (startIdx + i) % capacity;
            auto &e = log.entries[idx];
            printf("  [%8lu ms] %-20s -> %-20s  SW=0x%04X\r\n",
                   static_cast<unsigned long>(e.timestamp_ms),
                   cia402StateStr(e.fromState),
                   cia402StateStr(e.toState),
                   e.statusword);
        }
    }

    /* ====================================================================== */
    /* Full Dump                                                              */
    /* ====================================================================== */
    void printAll(uint8_t driveNodeId, const Interfaces::HLDriver::ICiA402 *motor)
    {
        printBusStatus();
        printf("\r\n");
        printStackStatus();
        printf("\r\n");
        printEmergencyHistory();
        printf("\r\n");
        printHeartbeatStatus();
        printf("\r\n");
        printSyncStatus();
        printf("\r\n");
        printPDOStatus();
        printf("\r\n");
        printDriveErrors(driveNodeId);
        printf("\r\n");
        printStateLog(motor);
    }

    /* ====================================================================== */
    /* Continuous Watch                                                       */
    /* ====================================================================== */
    void watch(uint8_t driveNodeId, uint32_t count, uint32_t interval_ms)
    {
        printf("=== Watch Mode (every %lu ms, %s) ===\r\n",
               static_cast<unsigned long>(interval_ms),
               count == 0 ? "infinite" : "N lines");
        printf("NMT      Bus   CAN-Err  SW       State              Pos          Mode  HB\r\n");
        printf("-------- ----- -------- -------- ------------------ ------------ ----- --------\r\n");

        uint32_t iteration = 0;
        while (count == 0 || iteration < count)
        {
            /* NMT state */
            CO_NMT_internalState_t nmt = CO_NMT_INITIALIZING;
            if (s_co != nullptr && s_co->NMT != nullptr)
            {
                nmt = CO_NMT_getInternalState(s_co->NMT);
            }

            /* Bus status */
            uint16_t rxErr = 0, txErr = 0, overflow = 0;
            if (s_ican != nullptr)
            {
                s_ican->getErrorCounters(rxErr, txErr, overflow);
            }

            const char *busStr = "OK";
            if (txErr >= 256)
            {
                busStr = "BOFF";
            }
            else if (rxErr >= 128 || txErr >= 128)
            {
                busStr = "PASV";
            }
            else if (rxErr >= 96 || txErr >= 96)
            {
                busStr = "WARN";
            }

            /* CAN error flags */
            uint16_t canErr = 0;
            if (s_co != nullptr)
            {
                canErr = s_co->CANmodule->CANerrorStatus;
            }

            /* PDO snapshot */
            CO_LOCK_OD(nullptr);
            uint16_t sw = OD_RAM.x2004_statusword;
            int32_t pos = OD_RAM.x2005_actualPosition;
            int8_t mode = OD_RAM.x2006_modesOfOperationDisplay;
            CO_UNLOCK_OD(nullptr);

            /* Heartbeat */
            const char *hbStr = "N/A";
#if (CO_CONFIG_HB_CONS) & CO_CONFIG_HB_CONS_ENABLE
            if (s_co != nullptr && s_co->HBcons != nullptr)
            {
                int8_t idx = CO_HBconsumer_getIdxByNodeId(s_co->HBcons, driveNodeId);
                if (idx >= 0)
                {
                    CO_HBconsumer_state_t hbs = CO_HBconsumer_getState(s_co->HBcons, static_cast<uint8_t>(idx));
                    hbStr = hbStateStr(hbs);
                }
            }
#endif

            printf("%-8s %-5s 0x%04X   0x%04X   %-18s %-12ld %-5d %s\r\n",
                   nmtStateStr(nmt), busStr, canErr, sw,
                   decodeCiA402State(sw),
                   static_cast<long>(pos), mode, hbStr);

            iteration++;
            tx_thread_sleep(interval_ms);
        }
        printf("=== Watch ended ===\r\n");
    }

} // namespace Diagnostics
