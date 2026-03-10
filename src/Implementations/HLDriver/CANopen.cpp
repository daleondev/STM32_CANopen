#include "Implementations/HLDriver/CANopen.hpp"

#include <cstdio>
#include <cstring>
#include <tx_api.h>

extern "C"
{
#include "CANopen.h"
#include "301/CO_SDOclient.h"
#include "OD.h"
}

namespace Implementations::HLDriver
{

    CANopen::CANopen(Interfaces::LLDriver::ICAN &ican) : ican_{ican}
    {
    }

    bool CANopen::init(uint8_t nodeId, uint16_t bitRate)
    {
        /* De-init if previously initialized */
        if (co_ != nullptr)
        {
            CO_CANsetConfigurationMode(static_cast<void *>(&ican_));
            CO_delete(co_);
            co_ = nullptr;
            initialized_ = false;
        }

        /* Allocate CANopen objects (uses heap once at startup) */
        uint32_t heapUsed = 0;
        co_ = CO_new(nullptr, &heapUsed);
        if (co_ == nullptr)
        {
            printf("CANopen: CO_new failed\r\n");
            return false;
        }
        printf("CANopen: allocated %lu bytes\r\n", static_cast<unsigned long>(heapUsed));

        /* Enter CAN configuration mode */
        CO_CANsetConfigurationMode(static_cast<void *>(&ican_));
        CO_CANmodule_disable(co_->CANmodule);

        /* Initialize CAN driver */
        CO_ReturnError_t err = CO_CANinit(co_, static_cast<void *>(&ican_), bitRate);
        if (err != CO_ERROR_NO)
        {
            printf("CANopen: CO_CANinit failed: %d\r\n", err);
            CO_delete(co_);
            co_ = nullptr;
            return false;
        }

        /* Initialize CANopen stack (NMT, SDO, Emergency, HB, etc.) */
        activeNodeId_ = nodeId;
        uint32_t errInfo = 0;

        err = CO_CANopenInit(co_,
                             nullptr,            /* alternate NMT */
                             nullptr,            /* alternate EM */
                             OD,                 /* Object Dictionary */
                             NULL,               /* OD status bits */
                             NMT_CONTROL,        /* NMT control */
                             FIRST_HB_TIME_MS,   /* first heartbeat time */
                             SDO_SRV_TIMEOUT_MS, /* SDO server timeout */
                             SDO_CLI_TIMEOUT_MS, /* SDO client timeout */
                             false,              /* SDO client block transfer */
                             activeNodeId_,
                             &errInfo);

        if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS)
        {
            if (err == CO_ERROR_OD_PARAMETERS)
            {
                printf("CANopen: OD entry error 0x%lX\r\n", static_cast<unsigned long>(errInfo));
            }
            else
            {
                printf("CANopen: CO_CANopenInit failed: %d\r\n", err);
            }
            CO_delete(co_);
            co_ = nullptr;
            return false;
        }

        /* Initialize PDO objects */
        err = CO_CANopenInitPDO(co_, co_->em, OD, activeNodeId_, &errInfo);
        if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS)
        {
            if (err == CO_ERROR_OD_PARAMETERS)
            {
                printf("CANopen: PDO OD entry error 0x%lX\r\n", static_cast<unsigned long>(errInfo));
            }
            else
            {
                printf("CANopen: CO_CANopenInitPDO failed: %d\r\n", err);
            }
            CO_delete(co_);
            co_ = nullptr;
            return false;
        }

        /* Start CAN */
        CO_CANsetNormalMode(co_->CANmodule);
        initialized_ = true;

        printf("CANopen: running (nodeId=%u, bitRate=%u kbit/s)\r\n", activeNodeId_, bitRate);

        return true;
    }

    uint8_t CANopen::process(uint32_t timeDifference_us)
    {
        if (co_ == nullptr || !initialized_)
        {
            return 0;
        }

        CO_NMT_reset_cmd_t reset = CO_process(co_, false, timeDifference_us, nullptr);
        return static_cast<uint8_t>(reset);
    }

    bool CANopen::processSync(uint32_t timeDifference_us)
    {
        if (co_ == nullptr || !initialized_ || !co_->CANmodule->CANnormal)
        {
            return false;
        }

        bool syncWas = false;

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
        syncWas = CO_process_SYNC(co_, timeDifference_us, nullptr);
#endif

#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
        CO_process_RPDO(co_, syncWas, timeDifference_us, nullptr);
#endif

#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
        CO_process_TPDO(co_, syncWas, timeDifference_us, nullptr);
#endif

        return syncWas;
    }

    bool CANopen::sendNMTCommand(uint8_t command, uint8_t targetNodeId)
    {
#if (CO_CONFIG_NMT) & CO_CONFIG_NMT_MASTER
        if (co_ == nullptr || co_->NMT == nullptr)
        {
            return false;
        }
        CO_ReturnError_t err = CO_NMT_sendCommand(co_->NMT, static_cast<CO_NMT_command_t>(command), targetNodeId);
        return err == CO_ERROR_NO;
#else
        (void)command;
        (void)targetNodeId;
        return false;
#endif
    }

    Interfaces::HLDriver::SDOResult CANopen::sdoRead(uint8_t nodeId, uint16_t index, uint8_t subIndex,
                                                     std::span<uint8_t> buf)
    {
        Interfaces::HLDriver::SDOResult result{};

#if (CO_CONFIG_SDO_CLI) & CO_CONFIG_SDO_CLI_ENABLE
        if (co_ == nullptr || co_->SDOclient == nullptr || buf.empty())
        {
            result.abortCode = 0x08000000U; /* General error */
            return result;
        }

        CO_SDOclient_t *sdo = co_->SDOclient;

        /* Setup SDO client for the target node */
        CO_SDO_return_t ret = CO_SDOclient_setup(sdo,
                                                 CO_CAN_ID_SDO_CLI + nodeId,
                                                 CO_CAN_ID_SDO_SRV + nodeId,
                                                 nodeId);
        if (ret != CO_SDO_RT_ok_communicationEnd)
        {
            result.abortCode = 0x08000000U;
            return result;
        }

        /* Initiate upload */
        ret = CO_SDOclientUploadInitiate(sdo, index, subIndex, SDO_TRANSFER_TIMEOUT_MS, false);
        if (ret != CO_SDO_RT_ok_communicationEnd)
        {
            result.abortCode = 0x08000000U;
            return result;
        }

        /* Process upload */
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
        do
        {
            constexpr uint32_t pollInterval_us = 1000U; /* 1 ms */
            ret = CO_SDOclientUpload(sdo, pollInterval_us, false, &abortCode, nullptr, nullptr, nullptr);
            if (ret < 0)
            {
                result.abortCode = abortCode;
                CO_SDOclientClose(sdo);
                return result;
            }
            if (ret > 0)
            {
                tx_thread_sleep(1); /* yield for 1 tick (~1 ms) */
            }
        } while (ret > 0);

        /* Read data from the FIFO buffer */
        result.bytesTransferred = CO_SDOclientUploadBufRead(sdo, buf.data(), buf.size());
        result.success = true;
        result.abortCode = CO_SDO_AB_NONE;

        CO_SDOclientClose(sdo);
#else
        (void)nodeId;
        (void)index;
        (void)subIndex;
        result.abortCode = 0x08000000U;
#endif

        return result;
    }

    Interfaces::HLDriver::SDOResult CANopen::sdoWrite(uint8_t nodeId, uint16_t index, uint8_t subIndex,
                                                      std::span<const uint8_t> data)
    {
        Interfaces::HLDriver::SDOResult result{};

#if (CO_CONFIG_SDO_CLI) & CO_CONFIG_SDO_CLI_ENABLE
        if (co_ == nullptr || co_->SDOclient == nullptr || data.empty())
        {
            result.abortCode = 0x08000000U;
            return result;
        }

        CO_SDOclient_t *sdo = co_->SDOclient;

        /* Setup SDO client for the target node */
        CO_SDO_return_t ret = CO_SDOclient_setup(sdo,
                                                 CO_CAN_ID_SDO_CLI + nodeId,
                                                 CO_CAN_ID_SDO_SRV + nodeId,
                                                 nodeId);
        if (ret != CO_SDO_RT_ok_communicationEnd)
        {
            result.abortCode = 0x08000000U;
            return result;
        }

        /* Initiate download */
        ret = CO_SDOclientDownloadInitiate(sdo, index, subIndex, data.size(), SDO_TRANSFER_TIMEOUT_MS, false);
        if (ret != CO_SDO_RT_ok_communicationEnd)
        {
            result.abortCode = 0x08000000U;
            return result;
        }

        /* Write data into the FIFO buffer */
        size_t written = CO_SDOclientDownloadBufWrite(sdo, data.data(), data.size());
        bool bufferPartial = (written < data.size());

        /* Process download */
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
        do
        {
            constexpr uint32_t pollInterval_us = 1000U;
            ret = CO_SDOclientDownload(sdo, pollInterval_us, false, bufferPartial, &abortCode, nullptr, nullptr);
            if (ret < 0)
            {
                result.abortCode = abortCode;
                CO_SDOclientClose(sdo);
                return result;
            }
            if (ret > 0)
            {
                tx_thread_sleep(1);
            }
        } while (ret > 0);

        result.success = true;
        result.bytesTransferred = data.size();
        result.abortCode = CO_SDO_AB_NONE;

        CO_SDOclientClose(sdo);
#else
        (void)nodeId;
        (void)index;
        (void)subIndex;
        result.abortCode = 0x08000000U;
#endif

        return result;
    }

    bool CANopen::isRunning() const
    {
        return initialized_ && co_ != nullptr && co_->CANmodule->CANnormal;
    }

} // namespace Implementations::HLDriver
