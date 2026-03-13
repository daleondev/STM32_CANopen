#pragma once

#include "Interfaces/HLDriver/ICANopen.hpp"
#include "Interfaces/LLDriver/ICAN.hpp"

extern "C"
{
#include "CANopen.h"
#include "OD.h"
}

namespace Implementations::HLDriver
{

    /**
     * @brief High-level CANopen driver implementation.
     *
     * Manages the full CANopenNode stack lifecycle: allocation, initialization,
     * cyclic processing, NMT master commands, and SDO client transfers.
     */
    class CANopen final : public Interfaces::HLDriver::ICANopen
    {
    public:
        /**
         * @brief Construct with a reference to the low-level CAN driver.
         * @param ican Reference to the LL CAN driver (stored as CANptr for the port driver).
         */
        explicit CANopen(Interfaces::LLDriver::ICAN &ican);

        bool init(uint8_t nodeId, uint16_t bitRate) override;
        uint8_t process(uint32_t timeDifference_us) override;
        bool processSync(uint32_t timeDifference_us) override;
        bool sendNMTCommand(uint8_t command, uint8_t targetNodeId) override;
        Interfaces::HLDriver::SDOResult sdoRead(uint8_t nodeId, uint16_t index, uint8_t subIndex,
                                                std::span<uint8_t> buf) override;
        Interfaces::HLDriver::SDOResult sdoWrite(uint8_t nodeId, uint16_t index, uint8_t subIndex,
                                                 std::span<const uint8_t> data) override;
        bool isRunning() const override;

        /**
         * @brief Get the raw CANopenNode stack handle (for diagnostics).
         */
        CO_t *getCO() const { return co_; }

    private:
        Interfaces::LLDriver::ICAN &ican_;
        CO_t *co_{nullptr};
        uint8_t activeNodeId_{0};
        bool initialized_{false};

        /* NMT control flags */
        static constexpr uint16_t NMT_CONTROL =
            CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION;

        static constexpr uint16_t FIRST_HB_TIME_MS = 500;
        static constexpr uint16_t SDO_SRV_TIMEOUT_MS = 1000;
        static constexpr uint16_t SDO_CLI_TIMEOUT_MS = 500;
        static constexpr uint16_t SDO_TRANSFER_TIMEOUT_MS = 3000;
    };

} // namespace Implementations::HLDriver
