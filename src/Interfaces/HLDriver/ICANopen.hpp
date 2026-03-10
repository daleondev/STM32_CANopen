#pragma once

#include <cstdint>
#include <span>

namespace Interfaces::HLDriver
{

    /**
     * @brief Result of an SDO transfer operation.
     */
    struct SDOResult
    {
        bool success{false};
        uint32_t abortCode{0};
        size_t bytesTransferred{0};
    };

    /**
     * @brief High-level CANopen driver interface.
     *
     * Manages the CANopenNode stack lifecycle, providing initialization,
     * cyclic processing, NMT master commands, and SDO client operations.
     */
    class ICANopen
    {
    public:
        virtual ~ICANopen() = default;

        /**
         * @brief Initialize the CANopen stack.
         * @param nodeId Local node ID (1..127), typically 0x7F for a master.
         * @param bitRate CAN bitrate in kbit/s.
         * @return true on success.
         */
        virtual bool init(uint8_t nodeId, uint16_t bitRate) = 0;

        /**
         * @brief Run the main asynchronous CANopen processing.
         * @param timeDifference_us Time since last call in microseconds.
         * @return NMT reset command (0 = none, 1 = comm reset, 2 = app reset).
         */
        virtual uint8_t process(uint32_t timeDifference_us) = 0;

        /**
         * @brief Run the time-critical synchronous processing (SYNC, RPDO, TPDO).
         * @param timeDifference_us Time since last call in microseconds.
         * @return true if a SYNC message was just received or transmitted.
         */
        virtual bool processSync(uint32_t timeDifference_us) = 0;

        /**
         * @brief Send an NMT command to a remote node.
         * @param command NMT command code (1=start, 2=stop, 128=pre-op, 129=reset node, 130=reset comm).
         * @param targetNodeId Target node ID (0 = all nodes).
         * @return true on success.
         */
        virtual bool sendNMTCommand(uint8_t command, uint8_t targetNodeId) = 0;

        /**
         * @brief Read a value from a remote node's Object Dictionary via SDO.
         * @param nodeId Remote node ID.
         * @param index OD index.
         * @param subIndex OD sub-index.
         * @param buf Output buffer for the read data.
         * @return SDOResult with success status, abort code, and bytes read.
         */
        virtual SDOResult sdoRead(uint8_t nodeId, uint16_t index, uint8_t subIndex, std::span<uint8_t> buf) = 0;

        /**
         * @brief Write a value to a remote node's Object Dictionary via SDO.
         * @param nodeId Remote node ID.
         * @param index OD index.
         * @param subIndex OD sub-index.
         * @param data Data to write.
         * @return SDOResult with success status and abort code.
         */
        virtual SDOResult sdoWrite(uint8_t nodeId, uint16_t index, uint8_t subIndex, std::span<const uint8_t> data) = 0;

        /**
         * @brief Check if the CANopen stack is running in normal mode.
         * @return true if running.
         */
        virtual bool isRunning() const = 0;
    };

} // namespace Interfaces::HLDriver
