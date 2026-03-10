#pragma once

#include <array>
#include <cstdint>

namespace Interfaces::LLDriver
{

    /**
     * @brief CAN message structure for send/receive operations.
     */
    struct CANMessage
    {
        uint32_t id{};
        uint8_t dlc{};
        std::array<uint8_t, 8> data{};
    };

    /**
     * @brief Callback type for received CAN messages (invoked from ISR context).
     */
    using CANRxCallback = void (*)(const CANMessage &msg, void *context);

    /**
     * @brief Callback type for TX-complete notification (invoked from ISR context).
     */
    using CANTxCompleteCallback = void (*)(void *context);

    /**
     * @brief Low-level CAN driver interface.
     *
     * Abstracts the STM32H7 FDCAN peripheral, providing basic send/receive
     * functionality to higher layers. Implementations must be ISR-safe where noted.
     */
    class ICAN
    {
    public:
        virtual ~ICAN() = default;

        /**
         * @brief Initialize the CAN peripheral at the given bitrate.
         * @param bitRate CAN bus bitrate in kbit/s (e.g. 500 for 500 kbit/s).
         * @return true on success.
         */
        virtual bool init(uint16_t bitRate) = 0;

        /**
         * @brief Configure a hardware acceptance filter.
         * @param index Filter bank index.
         * @param id Standard 11-bit CAN identifier to match.
         * @param mask Filter mask (1 = must match, 0 = don't care).
         * @return true on success.
         */
        virtual bool configureFilter(uint16_t index, uint32_t id, uint32_t mask) = 0;

        /**
         * @brief Transmit a single CAN frame.
         * @param msg The message to transmit.
         * @return true if the message was queued for transmission.
         */
        virtual bool send(const CANMessage &msg) = 0;

        /**
         * @brief Put the CAN peripheral into normal operating mode.
         */
        virtual void setNormalMode() = 0;

        /**
         * @brief Put the CAN peripheral into configuration mode.
         */
        virtual void setConfigMode() = 0;

        /**
         * @brief Disable and shut down the CAN peripheral.
         */
        virtual void disable() = 0;

        /**
         * @brief Read the current error counters from the CAN peripheral.
         * @param[out] rxErr Receive error counter.
         * @param[out] txErr Transmit error counter.
         * @param[out] overflow RX FIFO overflow count.
         */
        virtual void getErrorCounters(uint16_t &rxErr, uint16_t &txErr, uint16_t &overflow) = 0;

        /**
         * @brief Register a callback invoked from the RX ISR when a message is received.
         * @param callback Function pointer (must be ISR-safe).
         * @param context User context passed to callback.
         */
        virtual void registerRxCallback(CANRxCallback callback, void *context) = 0;

        /**
         * @brief Register a callback invoked from the TX-complete ISR.
         * @param callback Function pointer (must be ISR-safe).
         * @param context User context passed to callback.
         */
        virtual void registerTxCompleteCallback(CANTxCompleteCallback callback, void *context) = 0;
    };

} // namespace Interfaces::LLDriver
