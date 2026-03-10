#pragma once

#include "Interfaces/LLDriver/ICAN.hpp"

extern "C"
{
#include "fdcan.h"
}

namespace Implementations::LLDriver
{

    /**
     * @brief Low-level CAN driver implementation for STM32H7 FDCAN peripheral.
     *
     * Wraps the HAL FDCAN API, configuring Message RAM, filters, and interrupts.
     * Delegates RX/TX events to registered callbacks from ISR context.
     */
    class CAN final : public Interfaces::LLDriver::ICAN
    {
    public:
        /**
         * @brief Construct with a pointer to the HAL FDCAN handle.
         * @param hfdcan Pointer to the FDCAN_HandleTypeDef (e.g. &hfdcan1).
         */
        explicit CAN(FDCAN_HandleTypeDef *hfdcan);

        bool init(uint16_t bitRate) override;
        bool configureFilter(uint16_t index, uint32_t id, uint32_t mask) override;
        bool send(const Interfaces::LLDriver::CANMessage &msg) override;
        void setNormalMode() override;
        void setConfigMode() override;
        void disable() override;
        void getErrorCounters(uint16_t &rxErr, uint16_t &txErr, uint16_t &overflow) override;
        void registerRxCallback(Interfaces::LLDriver::CANRxCallback callback, void *context) override;
        void registerTxCompleteCallback(Interfaces::LLDriver::CANTxCompleteCallback callback, void *context) override;

        /**
         * @brief Called from HAL_FDCAN_RxFifo0Callback ISR.
         */
        void handleRxIRQ();

        /**
         * @brief Called from HAL_FDCAN_TxBufferCompleteCallback ISR.
         */
        void handleTxCompleteIRQ();

        /**
         * @brief Get the singleton-like instance pointer for ISR routing.
         */
        static CAN *instance();

    private:
        FDCAN_HandleTypeDef *hfdcan_;
        Interfaces::LLDriver::CANRxCallback rxCallback_{nullptr};
        void *rxCallbackContext_{nullptr};
        Interfaces::LLDriver::CANTxCompleteCallback txCompleteCallback_{nullptr};
        void *txCompleteCallbackContext_{nullptr};
        uint16_t overflowCount_{0};

        static CAN *instance_;

        void configureMessageRAM();
        void configureDefaultFilter();
        void enableInterrupts();
    };

} // namespace Implementations::LLDriver
