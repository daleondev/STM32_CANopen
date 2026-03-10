#include "Implementations/LLDriver/CAN.hpp"

#include <cstring>

extern "C"
{
#include "stm32h7xx_hal.h"
}

namespace Implementations::LLDriver
{

    CAN *CAN::instance_ = nullptr;

    CAN *CAN::instance()
    {
        return instance_;
    }

    CAN::CAN(FDCAN_HandleTypeDef *hfdcan) : hfdcan_{hfdcan}
    {
        instance_ = this;
    }

    bool CAN::init(uint16_t bitRate)
    {
        /* Stop peripheral if it was running */
        HAL_FDCAN_Stop(hfdcan_);
        HAL_FDCAN_DeInit(hfdcan_);

        /*
         * Reconfigure FDCAN init parameters.
         * CAN clock source is PLL1Q = 40 MHz (configured by CubeMX).
         * Bit timing: bitRate = CAN_CLK / (Prescaler * (1 + TimeSeg1 + TimeSeg2))
         *
         * For 500 kbit/s: 40 MHz / (4 * (1 + 15 + 4)) = 40 MHz / 80 = 500 kbit/s
         * For 250 kbit/s: 40 MHz / (8 * (1 + 15 + 4)) = 40 MHz / 160 = 250 kbit/s
         * For 125 kbit/s: 40 MHz / (16 * (1 + 15 + 4)) = 40 MHz / 320 = 125 kbit/s
         * For 1000 kbit/s: 40 MHz / (2 * (1 + 15 + 4)) = 40 MHz / 40 = 1000 kbit/s
         */
        constexpr uint32_t canClockHz = 40'000'000U;
        constexpr uint32_t tqPerBit = 1U + 15U + 4U; /* 20 TQ per bit */

        uint32_t prescaler = canClockHz / (static_cast<uint32_t>(bitRate) * 1000U * tqPerBit);
        if (prescaler == 0U)
        {
            prescaler = 1U;
        }

        hfdcan_->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
        hfdcan_->Init.Mode = FDCAN_MODE_NORMAL;
        hfdcan_->Init.AutoRetransmission = ENABLE;
        hfdcan_->Init.TransmitPause = DISABLE;
        hfdcan_->Init.ProtocolException = DISABLE;
        hfdcan_->Init.NominalPrescaler = prescaler;
        hfdcan_->Init.NominalSyncJumpWidth = 1;
        hfdcan_->Init.NominalTimeSeg1 = 15;
        hfdcan_->Init.NominalTimeSeg2 = 4;

        configureMessageRAM();

        if (HAL_FDCAN_Init(hfdcan_) != HAL_OK)
        {
            return false;
        }

        configureDefaultFilter();
        enableInterrupts();

        return true;
    }

    void CAN::configureMessageRAM()
    {
        /* Configure Message RAM: filters, RX FIFO, TX FIFO */
        hfdcan_->Init.StdFiltersNbr = 28;
        hfdcan_->Init.ExtFiltersNbr = 0;
        hfdcan_->Init.RxFifo0ElmtsNbr = 16;
        hfdcan_->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
        hfdcan_->Init.RxFifo1ElmtsNbr = 0;
        hfdcan_->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
        hfdcan_->Init.RxBuffersNbr = 0;
        hfdcan_->Init.RxBufferSize = FDCAN_DATA_BYTES_8;
        hfdcan_->Init.TxEventsNbr = 0;
        hfdcan_->Init.TxBuffersNbr = 0;
        hfdcan_->Init.TxFifoQueueElmtsNbr = 8;
        hfdcan_->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
        hfdcan_->Init.TxElmtSize = FDCAN_DATA_BYTES_8;
    }

    void CAN::configureDefaultFilter()
    {
        /* Accept all standard-ID messages into RX FIFO 0 */
        FDCAN_FilterTypeDef filter{};
        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterIndex = 0;
        filter.FilterType = FDCAN_FILTER_MASK;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter.FilterID1 = 0x000;
        filter.FilterID2 = 0x000; /* mask = 0 → accept all */
        HAL_FDCAN_ConfigFilter(hfdcan_, &filter);

        /* Reject all non-matching standard and extended frames */
        HAL_FDCAN_ConfigGlobalFilter(hfdcan_, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    }

    void CAN::enableInterrupts()
    {
        /* Enable RX FIFO 0 new-message interrupt */
        HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

        /* Enable TX complete interrupt for all TX buffers */
        HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);

        /* Enable RX FIFO 0 message-lost interrupt for overflow tracking */
        HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0);

        /* Enable FDCAN1 interrupt lines in NVIC */
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
        HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
    }

    bool CAN::configureFilter(uint16_t index, uint32_t id, uint32_t mask)
    {
        FDCAN_FilterTypeDef filter{};
        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterIndex = index;
        filter.FilterType = FDCAN_FILTER_MASK;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter.FilterID1 = id & 0x7FFU;
        filter.FilterID2 = mask & 0x7FFU;

        return HAL_FDCAN_ConfigFilter(hfdcan_, &filter) == HAL_OK;
    }

    bool CAN::send(const Interfaces::LLDriver::CANMessage &msg)
    {
        FDCAN_TxHeaderTypeDef txHeader{};
        txHeader.Identifier = msg.id & 0x7FFU;
        txHeader.IdType = FDCAN_STANDARD_ID;
        txHeader.TxFrameType = FDCAN_DATA_FRAME;

        /* Map DLC byte count to HAL FDCAN DLC constant */
        switch (msg.dlc)
        {
        case 0:
            txHeader.DataLength = FDCAN_DLC_BYTES_0;
            break;
        case 1:
            txHeader.DataLength = FDCAN_DLC_BYTES_1;
            break;
        case 2:
            txHeader.DataLength = FDCAN_DLC_BYTES_2;
            break;
        case 3:
            txHeader.DataLength = FDCAN_DLC_BYTES_3;
            break;
        case 4:
            txHeader.DataLength = FDCAN_DLC_BYTES_4;
            break;
        case 5:
            txHeader.DataLength = FDCAN_DLC_BYTES_5;
            break;
        case 6:
            txHeader.DataLength = FDCAN_DLC_BYTES_6;
            break;
        case 7:
            txHeader.DataLength = FDCAN_DLC_BYTES_7;
            break;
        default:
            txHeader.DataLength = FDCAN_DLC_BYTES_8;
            break;
        }

        txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        txHeader.BitRateSwitch = FDCAN_BRS_OFF;
        txHeader.FDFormat = FDCAN_CLASSIC_CAN;
        txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        txHeader.MessageMarker = 0;

        return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, const_cast<uint8_t *>(msg.data.data())) == HAL_OK;
    }

    void CAN::setNormalMode()
    {
        HAL_FDCAN_Start(hfdcan_);
    }

    void CAN::setConfigMode()
    {
        HAL_FDCAN_Stop(hfdcan_);
    }

    void CAN::disable()
    {
        HAL_FDCAN_Stop(hfdcan_);
        HAL_FDCAN_DeInit(hfdcan_);
    }

    void CAN::getErrorCounters(uint16_t &rxErr, uint16_t &txErr, uint16_t &overflow)
    {
        FDCAN_ErrorCountersTypeDef errCounters{};
        HAL_FDCAN_GetErrorCounters(hfdcan_, &errCounters);
        rxErr = static_cast<uint16_t>(errCounters.RxErrorCnt);
        txErr = static_cast<uint16_t>(errCounters.TxErrorCnt);
        overflow = overflowCount_;
    }

    void CAN::registerRxCallback(Interfaces::LLDriver::CANRxCallback callback, void *context)
    {
        rxCallback_ = callback;
        rxCallbackContext_ = context;
    }

    void CAN::registerTxCompleteCallback(Interfaces::LLDriver::CANTxCompleteCallback callback, void *context)
    {
        txCompleteCallback_ = callback;
        txCompleteCallbackContext_ = context;
    }

    void CAN::handleRxIRQ()
    {
        FDCAN_RxHeaderTypeDef rxHeader{};
        uint8_t rxData[8]{};

        while (HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
        {
            if (rxCallback_ != nullptr)
            {
                Interfaces::LLDriver::CANMessage msg{};
                msg.id = rxHeader.Identifier;

                /* Convert HAL DLC constant back to byte count */
                msg.dlc = static_cast<uint8_t>(rxHeader.DataLength >> 16U);

                std::memcpy(msg.data.data(), rxData, msg.dlc);
                rxCallback_(msg, rxCallbackContext_);
            }
        }
    }

    void CAN::handleTxCompleteIRQ()
    {
        if (txCompleteCallback_ != nullptr)
        {
            txCompleteCallback_(txCompleteCallbackContext_);
        }
    }

} // namespace Implementations::LLDriver

/*
 * HAL FDCAN ISR callbacks — route to the CAN driver instance.
 */
extern "C"
{
    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
    {
        (void)RxFifo0ITs;
        auto *can = Implementations::LLDriver::CAN::instance();
        if (can != nullptr)
        {
            if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
            {
                can->handleRxIRQ();
            }
        }
    }

    void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
    {
        (void)BufferIndexes;
        auto *can = Implementations::LLDriver::CAN::instance();
        if (can != nullptr)
        {
            can->handleTxCompleteIRQ();
        }
    }

    void FDCAN1_IT0_IRQHandler(void)
    {
        HAL_FDCAN_IRQHandler(&hfdcan1);
    }

    void FDCAN1_IT1_IRQHandler(void)
    {
        HAL_FDCAN_IRQHandler(&hfdcan1);
    }
}
