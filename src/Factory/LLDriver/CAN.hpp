#pragma once

#include "Interfaces/LLDriver/ICAN.hpp"

extern "C"
{
#include "stm32h7xx_hal.h"
}

namespace Factory::LLDriver
{

    /**
     * @brief Factory for the low-level CAN driver.
     *
     * Returns a reference to a statically-allocated CAN driver instance
     * (no heap allocation).
     */
    class CAN
    {
    public:
        CAN() = delete;

        /**
         * @brief Create (or get) the singleton CAN driver instance.
         * @param hfdcan Pointer to the HAL FDCAN handle.
         * @return Reference to the ICAN interface.
         */
        static Interfaces::LLDriver::ICAN &create(FDCAN_HandleTypeDef *hfdcan);
    };

} // namespace Factory::LLDriver
