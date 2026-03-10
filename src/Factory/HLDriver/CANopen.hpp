#pragma once

#include "Interfaces/HLDriver/ICANopen.hpp"
#include "Interfaces/LLDriver/ICAN.hpp"

namespace Factory::HLDriver
{

    /**
     * @brief Factory for the high-level CANopen driver.
     *
     * Returns a reference to a statically-allocated CANopen driver instance
     * (no heap allocation for the driver object itself).
     */
    class CANopen
    {
    public:
        CANopen() = delete;

        /**
         * @brief Create (or get) the singleton CANopen driver instance.
         * @param ican Reference to the low-level CAN driver.
         * @return Reference to the ICANopen interface.
         */
        static Interfaces::HLDriver::ICANopen &create(Interfaces::LLDriver::ICAN &ican);
    };

} // namespace Factory::HLDriver
