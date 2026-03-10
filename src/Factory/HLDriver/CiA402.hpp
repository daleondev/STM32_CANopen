#pragma once

#include "Interfaces/HLDriver/ICiA402.hpp"
#include "Interfaces/HLDriver/ICANopen.hpp"

namespace Factory::HLDriver
{

    /**
     * @brief Factory for the CiA 402 motor driver.
     *
     * Returns a reference to a statically-allocated CiA402 instance.
     */
    class CiA402
    {
    public:
        CiA402() = delete;

        /**
         * @brief Create (or get) the singleton CiA402 driver instance.
         * @param canopen Reference to the CANopen driver.
         * @return Reference to the ICiA402 interface.
         */
        static Interfaces::HLDriver::ICiA402 &create(Interfaces::HLDriver::ICANopen &canopen);
    };

} // namespace Factory::HLDriver
