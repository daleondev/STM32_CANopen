#pragma once

#include "Interfaces/HLDriver/INanotecPnD.hpp"
#include "Interfaces/HLDriver/ICANopen.hpp"

namespace Factory::HLDriver
{

    /**
     * @brief Factory for the minimal Nanotec Plug & Drive bringup driver.
     */
    class NanotecPnD
    {
    public:
        NanotecPnD() = delete;

        /**
         * @brief Create (or get) the singleton NanotecPnD instance.
         * @param canopen Reference to the CANopen driver.
         * @return Reference to the INanotecPnD interface.
         */
        static Interfaces::HLDriver::INanotecPnD &create(Interfaces::HLDriver::ICANopen &canopen);
    };

} // namespace Factory::HLDriver
