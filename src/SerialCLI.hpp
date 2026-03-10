#pragma once

#include "Interfaces/HLDriver/ICANopen.hpp"

namespace SerialCLI
{

    /**
     * @brief Initialize the Serial CLI module.
     * @param canopen Reference to the high-level CANopen driver.
     */
    void init(Interfaces::HLDriver::ICANopen &canopen);

    /**
     * @brief Run the Serial CLI processing loop (blocking).
     *
     * Call this from the serial communication thread entry function.
     * Reads commands from UART and executes them.
     */
    [[noreturn]] void run();

} // namespace SerialCLI
