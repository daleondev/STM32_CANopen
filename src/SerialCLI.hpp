#pragma once

#include "Interfaces/HLDriver/ICANopen.hpp"
#include "Interfaces/HLDriver/ICiA402.hpp"

namespace SerialCLI
{

    /**
     * @brief Initialize the Serial CLI module.
     * @param canopen Reference to the high-level CANopen driver.
     * @param motor   Optional pointer to the CiA 402 motor driver (nullptr to disable motor commands).
     */
    void init(Interfaces::HLDriver::ICANopen &canopen,
              Interfaces::HLDriver::ICiA402 *motor = nullptr);

    /**
     * @brief Run the Serial CLI processing loop (blocking).
     *
     * Call this from the serial communication thread entry function.
     * Reads commands from UART and executes them.
     */
    [[noreturn]] void run();

} // namespace SerialCLI
