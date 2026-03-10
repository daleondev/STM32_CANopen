#include "Factory/LLDriver/CAN.hpp"
#include "Implementations/LLDriver/CAN.hpp"

namespace Factory::LLDriver
{

    Interfaces::LLDriver::ICAN &CAN::create(FDCAN_HandleTypeDef *hfdcan)
    {
        static Implementations::LLDriver::CAN instance{hfdcan};
        return instance;
    }

} // namespace Factory::LLDriver
