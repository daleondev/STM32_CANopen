#include "Factory/HLDriver/CANopen.hpp"
#include "Implementations/HLDriver/CANopen.hpp"

namespace Factory::HLDriver
{

    Interfaces::HLDriver::ICANopen &CANopen::create(Interfaces::LLDriver::ICAN &ican)
    {
        static Implementations::HLDriver::CANopen instance{ican};
        return instance;
    }

} // namespace Factory::HLDriver
