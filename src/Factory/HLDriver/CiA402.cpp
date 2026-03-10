#include "Factory/HLDriver/CiA402.hpp"
#include "Implementations/HLDriver/CiA402.hpp"

namespace Factory::HLDriver
{

    Interfaces::HLDriver::ICiA402 &CiA402::create(Interfaces::HLDriver::ICANopen &canopen)
    {
        static Implementations::HLDriver::CiA402 instance{canopen};
        return instance;
    }

} // namespace Factory::HLDriver
