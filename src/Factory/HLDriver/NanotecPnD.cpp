#include "Factory/HLDriver/NanotecPnD.hpp"
#include "Implementations/HLDriver/NanotecPnD.hpp"

namespace Factory::HLDriver
{

    Interfaces::HLDriver::INanotecPnD &NanotecPnD::create(Interfaces::HLDriver::ICANopen &canopen)
    {
        static Implementations::HLDriver::NanotecPnD instance{canopen};
        return instance;
    }

} // namespace Factory::HLDriver
