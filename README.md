# STM32_CANopen

This projects implements a CANopen master on a NUCLEO-H753ZI board. Its goal is controlling a CANopen slave device, in this case a cia 402 motor controller, using the CANopen protocol. The project is written in C++20.

## Driver Structure

The CANopen protocol is implemented in a layered architecture, with the following layers:
1. **Low Level CAN Driver**: This layer interfaces directly with the CAN hardware, providing basic send and receive functionality. It abstracts away the specifics of the STM32H753ZI's CAN peripheral.
2. **CANopenNode Library Port Driver**: This layer implements the necessary functions to interface with the CANopenNode library, which is a popular open-source CANopen stack. It translates the CANopenNode API calls into calls to the low-level CAN driver.
3. **High Level CANopen Driver**: This layer implements the CANopen protocol logic, including handling of CANopen messages, state machines, and application logic. It uses the CANopenNode library for protocol handling and relies on the port driver for communication.

The layering looks as follows:
     |----------------------|
     | Low Level CAN Driver |
     |----------------------|
                |
                v
|---------------------------------|
| CANopenNode Library Port Driver |
|---------------------------------|
                |
                v
   |---------------------------|
   | High Level CANopen Driver |
   |---------------------------|

## Driver Source Files

### Low Level CAN Driver
- `src/Interfaces/LLDriver/ICAN.hpp`: The header file for the low-level CAN driver interface, defining the necessary functions and data structures for CAN communication.
- `src/Implementations/LLDriver/CAN.hpp`: The header file for the low-level CAN driver implementation, declaring the class that implements the CAN driver interface and its member functions.
- `src/Implementations/LLDriver/CAN.cpp`: The implementation of the low-level CAN driver, providing the actual functionality for sending and receiving CAN messages.
- `src/Factory/LLDriver/CAN.hpp`: The header file for the low-level CAN driver factory, declaring the class that creates instances of the low-level CAN driver.
- `src/Factory/LLDriver/CAN.cpp`: The implementation of the low-level CAN driver factory, providing the actual functionality for creating low-level CAN driver instances.

### CANopenNode Library Port Driver
- `extern/CANopenNode/CANopenNode_port/CO_driver`: The ported CANopenNode driver source files, which implement the necessary functions to interface with the CANopenNode library.

### High Level CANopen Driver
- `src/Interfaces/HLDriver/ICANopen.hpp`: The header file for the high-level CANopen driver interface, defining the necessary functions and data structures for CANopen communication.
- `src/Implementations/HLDriver/CANopen.hpp`: The header file for the high-level CANopen driver implementation, declaring the class that implements the CANopen driver interface and its member functions.
- `src/Implementations/HLDriver/CANopen.cpp`: The implementation of the high-level CANopen driver, providing the actual functionality for CANopen communication.
- `src/Factory/HLDriver/CANopen.hpp`: The header file for the high-level CANopen driver factory, declaring the class that creates instances of the CANopen driver.
- `src/Factory/HLDriver/CANopen.cpp`: The implementation of the high-level CANopen driver factory, providing the actual functionality for creating CANopen driver instances.

**Note:** Low Level and High Level Drivers are guarded by namespaces e.g. Interfaces::LLDriver::ICAN, Implementations::LLDriver::CAN, Factory::LLDriver::CAN, ...

## Threads
The project uses Threadx as the RTOS, and the following threads are implemented:
- **Main Thread**: This thread is responsible for initializing the system, including setting up the CANopen driver and starting the CANopen communication thread.
- **CANopen Thread**: This thread is responsible for handling CANopen communication, including sending
- **Serial Communication Thread**: This thread is responsible for handling serial communication, allowing for debugging and interaction with the system via a serial interface.

## User Interaction
The user can interact with the system via a serial interface, which allows for sending commands and receiving responses. The serial communication thread handles this interaction, providing a simple command-line interface for controlling the CANopen communication and monitoring the system status.