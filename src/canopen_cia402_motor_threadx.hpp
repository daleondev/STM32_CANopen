#pragma once

#include <lely/can/msg.h>

#include <tx_api.h>

#include <cstdint>
#include <memory>
#include <string_view>

// Runtime knobs for the embedded pump sample.
//
// These values are intentionally small and easy to reason about so a board
// integration can override only the pieces that matter: node IDs, speed,
// timing and ThreadX priorities.
struct ThreadXCia402Config
{
    uint8_t master_id{1};
    uint8_t node_id{2};
    int32_t target_velocity{1000};
    uint32_t profile_acceleration{2000};
    uint32_t profile_deceleration{2000};
    uint32_t run_time_ms{2000};
    uint32_t boot_settle_time_ms{250};
    UINT transport_thread_priority{16};
    UINT timer_thread_priority{16};
    size_t rx_queue_depth{32};
};

class ThreadXCia402Logger
{
public:
    virtual ~ThreadXCia402Logger() = default;

    // Informational messages for high-level state transitions.
    virtual void Info(std::string_view message) = 0;

    // Error messages for setup failures, SDO failures and shutdown paths.
    virtual void Error(std::string_view message) = 0;
};

// Callback sink implemented by the Lely bridge. A concrete CAN driver calls
// `OnCanMessage()` whenever a full CAN frame was received from hardware.
class ThreadXCanReceiver
{
public:
    virtual ~ThreadXCanReceiver() = default;

    virtual bool OnCanMessage(const can_msg &msg) = 0;
};

// Thin hardware abstraction for the board-specific CAN/FDCAN backend.
//
// The sample owns the CANopen logic and ThreadX worker threads, while the
// application provides the actual controller-specific implementation.
class ThreadXCanPort
{
public:
    virtual ~ThreadXCanPort() = default;

    // Called once during startup so the CAN backend knows where to deliver
    // received frames.
    virtual void SetReceiver(ThreadXCanReceiver *receiver) = 0;

    // Bring the hardware interface online.
    virtual bool Start() = 0;

    // Stop RX/TX activity and release board-specific resources.
    virtual void Stop() = 0;

    // Transmit a single CAN frame. The timeout is already converted to
    // ThreadX ticks by the sample.
    virtual bool Write(const can_msg &msg, ULONG timeout_ticks) = 0;
};

// High-level wrapper around the complete embedded sample. Users only need
// to provide configuration, a CAN backend and optionally a logger.
class ThreadXCia402MotorApp
{
public:
    ThreadXCia402MotorApp(ThreadXCia402Config config,
                          ThreadXCanPort &can_port,
                          ThreadXCia402Logger *logger = nullptr);
    ~ThreadXCia402MotorApp();

    ThreadXCia402MotorApp(const ThreadXCia402MotorApp &) = delete;
    ThreadXCia402MotorApp &operator=(const ThreadXCia402MotorApp &) = delete;

    ThreadXCia402MotorApp(ThreadXCia402MotorApp &&) noexcept;
    ThreadXCia402MotorApp &operator=(ThreadXCia402MotorApp &&) noexcept;

    int Run();
    void RequestStop();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};