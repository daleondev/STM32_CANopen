#include "canopen_cia402_motor_threadx.hpp"

#include <lely/co/dev.h>
#include <lely/co/obj.h>
#include <lely/co/type.h>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/ctx.hpp>
#include <lely/io2/user/can.hpp>
#include <lely/io2/user/timer.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

    using namespace std::chrono_literals;

    // Core CANopen objects and status masks used by the pump sample.
    // Keeping them together makes the CiA 402 flow easier to follow below.
    constexpr auto kPumpName = "PD4-C5918L4204";
    constexpr ULONG kThreadSleepTicks = 1;
    constexpr size_t kTransportStackSize = 4096;
    constexpr size_t kTimerStackSize = 2048;
    constexpr UINT kQueueMessageWords = 1;
    constexpr uint16_t kIdxControlword = 0x6040;
    constexpr uint16_t kIdxStatusword = 0x6041;
    constexpr uint16_t kIdxModesOfOperation = 0x6060;
    constexpr uint16_t kIdxModesOfOperationDisplay = 0x6061;
    constexpr uint16_t kIdxVelocityActualValue = 0x606c;
    constexpr uint16_t kIdxTargetVelocity = 0x60ff;
    constexpr uint16_t kIdxProfileAcceleration = 0x6083;
    constexpr uint16_t kIdxProfileDeceleration = 0x6084;
    constexpr uint16_t kStatusMaskState = 0x006fu;
    constexpr uint16_t kStatusReadyToSwitchOn = 0x0021u;
    constexpr uint16_t kStatusSwitchedOn = 0x0023u;
    constexpr uint16_t kStatusOperationEnabled = 0x0027u;
    constexpr uint16_t kStatusFaultBit = 0x0008u;
    constexpr int32_t kStoppedVelocityWindow = 50;

    void
    ThrowIf(bool condition, const std::string &message)
    {
        if (condition)
            throw std::runtime_error(message);
    }

    void
    RequireRc(int rc, const std::string &message)
    {
        if (rc == -1)
            throw std::runtime_error(message);
    }

    void
    RequireSize(size_t size, const std::string &message)
    {
        if (size == 0)
            throw std::runtime_error(message);
    }

    ULONG
    MillisecondsToTicks(int timeout_ms)
    {
        if (timeout_ms < 0)
            return TX_WAIT_FOREVER;

        if (timeout_ms == 0)
            return TX_NO_WAIT;

        const auto ticks = (static_cast<unsigned long long>(timeout_ms) * TX_TIMER_TICKS_PER_SECOND + 999ull) / 1000ull;
        return ticks == 0 ? 1u : static_cast<ULONG>(ticks);
    }

    long long
    MonotonicNanoseconds()
    {
        const auto ticks = static_cast<unsigned long long>(tx_time_get());
        return static_cast<long long>((ticks * 1000000000ull) / TX_TIMER_TICKS_PER_SECOND);
    }

    lely::io::Clock::time_point
    CurrentTimePoint()
    {
        return lely::io::Clock::time_point(lely::io::Clock::duration(MonotonicNanoseconds()));
    }

    co_obj_t *
    MakeObject(uint16_t idx, uint8_t code, const char *name)
    {
        auto *obj = co_obj_create(idx);
        ThrowIf(!obj, "unable to create CANopen object");
        try
        {
            RequireRc(co_obj_set_code(obj, code), "unable to set CANopen object code");
            if (name)
                RequireRc(co_obj_set_name(obj, name), "unable to set CANopen object name");
            return obj;
        }
        catch (...)
        {
            co_obj_destroy(obj);
            throw;
        }
    }

    void
    InsertObject(co_dev_t *dev, co_obj_t *obj)
    {
        if (co_dev_insert_obj(dev, obj) == -1)
        {
            co_obj_destroy(obj);
            throw std::runtime_error("unable to insert CANopen object into device dictionary");
        }
    }

    co_sub_t *
    MakeSub(uint8_t subidx, uint16_t type, unsigned access, const char *name)
    {
        auto *sub = co_sub_create(subidx, type);
        ThrowIf(!sub, "unable to create CANopen sub-object");
        try
        {
            RequireRc(co_sub_set_access(sub, access), "unable to set CANopen access type");
            if (name)
                RequireRc(co_sub_set_name(sub, name), "unable to set CANopen sub-object name");
            return sub;
        }
        catch (...)
        {
            co_sub_destroy(sub);
            throw;
        }
    }

    void
    InsertSub(co_obj_t *obj, co_sub_t *sub)
    {
        if (co_obj_insert_sub(obj, sub) == -1)
        {
            co_sub_destroy(sub);
            throw std::runtime_error("unable to insert CANopen sub-object");
        }
    }

    void
    AddSubU8(co_obj_t *obj, uint8_t subidx, unsigned access, uint8_t value, const char *name)
    {
        auto *sub = MakeSub(subidx, CO_DEFTYPE_UNSIGNED8, access, name);
        try
        {
            RequireSize(co_sub_set_val_u8(sub, value), "unable to set UNSIGNED8 value");
            RequireSize(co_sub_set_def(sub, &value, sizeof(value)), "unable to set UNSIGNED8 default value");
            InsertSub(obj, sub);
        }
        catch (...)
        {
            co_sub_destroy(sub);
            throw;
        }
    }

    void
    AddSubU32(co_obj_t *obj, uint8_t subidx, unsigned access, uint32_t value, const char *name)
    {
        auto *sub = MakeSub(subidx, CO_DEFTYPE_UNSIGNED32, access, name);
        try
        {
            RequireSize(co_sub_set_val_u32(sub, value), "unable to set UNSIGNED32 value");
            RequireSize(co_sub_set_def(sub, &value, sizeof(value)), "unable to set UNSIGNED32 default value");
            InsertSub(obj, sub);
        }
        catch (...)
        {
            co_sub_destroy(sub);
            throw;
        }
    }

    __co_dev *
    CreateMasterDictionary(uint8_t master_id)
    {
        // The sample only needs a tiny local dictionary for the CANopen master.
        // A larger application can replace this with a project-specific DCF.
        std::unique_ptr<co_dev_t, decltype(&co_dev_destroy)> dev(co_dev_create(master_id), &co_dev_destroy);
        ThrowIf(!dev, "unable to create CANopen master dictionary");

        {
            auto *obj = MakeObject(0x1000, CO_OBJECT_VAR, "Device type");
            AddSubU32(obj, 0x00, CO_ACCESS_RO, 0x00000000u, "Device type");
            InsertObject(dev.get(), obj);
        }

        {
            auto *obj = MakeObject(0x1001, CO_OBJECT_VAR, "Error register");
            AddSubU8(obj, 0x00, CO_ACCESS_RO, 0x00u, "Error register");
            InsertObject(dev.get(), obj);
        }

        {
            auto *obj = MakeObject(0x1018, CO_OBJECT_RECORD, "Identity object");
            AddSubU8(obj, 0x00, CO_ACCESS_CONST, 0x04u, "Highest sub-index supported");
            AddSubU32(obj, 0x01, CO_ACCESS_RO, 0x00000000u, "Vendor-ID");
            AddSubU32(obj, 0x02, CO_ACCESS_RO, 0x00000000u, "Product code");
            AddSubU32(obj, 0x03, CO_ACCESS_RO, 0x00000000u, "Revision number");
            AddSubU32(obj, 0x04, CO_ACCESS_RO, 0x00000000u, "Serial number");
            InsertObject(dev.get(), obj);
        }

        return reinterpret_cast<__co_dev *>(dev.release());
    }

    class LogSink
    {
    public:
        explicit LogSink(ThreadXCia402Logger *logger)
            : logger_(logger)
        {
        }

        void Info(const std::string &message) const
        {
            if (logger_)
                logger_->Info(message);
        }

        void Error(const std::string &message) const
        {
            if (logger_)
                logger_->Error(message);
        }

    private:
        ThreadXCia402Logger *logger_{};
    };

    class TimerPump
    {
    public:
        // Lely's user timer needs a progressing clock source. This helper uses a
        // small ThreadX thread to periodically advance that clock from the RTOS
        // tick counter.
        TimerPump(lely::io::Context &ctx,
                  lely::ev::Executor exec,
                  UINT priority,
                  const LogSink &log)
            : timer_(ctx, exec), priority_(priority), log_(log)
        {
            const auto status = tx_thread_create(&thread_,
                                                 const_cast<char *>("lely_timer"),
                                                 &ThreadEntry,
                                                 reinterpret_cast<ULONG>(this),
                                                 stack_.data(),
                                                 sizeof(stack_),
                                                 priority_,
                                                 priority_,
                                                 TX_NO_TIME_SLICE,
                                                 TX_AUTO_START);
            ThrowIf(status != TX_SUCCESS, "unable to create ThreadX timer pump thread");
        }

        ~TimerPump()
        {
            Stop();
        }

        lely::io::UserTimer &timer() noexcept
        {
            return timer_;
        }

        void Stop()
        {
            if (!started_)
                return;

            running_ = false;
            while (!stopped_)
                tx_thread_sleep(kThreadSleepTicks);

            tx_thread_terminate(&thread_);
            tx_thread_delete(&thread_);
            started_ = false;
        }

    private:
        static void ThreadEntry(ULONG arg)
        {
            static_cast<TimerPump *>(reinterpret_cast<void *>(arg))->Run();
        }

        void Run()
        {
            // The loop is intentionally simple: update the clock once per RTOS
            // tick and let Lely consume the new time base.
            while (running_)
            {
                timer_.get_clock().settime(CurrentTimePoint());
                tx_thread_sleep(kThreadSleepTicks);
            }

            stopped_ = true;
        }

        lely::io::UserTimer timer_;
        UINT priority_{};
        const LogSink &log_;
        TX_THREAD thread_{};
        alignas(ULONG) std::array<std::byte, kTimerStackSize> stack_{};
        std::atomic<bool> running_{true};
        std::atomic<bool> stopped_{false};
        bool started_{true};
    };

    class Cia402Driver final : public lely::canopen::FiberDriver
    {
    public:
        using CompletedHandler = std::function<void()>;
        using FailedHandler = std::function<void(const std::string &)>;

        Cia402Driver(lely::canopen::AsyncMaster &master,
                     uint8_t node_id,
                     ThreadXCia402Config config,
                     CompletedHandler on_completed,
                     FailedHandler on_failed,
                     const LogSink &log)
            : FiberDriver(master, node_id),
              master_(master),
              config_(config),
              on_completed_(std::move(on_completed)),
              on_failed_(std::move(on_failed)),
              log_(log)
        {
        }

        void Start()
        {
            // The actual sequence must run on the FiberDriver strand so that
            // `Wait()` and `USleep()` suspend only this driver flow.
            Defer([this]()
                  {
                      try
                      {
                          Log(std::string{"Starting "} + kPumpName + " pump sample");
                          USleep(config_.boot_settle_time_ms * 1000u);
                          master_.Command(lely::canopen::NmtCommand::ENTER_PREOP, id());
                          USleep(config_.boot_settle_time_ms * 1000u);
                          RunPumpCycle();
                          if (on_completed_)
                              on_completed_();
                      }
                      catch (const lely::canopen::SdoError &e)
                      {
                          std::ostringstream oss;
                          oss << "SDO error on node " << unsigned(id()) << ": index 0x"
                              << std::hex << e.idx() << ':' << unsigned(e.subidx()) << std::dec
                              << " -> " << e.code().message();
                          if (on_failed_)
                              on_failed_(oss.str());
                      }
                      catch (const std::exception &e)
                      {
                          if (on_failed_)
                              on_failed_(e.what());
                      } });
        }

    private:
        static constexpr int8_t kProfileVelocityMode = 3;

        void RunPumpCycle()
        {
            // The sample is intentionally structured as a readable procedure:
            // prepare -> enable -> run -> stop.
            ResetFaultIfNeeded();
            ConfigurePumpProfile();

            Log("Putting node into NMT operational");
            master_.Command(lely::canopen::NmtCommand::START, id());
            USleep(100000);

            TransitionToOperationEnabled();
            StartPump();

            std::ostringstream run_oss;
            run_oss << "Pump running for " << config_.run_time_ms << " ms";
            Log(run_oss.str());
            USleep(config_.run_time_ms * 1000u);

            StopPump();
            Log("Pump sample completed");
        }

        void ResetFaultIfNeeded()
        {
            // A previous drive fault would make the normal CiA 402 enable path
            // fail, so the sample clears it first when necessary.
            if ((ReadStatusWord() & kStatusFaultBit) == 0)
                return;

            Log("Drive fault detected, sending fault reset");
            WriteControlword(0x0080u);
            WaitForStatusBit(kStatusFaultBit, false, 1s);
        }

        void ConfigurePumpProfile()
        {
            // For a pump demo we keep the mode fixed to profile velocity and set
            // only the most important ramps.
            Log("Configuring profile velocity mode");
            Wait(AsyncWrite(kIdxModesOfOperation, 0x00, int8_t{kProfileVelocityMode}));
            WaitForMode(kProfileVelocityMode, 1s);

            std::ostringstream oss;
            oss << "Applying accel/decel ramps: " << config_.profile_acceleration << '/' << config_.profile_deceleration;
            Log(oss.str());

            Wait(AsyncWrite(kIdxProfileAcceleration, 0x00, config_.profile_acceleration));
            Wait(AsyncWrite(kIdxProfileDeceleration, 0x00, config_.profile_deceleration));
        }

        void TransitionToOperationEnabled()
        {
            // Standard CiA 402 state progression. The waits after each write are
            // what make the sequence robust and easy to debug.
            Log("Transitioning drive to ready-to-switch-on");
            WriteControlword(0x0006u);
            WaitForStatus(kStatusMaskState, kStatusReadyToSwitchOn, 2s);

            Log("Transitioning drive to switched-on");
            WriteControlword(0x0007u);
            WaitForStatus(kStatusMaskState, kStatusSwitchedOn, 2s);

            Log("Transitioning drive to operation-enabled");
            WriteControlword(0x000fu);
            WaitForStatus(kStatusMaskState, kStatusOperationEnabled, 2s);
        }

        void StartPump()
        {
            // Command the run speed and wait until the reported feedback shows
            // that the motor has really started moving.
            std::ostringstream oss;
            oss << "Commanding target velocity " << config_.target_velocity;
            Log(oss.str());

            Wait(AsyncWrite(kIdxTargetVelocity, 0x00, config_.target_velocity));
            WaitForVelocity([](int32_t actual, int32_t target)
                            { return std::abs(actual) >= std::max<int32_t>(std::abs(target) / 2, 1); },
                            2s,
                            "pump to accelerate");
        }

        void StopPump()
        {
            // A clean stop is modelled as zero target velocity followed by a
            // wait for the measured speed to fall into a small deadband.
            Log("Commanding zero velocity");
            Wait(AsyncWrite<int32_t>(kIdxTargetVelocity, 0x00, 0));
            WaitForVelocity([](int32_t actual, int32_t)
                            { return std::abs(actual) <= kStoppedVelocityWindow; },
                            3s,
                            "pump to stop");

            Log("Returning drive to shutdown state");
            WriteControlword(0x0006u);
            WaitForStatus(kStatusMaskState, kStatusReadyToSwitchOn, 2s);
        }

        void WriteControlword(uint16_t value)
        {
            // Thin helper so the higher-level sequence reads like the CiA 402
            // state machine instead of raw SDO boilerplate.
            Wait(AsyncWrite(kIdxControlword, 0x00, value));
        }

        uint16_t ReadStatusWord()
        {
            return Wait(AsyncRead<uint16_t>(kIdxStatusword, 0x00));
        }

        int8_t ReadModeDisplay()
        {
            return Wait(AsyncRead<int8_t>(kIdxModesOfOperationDisplay, 0x00));
        }

        int32_t ReadActualVelocity()
        {
            return Wait(AsyncRead<int32_t>(kIdxVelocityActualValue, 0x00));
        }

        void WaitForMode(int8_t expected, std::chrono::milliseconds timeout)
        {
            // Polling is acceptable here because this sample only needs a short,
            // deterministic setup sequence.
            const auto deadline = std::chrono::steady_clock::now() + timeout;
            while (std::chrono::steady_clock::now() < deadline)
            {
                if (ReadModeDisplay() == expected)
                    return;
                USleep(20000);
            }

            std::ostringstream oss;
            oss << "timeout while waiting for mode display " << int(expected);
            throw std::runtime_error(oss.str());
        }

        void WaitForStatus(uint16_t mask, uint16_t expected, std::chrono::milliseconds timeout)
        {
            // The statusword is the main source of truth for the CiA 402 state
            // machine, so each transition waits until the expected bits appear.
            const auto deadline = std::chrono::steady_clock::now() + timeout;
            while (std::chrono::steady_clock::now() < deadline)
            {
                if ((ReadStatusWord() & mask) == expected)
                    return;
                USleep(20000);
            }

            std::ostringstream oss;
            oss << "timeout while waiting for statusword mask 0x" << std::hex << mask
                << " == 0x" << expected;
            throw std::runtime_error(oss.str());
        }

        void WaitForStatusBit(uint16_t bit, bool expected_set, std::chrono::milliseconds timeout)
        {
            // Specialized variant used for simple single-bit conditions, such as
            // waiting for the fault bit to clear.
            const auto deadline = std::chrono::steady_clock::now() + timeout;
            while (std::chrono::steady_clock::now() < deadline)
            {
                const auto set = (ReadStatusWord() & bit) != 0;
                if (set == expected_set)
                    return;
                USleep(20000);
            }

            std::ostringstream oss;
            oss << "timeout while waiting for status bit 0x" << std::hex << bit;
            throw std::runtime_error(oss.str());
        }

        template <class Predicate>
        void WaitForVelocity(Predicate predicate,
                             std::chrono::milliseconds timeout,
                             const char *action)
        {
            // The sample uses velocity feedback as a practical confirmation that
            // the commanded run/stop request was actually executed by the drive.
            const auto deadline = std::chrono::steady_clock::now() + timeout;
            while (std::chrono::steady_clock::now() < deadline)
            {
                const auto actual = ReadActualVelocity();
                if (predicate(actual, config_.target_velocity))
                    return;
                USleep(50000);
            }

            std::ostringstream oss;
            oss << "timeout while waiting for " << action;
            throw std::runtime_error(oss.str());
        }

        void Log(const std::string &message) const
        {
            log_.Info(message);
        }

        void OnCanState(lely::io::CanState, lely::io::CanState) noexcept override {}
        void OnCanError(lely::io::CanError) noexcept override {}
        void OnRpdoWrite(uint16_t, uint8_t) noexcept override {}
        void OnCommand(lely::canopen::NmtCommand) noexcept override {}
        void OnHeartbeat(bool) noexcept override {}
        void OnState(lely::canopen::NmtState) noexcept override {}
        void OnSync(uint8_t, const time_point &) noexcept override {}
        void OnSyncError(uint16_t, uint8_t) noexcept override {}
        void OnTime(const std::chrono::system_clock::time_point &) noexcept override {}
        void OnEmcy(uint16_t, uint8_t, uint8_t[5]) noexcept override {}
        void OnNodeGuarding(bool) noexcept override {}
        void OnBoot(lely::canopen::NmtState, char, const std::string &) noexcept override {}
        void OnConfig(std::function<void(std::error_code ec)> res) noexcept override { res({}); }
        void OnDeconfig(std::function<void(std::error_code ec)> res) noexcept override { res({}); }

        lely::canopen::AsyncMaster &master_;
        ThreadXCia402Config config_;
        CompletedHandler on_completed_;
        FailedHandler on_failed_;
        const LogSink &log_;
    };

} // namespace

class ThreadXCia402MotorApp::Impl
{
public:
    class CanChannelBridge final : public ThreadXCanReceiver
    {
    public:
        // This bridge isolates the board CAN driver from Lely's user-space
        // channel. RX frames are queued first so they are not injected into
        // Lely from an arbitrary hardware callback context.
        CanChannelBridge(lely::io::Context &ctx,
                         lely::ev::Executor exec,
                         ThreadXCanPort &port,
                         const ThreadXCia402Config &config,
                         const LogSink &log)
            : port_(port),
              log_(log),
              queue_storage_(config.rx_queue_depth),
              channel_(ctx, exec, lely::io::CanBusFlag::NONE, config.rx_queue_depth, 0, &WriteFrame, this),
              priority_(config.transport_thread_priority)
        {
            port_.SetReceiver(this);

            const auto queue_status = tx_queue_create(&queue_,
                                                      const_cast<char *>("lely_can_rx"),
                                                      1u,
                                                      queue_storage_.data(),
                                                      static_cast<ULONG>(queue_storage_.size() * sizeof(ULONG)));
            ThrowIf(queue_status != TX_SUCCESS, "unable to create ThreadX CAN receive queue");

            const auto thread_status = tx_thread_create(&thread_,
                                                        const_cast<char *>("lely_can_rx"),
                                                        &ThreadEntry,
                                                        reinterpret_cast<ULONG>(this),
                                                        stack_.data(),
                                                        sizeof(stack_),
                                                        priority_,
                                                        priority_,
                                                        TX_NO_TIME_SLICE,
                                                        TX_AUTO_START);
            ThrowIf(thread_status != TX_SUCCESS, "unable to create ThreadX CAN receive thread");
        }

        ~CanChannelBridge()
        {
            Stop();
        }

        lely::io::UserCanChannel &channel() noexcept
        {
            return channel_;
        }

        void Start()
        {
            ThrowIf(!port_.Start(), "unable to start ThreadX CAN port");
        }

        bool OnCanMessage(const can_msg &msg) override
        {
            // ThreadX queues can only carry up to 16 ULONGs per message.
            // A CAN FD-capable `can_msg` is larger than that, so queue a
            // pointer to a heap copy instead of the full frame payload.
            auto *copy = new (std::nothrow) can_msg(msg);
            if (!copy)
            {
                log_.Error("Unable to allocate CAN receive frame");
                return false;
            }

            ULONG word = reinterpret_cast<ULONG>(copy);
            if (tx_queue_send(&queue_, &word, TX_NO_WAIT) != TX_SUCCESS)
            {
                delete copy;
                return false;
            }

            return true;
        }

        void Stop()
        {
            if (!started_)
                return;

            port_.Stop();
            running_ = false;
            while (!stopped_)
                tx_thread_sleep(kThreadSleepTicks);

            tx_thread_terminate(&thread_);
            tx_thread_delete(&thread_);
            DrainPendingFrames();
            tx_queue_delete(&queue_);
            started_ = false;
        }

    private:
        static int WriteFrame(const can_msg *msg, int timeout, void *arg)
        {
            // Outbound frames go straight to the board-specific CAN backend.
            auto *self = static_cast<CanChannelBridge *>(arg);
            return self->port_.Write(*msg, MillisecondsToTicks(timeout)) ? 0 : -1;
        }

        static void ThreadEntry(ULONG arg)
        {
            static_cast<CanChannelBridge *>(reinterpret_cast<void *>(arg))->Run();
        }

        void Run()
        {
            ULONG word = 0;

            // Drain queued hardware frames and inject them into Lely from a
            // normal ThreadX thread context.
            while (running_)
            {
                const auto status = tx_queue_receive(&queue_, &word, kThreadSleepTicks);
                if (status == TX_QUEUE_EMPTY)
                    continue;
                if (status != TX_SUCCESS)
                {
                    log_.Error("ThreadX CAN queue receive failed");
                    continue;
                }

                std::unique_ptr<can_msg> msg(reinterpret_cast<can_msg *>(word));
                if (!msg)
                    continue;

                try
                {
                    channel_.on_read(msg.get());
                }
                catch (...)
                {
                    log_.Error("Unable to inject CAN frame into Lely channel");
                }
            }

            stopped_ = true;
        }

        void DrainPendingFrames()
        {
            ULONG word = 0;
            while (tx_queue_receive(&queue_, &word, TX_NO_WAIT) == TX_SUCCESS)
            {
                delete reinterpret_cast<can_msg *>(word);
            }
        }

        ThreadXCanPort &port_;
        const LogSink &log_;
        std::vector<ULONG> queue_storage_;
        lely::io::UserCanChannel channel_;
        UINT priority_{};
        TX_QUEUE queue_{};
        TX_THREAD thread_{};
        alignas(ULONG) std::array<std::byte, kTransportStackSize> stack_{};
        std::atomic<bool> running_{true};
        std::atomic<bool> stopped_{false};
        bool started_{true};
    };

    Impl(ThreadXCia402Config config,
         ThreadXCanPort &can_port,
         ThreadXCia402Logger *logger)
        : config_(config),
          can_port_(can_port),
          log_(logger),
          ctx_(),
          loop_(),
          timer_(ctx_, loop_.get_executor(), config.timer_thread_priority, log_),
          channel_(ctx_, loop_.get_executor(), can_port, config_, log_),
          master_(timer_.timer(), channel_.channel(), CreateMasterDictionary(config.master_id), config.master_id),
          driver_(master_, config.node_id, config_, [this]()
                  {
                          result_ = 0;
                          ctx_.shutdown(); }, [this](const std::string &message)
                  {
                          result_ = 1;
                          last_error_ = message;
                          log_.Error(message);
                          ctx_.shutdown(); }, log_)
    {
        channel_.Start();
    }

    int Run()
    {
        // Startup is small on purpose: validate input, reset the master,
        // launch the driver and let the event loop run until completion.
        ThrowIf(config_.node_id == 0 || config_.node_id > 127, "node-id must be in the range 1..127");

        if (can_port_.IsSimulated())
            return RunSimulated();

        master_.Reset();
        driver_.Start();
        loop_.run();

        if (result_ != 0 && !last_error_.empty())
            log_.Error(last_error_);

        return result_;
    }

    void RequestStop()
    {
        // External callers can request a cooperative shutdown of the sample.
        result_ = 1;
        last_error_ = "stopped by caller";
        ctx_.shutdown();
    }

private:
    int RunSimulated()
    {
        log_.Info(std::string{"Starting "} + kPumpName + " pump sample (simulated)");
        tx_thread_sleep(MillisecondsToTicks(static_cast<int>(config_.boot_settle_time_ms)));

        log_.Info("Configuring profile velocity mode");
        {
            std::ostringstream oss;
            oss << "Applying accel/decel ramps: " << config_.profile_acceleration << '/' << config_.profile_deceleration;
            log_.Info(oss.str());
        }

        log_.Info("Putting node into NMT operational");
        tx_thread_sleep(MillisecondsToTicks(100));

        log_.Info("Transitioning drive to ready-to-switch-on");
        tx_thread_sleep(MillisecondsToTicks(50));
        log_.Info("Transitioning drive to switched-on");
        tx_thread_sleep(MillisecondsToTicks(50));
        log_.Info("Transitioning drive to operation-enabled");
        tx_thread_sleep(MillisecondsToTicks(50));

        {
            std::ostringstream oss;
            oss << "Commanding target velocity " << config_.target_velocity;
            log_.Info(oss.str());
        }

        {
            std::ostringstream oss;
            oss << "Pump running for " << config_.run_time_ms << " ms";
            log_.Info(oss.str());
        }
        tx_thread_sleep(MillisecondsToTicks(static_cast<int>(config_.run_time_ms)));

        log_.Info("Commanding zero velocity");
        tx_thread_sleep(MillisecondsToTicks(200));
        log_.Info("Returning drive to shutdown state");
        tx_thread_sleep(MillisecondsToTicks(50));
        log_.Info("Pump sample completed");

        return 0;
    }

    ThreadXCanPort &can_port_;
    ThreadXCia402Config config_;
    LogSink log_;
    lely::io::Context ctx_;
    lely::ev::Loop loop_;
    TimerPump timer_;
    CanChannelBridge channel_;
    lely::canopen::AsyncMaster master_;
    Cia402Driver driver_;
    int result_{1};
    std::string last_error_;
};

ThreadXCia402MotorApp::ThreadXCia402MotorApp(ThreadXCia402Config config,
                                             ThreadXCanPort &can_port,
                                             ThreadXCia402Logger *logger)
    : impl_(std::make_unique<Impl>(config, can_port, logger))
{
}

ThreadXCia402MotorApp::~ThreadXCia402MotorApp() = default;

ThreadXCia402MotorApp::ThreadXCia402MotorApp(ThreadXCia402MotorApp &&) noexcept = default;

ThreadXCia402MotorApp &ThreadXCia402MotorApp::operator=(ThreadXCia402MotorApp &&) noexcept = default;

int ThreadXCia402MotorApp::Run()
{
    return impl_->Run();
}

void ThreadXCia402MotorApp::RequestStop()
{
    impl_->RequestStop();
}