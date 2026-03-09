#pragma once

#include "canopen_cia402_motor_threadx.hpp"

#include <array>
#include <cstdint>

class SimulatedCanPort final : public ThreadXCanPort
{
public:
    explicit SimulatedCanPort(uint8_t node_id, ThreadXCia402Logger *logger = nullptr);
    ~SimulatedCanPort() override;

    void SetReceiver(ThreadXCanReceiver *receiver) override;
    bool Start() override;
    void Stop() override;
    bool Write(const can_msg &msg, ULONG timeout_ticks) override;

private:
    enum class NmtState : uint8_t
    {
        kBootup = 0x00,
        kStopped = 0x04,
        kOperational = 0x05,
        kPreOperational = 0x7f,
    };

    static constexpr uint32_t kCobNmt = 0x000;
    static constexpr uint32_t kCobSdoRequestBase = 0x600;
    static constexpr uint32_t kCobSdoResponseBase = 0x580;
    static constexpr uint32_t kCobHeartbeatBase = 0x700;

    static void HeartbeatTimerEntry(ULONG arg);

    void OnHeartbeatTimer();
    bool EnqueueRx(const can_msg &msg);

    void ResetDeviceLocked();
    void AdvanceSimulationLocked();
    void ApplyControlwordLocked(uint16_t controlword);
    uint16_t ComposeStatuswordLocked() const;
    bool RestartHeartbeatTimerLocked();

    bool HandleNmt(const can_msg &msg);
    bool HandleSdo(const can_msg &msg);
    static ULONG TicksFromMs(uint16_t ms);

    void LogInfo(const char *message) const;
    void LogError(const char *message) const;

    static uint16_t ReadU16Le(const can_msg &msg);
    static uint32_t ReadU32Le(const can_msg &msg);
    static int32_t ReadI32Le(const can_msg &msg);

    can_msg MakeBootup() const;
    can_msg MakeHeartbeat(uint8_t state) const;
    can_msg MakeSdoAbort(uint16_t idx, uint8_t subidx, uint32_t abort_code) const;
    can_msg MakeSdoDownloadResponse(uint16_t idx, uint8_t subidx) const;
    can_msg MakeSdoUploadU8(uint16_t idx, uint8_t subidx, uint8_t value) const;
    can_msg MakeSdoUploadU16(uint16_t idx, uint8_t subidx, uint16_t value) const;
    can_msg MakeSdoUploadU32(uint16_t idx, uint8_t subidx, uint32_t value) const;
    can_msg MakeSdoUploadI8(uint16_t idx, uint8_t subidx, int8_t value) const;
    can_msg MakeSdoUploadI32(uint16_t idx, uint8_t subidx, int32_t value) const;

    uint8_t node_id_{};
    ThreadXCia402Logger *logger_{};
    ThreadXCanReceiver *receiver_{};
    TX_MUTEX lock_{};
    TX_TIMER heartbeat_timer_{};
    bool mutex_created_{false};
    bool timer_created_{false};
    bool started_{false};
    NmtState nmt_state_{NmtState::kPreOperational};
    uint16_t controlword_{0x0000u};
    uint16_t statusword_base_{0x0040u};
    int8_t mode_of_operation_{0};
    int8_t mode_display_{0};
    int32_t target_velocity_{0};
    int32_t actual_velocity_{0};
    uint32_t profile_acceleration_{2000u};
    uint32_t profile_deceleration_{2000u};
    uint16_t heartbeat_producer_time_ms_{100u};
    ULONG last_update_tick_{0};
};