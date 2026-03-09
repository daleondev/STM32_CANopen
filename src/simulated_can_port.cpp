#include "simulated_can_port.hpp"

#include <algorithm>
#include <cstdlib>

namespace
{

    constexpr uint32_t kIdxDeviceType = 0x1000;
    constexpr uint32_t kIdxErrorRegister = 0x1001;
    constexpr uint32_t kIdxHeartbeatProducerTime = 0x1017;
    constexpr uint32_t kIdxIdentity = 0x1018;
    constexpr uint32_t kSdoAbortNoObject = 0x06020000u;
    constexpr uint32_t kSdoAbortNoSubObject = 0x06090011u;
    constexpr uint32_t kSdoAbortTypeMismatch = 0x06070010u;

    constexpr uint32_t kDeviceType = 0x00020192u;
    constexpr uint32_t kVendorId = 0x00000001u;
    constexpr uint32_t kProductCode = 0x00000402u;
    constexpr uint32_t kRevisionNumber = 0x00000001u;
    constexpr uint32_t kSerialNumber = 0x00000001u;

    constexpr uint16_t kIdxControlword = 0x6040;
    constexpr uint16_t kIdxStatusword = 0x6041;
    constexpr uint16_t kIdxModesOfOperation = 0x6060;
    constexpr uint16_t kIdxModesOfOperationDisplay = 0x6061;
    constexpr uint16_t kIdxVelocityActualValue = 0x606c;
    constexpr uint16_t kIdxTargetVelocity = 0x60ff;
    constexpr uint16_t kIdxProfileAcceleration = 0x6083;
    constexpr uint16_t kIdxProfileDeceleration = 0x6084;

    constexpr uint8_t kNmtStart = 0x01;
    constexpr uint8_t kNmtStop = 0x02;
    constexpr uint8_t kNmtEnterPreOperational = 0x80;
    constexpr uint8_t kNmtResetNode = 0x81;
    constexpr uint8_t kNmtResetCommunication = 0x82;

    constexpr ULONG kMinimumStep = 1;

    void WriteU16Le(can_msg &msg, size_t offset, uint16_t value)
    {
        msg.data[offset + 0] = static_cast<uint8_t>(value & 0xffu);
        msg.data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xffu);
    }

    void WriteU32Le(can_msg &msg, size_t offset, uint32_t value)
    {
        msg.data[offset + 0] = static_cast<uint8_t>(value & 0xffu);
        msg.data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xffu);
        msg.data[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xffu);
        msg.data[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xffu);
    }

} // namespace

SimulatedCanPort::SimulatedCanPort(uint8_t node_id, ThreadXCia402Logger *logger)
    : node_id_(node_id), logger_(logger)
{
}

SimulatedCanPort::~SimulatedCanPort()
{
    Stop();
}

void SimulatedCanPort::SetReceiver(ThreadXCanReceiver *receiver)
{
    receiver_ = receiver;
}

bool SimulatedCanPort::Start()
{
    if (started_)
        return true;

    if (!receiver_)
    {
        LogError("Simulated CAN port has no receiver.");
        return false;
    }

    if (!mutex_created_)
    {
        if (tx_mutex_create(&lock_, const_cast<char *>("sim_can_lock"), TX_NO_INHERIT) != TX_SUCCESS)
        {
            LogError("Unable to create simulated CAN mutex.");
            return false;
        }
        mutex_created_ = true;
    }

    if (!timer_created_)
    {
        if (tx_timer_create(&heartbeat_timer_,
                            const_cast<char *>("sim_can_hb"),
                            &SimulatedCanPort::HeartbeatTimerEntry,
                            reinterpret_cast<ULONG>(this),
                            1u,
                            1u,
                            TX_NO_ACTIVATE) != TX_SUCCESS)
        {
            LogError("Unable to create simulated CAN heartbeat timer.");
            Stop();
            return false;
        }
        timer_created_ = true;
    }

    tx_mutex_get(&lock_, TX_WAIT_FOREVER);
    ResetDeviceLocked();
    started_ = true;
    tx_mutex_put(&lock_);

    LogInfo("Using representative simulated CANopen motor node.");
    return true;
}

void SimulatedCanPort::Stop()
{
    started_ = false;

    if (timer_created_)
    {
        tx_timer_deactivate(&heartbeat_timer_);
        tx_timer_delete(&heartbeat_timer_);
        timer_created_ = false;
    }

    if (mutex_created_)
    {
        tx_mutex_delete(&lock_);
        mutex_created_ = false;
    }

    started_ = false;
}

bool SimulatedCanPort::Write(const can_msg &msg, ULONG timeout_ticks)
{
    (void)timeout_ticks;

    if (!started_)
    {
        LogError("Simulated CAN port is not started.");
        return false;
    }

    tx_mutex_get(&lock_, TX_WAIT_FOREVER);
    AdvanceSimulationLocked();
    tx_mutex_put(&lock_);

    if (msg.id == kCobNmt)
        return HandleNmt(msg);

    if (msg.id == kCobSdoRequestBase + node_id_)
        return HandleSdo(msg);

    return true;
}

void SimulatedCanPort::HeartbeatTimerEntry(ULONG arg)
{
    static_cast<SimulatedCanPort *>(reinterpret_cast<void *>(arg))->OnHeartbeatTimer();
}

void SimulatedCanPort::OnHeartbeatTimer()
{
    tx_mutex_get(&lock_, TX_WAIT_FOREVER);
    if (!started_)
    {
        tx_mutex_put(&lock_);
        return;
    }

    AdvanceSimulationLocked();
    const can_msg msg = MakeHeartbeat(static_cast<uint8_t>(nmt_state_));
    tx_mutex_put(&lock_);
    EnqueueRx(msg);
}

bool SimulatedCanPort::EnqueueRx(const can_msg &msg)
{
    return receiver_ && receiver_->OnCanMessage(msg);
}

void SimulatedCanPort::ResetDeviceLocked()
{
    nmt_state_ = NmtState::kPreOperational;
    controlword_ = 0x0000u;
    statusword_base_ = 0x0040u;
    mode_of_operation_ = 0;
    mode_display_ = 0;
    target_velocity_ = 0;
    actual_velocity_ = 0;
    profile_acceleration_ = 2000u;
    profile_deceleration_ = 2000u;
    heartbeat_producer_time_ms_ = 0u;
    last_update_tick_ = tx_time_get();
}

void SimulatedCanPort::AdvanceSimulationLocked()
{
    const ULONG now = tx_time_get();
    const ULONG elapsed_ticks = now - last_update_tick_;
    last_update_tick_ = now;

    if (elapsed_ticks == 0)
        return;

    const bool operational = nmt_state_ == NmtState::kOperational;
    const bool enabled = (statusword_base_ & 0x006fu) == 0x0027u;
    const bool velocity_mode = mode_of_operation_ == 3;
    const int32_t desired_velocity = (operational && enabled && velocity_mode) ? target_velocity_ : 0;

    if (actual_velocity_ == desired_velocity)
        return;

    const uint32_t ramp_per_second = (std::abs(desired_velocity) > std::abs(actual_velocity_))
                                         ? profile_acceleration_
                                         : profile_deceleration_;
    const ULONG ramp_per_tick = std::max<ULONG>(
        kMinimumStep,
        static_cast<ULONG>((static_cast<unsigned long long>(std::max<uint32_t>(ramp_per_second, 1u)) * elapsed_ticks +
                            TX_TIMER_TICKS_PER_SECOND - 1u) /
                           TX_TIMER_TICKS_PER_SECOND));

    if (desired_velocity > actual_velocity_)
    {
        actual_velocity_ = std::min<int32_t>(desired_velocity, actual_velocity_ + static_cast<int32_t>(ramp_per_tick));
    }
    else
    {
        actual_velocity_ = std::max<int32_t>(desired_velocity, actual_velocity_ - static_cast<int32_t>(ramp_per_tick));
    }
}

bool SimulatedCanPort::HandleNmt(const can_msg &msg)
{
    if (msg.len < 2)
        return true;

    const uint8_t command = msg.data[0];
    const uint8_t target = msg.data[1];
    if (target != 0 && target != node_id_)
        return true;

    can_msg response = CAN_MSG_INIT;
    bool have_response = false;

    tx_mutex_get(&lock_, TX_WAIT_FOREVER);
    AdvanceSimulationLocked();

    switch (command)
    {
    case kNmtStart:
        nmt_state_ = NmtState::kOperational;
        response = MakeHeartbeat(static_cast<uint8_t>(nmt_state_));
        have_response = true;
        break;
    case kNmtStop:
        nmt_state_ = NmtState::kStopped;
        target_velocity_ = 0;
        response = MakeHeartbeat(static_cast<uint8_t>(nmt_state_));
        have_response = true;
        break;
    case kNmtEnterPreOperational:
        nmt_state_ = NmtState::kPreOperational;
        target_velocity_ = 0;
        response = MakeHeartbeat(static_cast<uint8_t>(nmt_state_));
        have_response = true;
        break;
    case kNmtResetNode:
    case kNmtResetCommunication:
        ResetDeviceLocked();
        RestartHeartbeatTimerLocked();
        response = MakeBootup();
        have_response = true;
        break;
    default:
        break;
    }

    tx_mutex_put(&lock_);
    return !have_response || EnqueueRx(response);
}

bool SimulatedCanPort::HandleSdo(const can_msg &msg)
{
    if (msg.len < 4)
        return false;

    const uint8_t command = msg.data[0];
    const uint16_t idx = static_cast<uint16_t>(msg.data[1] | (msg.data[2] << 8));
    const uint8_t subidx = msg.data[3];

    can_msg response = CAN_MSG_INIT;

    tx_mutex_get(&lock_, TX_WAIT_FOREVER);
    AdvanceSimulationLocked();

    if (command == 0x40)
    {
        switch (idx)
        {
        case kIdxDeviceType:
            response = (subidx == 0x00u) ? MakeSdoUploadU32(idx, subidx, kDeviceType)
                                         : MakeSdoAbort(idx, subidx, kSdoAbortNoSubObject);
            break;
        case kIdxErrorRegister:
            response = (subidx == 0x00u) ? MakeSdoUploadU8(idx, subidx, 0u)
                                         : MakeSdoAbort(idx, subidx, kSdoAbortNoSubObject);
            break;
        case kIdxHeartbeatProducerTime:
            response = (subidx == 0x00u) ? MakeSdoUploadU16(idx, subidx, heartbeat_producer_time_ms_)
                                         : MakeSdoAbort(idx, subidx, kSdoAbortNoSubObject);
            break;
        case kIdxIdentity:
            switch (subidx)
            {
            case 0x00:
                response = MakeSdoUploadU8(idx, subidx, 4u);
                break;
            case 0x01:
                response = MakeSdoUploadU32(idx, subidx, kVendorId);
                break;
            case 0x02:
                response = MakeSdoUploadU32(idx, subidx, kProductCode);
                break;
            case 0x03:
                response = MakeSdoUploadU32(idx, subidx, kRevisionNumber);
                break;
            case 0x04:
                response = MakeSdoUploadU32(idx, subidx, kSerialNumber);
                break;
            default:
                response = MakeSdoAbort(idx, subidx, kSdoAbortNoSubObject);
                break;
            }
            break;
        case kIdxStatusword:
            response = (subidx == 0x00u) ? MakeSdoUploadU16(idx, subidx, ComposeStatuswordLocked())
                                         : MakeSdoAbort(idx, subidx, kSdoAbortNoSubObject);
            break;
        case kIdxModesOfOperation:
            response = (subidx == 0x00u) ? MakeSdoUploadI8(idx, subidx, mode_of_operation_)
                                         : MakeSdoAbort(idx, subidx, kSdoAbortNoSubObject);
            break;
        case kIdxModesOfOperationDisplay:
            response = (subidx == 0x00u) ? MakeSdoUploadI8(idx, subidx, mode_display_)
                                         : MakeSdoAbort(idx, subidx, kSdoAbortNoSubObject);
            break;
        case kIdxVelocityActualValue:
            response = (subidx == 0x00u) ? MakeSdoUploadI32(idx, subidx, actual_velocity_)
                                         : MakeSdoAbort(idx, subidx, kSdoAbortNoSubObject);
            break;
        case kIdxTargetVelocity:
            response = (subidx == 0x00u) ? MakeSdoUploadI32(idx, subidx, target_velocity_)
                                         : MakeSdoAbort(idx, subidx, kSdoAbortNoSubObject);
            break;
        case kIdxProfileAcceleration:
            response = (subidx == 0x00u) ? MakeSdoUploadU32(idx, subidx, profile_acceleration_)
                                         : MakeSdoAbort(idx, subidx, kSdoAbortNoSubObject);
            break;
        case kIdxProfileDeceleration:
            response = (subidx == 0x00u) ? MakeSdoUploadU32(idx, subidx, profile_deceleration_)
                                         : MakeSdoAbort(idx, subidx, kSdoAbortNoSubObject);
            break;
        default:
            response = MakeSdoAbort(idx, subidx, kSdoAbortNoObject);
            break;
        }

        tx_mutex_put(&lock_);
        return EnqueueRx(response);
    }

    if ((command & 0xe0u) != 0x20u || (command & 0x02u) == 0)
    {
        tx_mutex_put(&lock_);
        return EnqueueRx(MakeSdoAbort(idx, subidx, kSdoAbortTypeMismatch));
    }

    const uint8_t bytes = static_cast<uint8_t>(4u - ((command >> 2) & 0x03u));

    switch (idx)
    {
    case kIdxHeartbeatProducerTime:
        if (subidx != 0x00u || bytes != 2u)
        {
            response = MakeSdoAbort(idx, subidx, kSdoAbortTypeMismatch);
        }
        else
        {
            heartbeat_producer_time_ms_ = ReadU16Le(msg);
            RestartHeartbeatTimerLocked();
            response = MakeSdoDownloadResponse(idx, subidx);
        }
        break;
    case kIdxControlword:
        if (subidx != 0x00u || bytes != 2u)
        {
            response = MakeSdoAbort(idx, subidx, kSdoAbortTypeMismatch);
        }
        else
        {
            ApplyControlwordLocked(ReadU16Le(msg));
            response = MakeSdoDownloadResponse(idx, subidx);
        }
        break;

    case kIdxModesOfOperation:
        if (subidx != 0x00u || bytes != 1u)
        {
            response = MakeSdoAbort(idx, subidx, kSdoAbortTypeMismatch);
        }
        else
        {
            mode_of_operation_ = static_cast<int8_t>(msg.data[4]);
            mode_display_ = mode_of_operation_;
            response = MakeSdoDownloadResponse(idx, subidx);
        }
        break;

    case kIdxTargetVelocity:
        if (subidx != 0x00u || bytes != 4u)
        {
            response = MakeSdoAbort(idx, subidx, kSdoAbortTypeMismatch);
        }
        else
        {
            target_velocity_ = ReadI32Le(msg);
            response = MakeSdoDownloadResponse(idx, subidx);
        }
        break;

    case kIdxProfileAcceleration:
        if (subidx != 0x00u || bytes != 4u)
        {
            response = MakeSdoAbort(idx, subidx, kSdoAbortTypeMismatch);
        }
        else
        {
            profile_acceleration_ = std::max<uint32_t>(ReadU32Le(msg), 1u);
            response = MakeSdoDownloadResponse(idx, subidx);
        }
        break;

    case kIdxProfileDeceleration:
        if (subidx != 0x00u || bytes != 4u)
        {
            response = MakeSdoAbort(idx, subidx, kSdoAbortTypeMismatch);
        }
        else
        {
            profile_deceleration_ = std::max<uint32_t>(ReadU32Le(msg), 1u);
            response = MakeSdoDownloadResponse(idx, subidx);
        }
        break;

    default:
        response = MakeSdoAbort(idx, subidx, kSdoAbortNoObject);
        break;
    }

    tx_mutex_put(&lock_);
    return EnqueueRx(response);
}

void SimulatedCanPort::ApplyControlwordLocked(uint16_t controlword)
{
    controlword_ = controlword;

    if ((controlword_ & 0x0080u) != 0u)
    {
        statusword_base_ = 0x0040u;
        target_velocity_ = 0;
        return;
    }

    if ((controlword_ & 0x008fu) == 0x000fu)
    {
        statusword_base_ = 0x0027u;
        return;
    }

    if ((controlword_ & 0x008fu) == 0x0007u)
    {
        statusword_base_ = 0x0023u;
        return;
    }

    if ((controlword_ & 0x0087u) == 0x0006u)
    {
        statusword_base_ = 0x0021u;
        return;
    }

    if ((controlword_ & 0x0086u) == 0x0000u)
    {
        statusword_base_ = 0x0040u;
        target_velocity_ = 0;
    }
}

uint16_t SimulatedCanPort::ComposeStatuswordLocked() const
{
    uint16_t statusword = static_cast<uint16_t>(statusword_base_ | 0x0200u);

    const bool enabled = (statusword_base_ & 0x006fu) == 0x0027u;
    const bool operational = nmt_state_ == NmtState::kOperational;
    const bool velocity_mode = mode_of_operation_ == 3;
    const int32_t desired_velocity = (enabled && operational && velocity_mode) ? target_velocity_ : 0;
    if (actual_velocity_ == desired_velocity)
        statusword = static_cast<uint16_t>(statusword | 0x0400u);

    return statusword;
}

bool SimulatedCanPort::RestartHeartbeatTimerLocked()
{
    if (!timer_created_)
        return true;

    tx_timer_deactivate(&heartbeat_timer_);
    if (heartbeat_producer_time_ms_ == 0u)
        return true;

    const ULONG ticks = TicksFromMs(heartbeat_producer_time_ms_);
    if (tx_timer_change(&heartbeat_timer_, ticks, ticks) != TX_SUCCESS)
        return false;

    return tx_timer_activate(&heartbeat_timer_) == TX_SUCCESS;
}

ULONG SimulatedCanPort::TicksFromMs(uint16_t ms)
{
    if (ms == 0u)
        return 0u;

    const auto ticks = (static_cast<unsigned long long>(ms) * TX_TIMER_TICKS_PER_SECOND + 999ull) / 1000ull;
    return static_cast<ULONG>(std::max<unsigned long long>(1ull, ticks));
}

void SimulatedCanPort::LogInfo(const char *message) const
{
    if (logger_)
        logger_->Info(message);
}

void SimulatedCanPort::LogError(const char *message) const
{
    if (logger_)
        logger_->Error(message);
}

uint16_t SimulatedCanPort::ReadU16Le(const can_msg &msg)
{
    return static_cast<uint16_t>(msg.data[4] | (msg.data[5] << 8));
}

uint32_t SimulatedCanPort::ReadU32Le(const can_msg &msg)
{
    return static_cast<uint32_t>(msg.data[4]) |
           (static_cast<uint32_t>(msg.data[5]) << 8) |
           (static_cast<uint32_t>(msg.data[6]) << 16) |
           (static_cast<uint32_t>(msg.data[7]) << 24);
}

int32_t SimulatedCanPort::ReadI32Le(const can_msg &msg)
{
    return static_cast<int32_t>(ReadU32Le(msg));
}

can_msg SimulatedCanPort::MakeBootup() const
{
    can_msg msg = CAN_MSG_INIT;
    msg.id = kCobHeartbeatBase + node_id_;
    msg.len = 1;
    msg.data[0] = static_cast<uint8_t>(NmtState::kBootup);
    return msg;
}

can_msg SimulatedCanPort::MakeHeartbeat(uint8_t state) const
{
    can_msg msg = CAN_MSG_INIT;
    msg.id = kCobHeartbeatBase + node_id_;
    msg.len = 1;
    msg.data[0] = state;
    return msg;
}

can_msg SimulatedCanPort::MakeSdoAbort(uint16_t idx, uint8_t subidx, uint32_t abort_code) const
{
    can_msg msg = CAN_MSG_INIT;
    msg.id = kCobSdoResponseBase + node_id_;
    msg.len = 8;
    msg.data[0] = 0x80;
    WriteU16Le(msg, 1, idx);
    msg.data[3] = subidx;
    WriteU32Le(msg, 4, abort_code);
    return msg;
}

can_msg SimulatedCanPort::MakeSdoDownloadResponse(uint16_t idx, uint8_t subidx) const
{
    can_msg msg = CAN_MSG_INIT;
    msg.id = kCobSdoResponseBase + node_id_;
    msg.len = 8;
    msg.data[0] = 0x60;
    WriteU16Le(msg, 1, idx);
    msg.data[3] = subidx;
    return msg;
}

can_msg SimulatedCanPort::MakeSdoUploadU8(uint16_t idx, uint8_t subidx, uint8_t value) const
{
    can_msg msg = CAN_MSG_INIT;
    msg.id = kCobSdoResponseBase + node_id_;
    msg.len = 8;
    msg.data[0] = 0x4f;
    WriteU16Le(msg, 1, idx);
    msg.data[3] = subidx;
    msg.data[4] = value;
    return msg;
}

can_msg SimulatedCanPort::MakeSdoUploadU16(uint16_t idx, uint8_t subidx, uint16_t value) const
{
    can_msg msg = CAN_MSG_INIT;
    msg.id = kCobSdoResponseBase + node_id_;
    msg.len = 8;
    msg.data[0] = 0x4b;
    WriteU16Le(msg, 1, idx);
    msg.data[3] = subidx;
    WriteU16Le(msg, 4, value);
    return msg;
}

can_msg SimulatedCanPort::MakeSdoUploadU32(uint16_t idx, uint8_t subidx, uint32_t value) const
{
    can_msg msg = CAN_MSG_INIT;
    msg.id = kCobSdoResponseBase + node_id_;
    msg.len = 8;
    msg.data[0] = 0x43;
    WriteU16Le(msg, 1, idx);
    msg.data[3] = subidx;
    WriteU32Le(msg, 4, value);
    return msg;
}

can_msg SimulatedCanPort::MakeSdoUploadI8(uint16_t idx, uint8_t subidx, int8_t value) const
{
    return MakeSdoUploadU8(idx, subidx, static_cast<uint8_t>(value));
}

can_msg SimulatedCanPort::MakeSdoUploadI32(uint16_t idx, uint8_t subidx, int32_t value) const
{
    return MakeSdoUploadU32(idx, subidx, static_cast<uint32_t>(value));
}