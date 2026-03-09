#include "simulated_can_port.hpp"

#include <algorithm>
#include <cstdlib>

namespace
{

    constexpr uint32_t kSdoAbortNoObject = 0x06020000u;
    constexpr uint32_t kSdoAbortNoSubObject = 0x06090011u;
    constexpr uint32_t kSdoAbortTypeMismatch = 0x06070010u;

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
    ResetDevice();
}

void SimulatedCanPort::SetReceiver(ThreadXCanReceiver *receiver)
{
    receiver_ = receiver;
}

bool SimulatedCanPort::Start()
{
    if (!receiver_)
    {
        LogError("Simulated CAN port has no receiver.");
        return false;
    }

    ResetDevice();
    started_ = true;
    LogInfo("Using simulated CANopen motor node.");
    return true;
}

void SimulatedCanPort::Stop()
{
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

    AdvanceSimulation();

    if (msg.id == kCobNmt)
        return HandleNmt(msg);

    if (msg.id == kCobSdoRequestBase + node_id_)
        return HandleSdo(msg);

    return true;
}

void SimulatedCanPort::ResetDevice()
{
    nmt_state_ = NmtState::kPreOperational;
    statusword_ = 0x0021u;
    controlword_ = 0x0000u;
    mode_of_operation_ = 0;
    mode_display_ = 0;
    target_velocity_ = 0;
    actual_velocity_ = 0;
    profile_acceleration_ = 2000u;
    profile_deceleration_ = 2000u;
    last_update_tick_ = tx_time_get();
}

void SimulatedCanPort::AdvanceSimulation()
{
    const ULONG now = tx_time_get();
    const ULONG elapsed_ticks = now - last_update_tick_;
    last_update_tick_ = now;

    if (elapsed_ticks == 0)
        return;

    const bool operational = nmt_state_ == NmtState::kOperational;
    const bool enabled = (statusword_ & 0x006fu) == 0x0027u;
    const int32_t desired_velocity = (operational && enabled) ? target_velocity_ : 0;

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

    switch (command)
    {
    case kNmtStart:
        nmt_state_ = NmtState::kOperational;
        return SendHeartbeat(static_cast<uint8_t>(nmt_state_));
    case kNmtStop:
        nmt_state_ = NmtState::kStopped;
        return SendHeartbeat(static_cast<uint8_t>(nmt_state_));
    case kNmtEnterPreOperational:
        nmt_state_ = NmtState::kPreOperational;
        return SendHeartbeat(static_cast<uint8_t>(nmt_state_));
    case kNmtResetNode:
    case kNmtResetCommunication:
        ResetDevice();
        return SendBootup();
    default:
        return true;
    }
}

bool SimulatedCanPort::HandleSdo(const can_msg &msg)
{
    if (msg.len < 4)
        return false;

    const uint8_t command = msg.data[0];
    const uint16_t idx = static_cast<uint16_t>(msg.data[1] | (msg.data[2] << 8));
    const uint8_t subidx = msg.data[3];

    if (subidx != 0x00)
        return Notify(MakeSdoAbort(idx, subidx, kSdoAbortNoSubObject));

    if (command == 0x40)
    {
        AdvanceSimulation();
        switch (idx)
        {
        case kIdxStatusword:
            return Notify(MakeSdoUploadU16(idx, subidx, statusword_));
        case kIdxModesOfOperationDisplay:
            return Notify(MakeSdoUploadI8(idx, subidx, mode_display_));
        case kIdxVelocityActualValue:
            return Notify(MakeSdoUploadI32(idx, subidx, actual_velocity_));
        default:
            return Notify(MakeSdoAbort(idx, subidx, kSdoAbortNoObject));
        }
    }

    if ((command & 0xe0u) != 0x20u || (command & 0x02u) == 0)
        return Notify(MakeSdoAbort(idx, subidx, kSdoAbortTypeMismatch));

    const uint8_t bytes = static_cast<uint8_t>(4u - ((command >> 2) & 0x03u));

    switch (idx)
    {
    case kIdxControlword:
        if (bytes != 2u)
            return Notify(MakeSdoAbort(idx, subidx, kSdoAbortTypeMismatch));
        controlword_ = ReadU16Le(msg);
        switch (controlword_)
        {
        case 0x0080u:
            statusword_ &= static_cast<uint16_t>(~0x0008u);
            statusword_ = 0x0021u;
            break;
        case 0x0006u:
            statusword_ = 0x0021u;
            break;
        case 0x0007u:
            statusword_ = 0x0023u;
            break;
        case 0x000fu:
            statusword_ = 0x0027u;
            break;
        default:
            break;
        }
        return Notify(MakeSdoDownloadResponse(idx, subidx));

    case kIdxModesOfOperation:
        if (bytes != 1u)
            return Notify(MakeSdoAbort(idx, subidx, kSdoAbortTypeMismatch));
        mode_of_operation_ = static_cast<int8_t>(msg.data[4]);
        mode_display_ = mode_of_operation_;
        return Notify(MakeSdoDownloadResponse(idx, subidx));

    case kIdxTargetVelocity:
        if (bytes != 4u)
            return Notify(MakeSdoAbort(idx, subidx, kSdoAbortTypeMismatch));
        target_velocity_ = ReadI32Le(msg);
        return Notify(MakeSdoDownloadResponse(idx, subidx));

    case kIdxProfileAcceleration:
        if (bytes != 4u)
            return Notify(MakeSdoAbort(idx, subidx, kSdoAbortTypeMismatch));
        profile_acceleration_ = ReadU32Le(msg);
        return Notify(MakeSdoDownloadResponse(idx, subidx));

    case kIdxProfileDeceleration:
        if (bytes != 4u)
            return Notify(MakeSdoAbort(idx, subidx, kSdoAbortTypeMismatch));
        profile_deceleration_ = ReadU32Le(msg);
        return Notify(MakeSdoDownloadResponse(idx, subidx));

    default:
        return Notify(MakeSdoAbort(idx, subidx, kSdoAbortNoObject));
    }
}

bool SimulatedCanPort::SendBootup()
{
    can_msg msg = CAN_MSG_INIT;
    msg.id = kCobHeartbeatBase + node_id_;
    msg.len = 1;
    msg.data[0] = static_cast<uint8_t>(NmtState::kBootup);

    const bool delivered = Notify(msg);
    nmt_state_ = NmtState::kPreOperational;
    return delivered && SendHeartbeat(static_cast<uint8_t>(nmt_state_));
}

bool SimulatedCanPort::SendHeartbeat(uint8_t state)
{
    can_msg msg = CAN_MSG_INIT;
    msg.id = kCobHeartbeatBase + node_id_;
    msg.len = 1;
    msg.data[0] = state;
    return Notify(msg);
}

bool SimulatedCanPort::Notify(const can_msg &msg) const
{
    return receiver_ && receiver_->OnCanMessage(msg);
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

can_msg SimulatedCanPort::MakeSdoUploadI8(uint16_t idx, uint8_t subidx, int8_t value) const
{
    can_msg msg = CAN_MSG_INIT;
    msg.id = kCobSdoResponseBase + node_id_;
    msg.len = 8;
    msg.data[0] = 0x4f;
    WriteU16Le(msg, 1, idx);
    msg.data[3] = subidx;
    msg.data[4] = static_cast<uint8_t>(value);
    return msg;
}

can_msg SimulatedCanPort::MakeSdoUploadI32(uint16_t idx, uint8_t subidx, int32_t value) const
{
    can_msg msg = CAN_MSG_INIT;
    msg.id = kCobSdoResponseBase + node_id_;
    msg.len = 8;
    msg.data[0] = 0x43;
    WriteU16Le(msg, 1, idx);
    msg.data[3] = subidx;
    WriteU32Le(msg, 4, static_cast<uint32_t>(value));
    return msg;
}