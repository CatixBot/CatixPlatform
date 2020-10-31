#include "servo/SignalingChannelShared.h"

#include <ros/ros.h>

servo::SignalingChannelShared::SignalingChannelShared(uint8_t signalingChannelIndex, 
    std::function<bool(double)> signalingChannelAccessor)
    : signalingChannelIndex(signalingChannelIndex)
    , signalingChannelAccessor(signalingChannelAccessor)
{
}

bool servo::SignalingChannelShared::setStrength(double strengthInPercentage)
{
    if (!this->signalingChannelAccessor)
    {
        ROS_ERROR("Signaling channel %d: can't get access to channel", signalingChannelIndex);
        return false;
    }

    if (!this->signalingChannelAccessor(strengthInPercentage))
    {
        ROS_ERROR("Signaling channel %d: setting channel strength failed", signalingChannelIndex);
        return false;
    }

    return true;
}
