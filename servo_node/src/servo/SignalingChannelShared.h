#pragma once

#include "servo/SignalingChannelInterface.h"

#include <functional>

namespace servo
{
    class SignalingChannelShared : public ISignalingChannel
    {
    public:
        SignalingChannelShared(uint8_t signalingChannelIndex, 
            std::function<bool(double)> signalingChannelAccessor);

    public:
        bool setStrength(double strengthInPercentage) override;

    private:
        uint8_t signalingChannelIndex;
        std::function<bool(double)> signalingChannelAccessor;
    };
}
