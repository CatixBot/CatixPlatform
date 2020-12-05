#pragma once

#include "servo/ServoInterface.h"
#include "servo/ServoParameters.h"
#include "servo/SignalingChannelInterface.h"

#include <memory>

namespace servo
{
    class ServoMG90S : public IServo
    {
    public:
        ServoMG90S(uint8_t servoIndex, std::shared_ptr<servo::ISignalingChannel> signalingChannel);

    public:
        bool setAngle(double angleRadians) override;
        void setParameters(servo::ServoParameters servoParameters) override;

    private:
        void fixIncorrectLimits();

    private:
        uint8_t servoIndex;
        servo::ServoParameters servoParameters;
        std::shared_ptr<servo::ISignalingChannel> signalingChannel;
    };
}
