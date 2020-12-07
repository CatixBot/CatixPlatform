#include "servo/SignalingControllerPCA9685.h"
#include "servo/SignalingChannelShared.h"

#include <ros/ros.h>

//---------------------------------------------------------------------

#define I2C_BUS 1
#define I2C_ADDRESS 0x40
#define SERVO_PULSE_RANGE 4096
#define NUMBER_OF_PWM_CHANNELS 16

//---------------------------------------------------------------------

servo::SignalingControllerPCA9685::SignalingControllerPCA9685()
    : pControllerPCA9685(std::make_unique<PCA9685>(I2C_BUS, I2C_ADDRESS))
{
    this->pControllerPCA9685->setPWMFreq(60);
}

std::unique_ptr<servo::ISignalingChannel> servo::SignalingControllerPCA9685::makeSignalingChannel(uint8_t signalingChannelIndex)
{
    auto signalingChannelAccessor = [pWeakControllerPCA9685 = std::weak_ptr<PCA9685>(this->pControllerPCA9685), signalingChannelIndex](double strengthPercentage)
    { 
        auto pSharedControllerPCA9685 = pWeakControllerPCA9685.lock();
        if (pSharedControllerPCA9685 == nullptr)
        {
            ROS_ERROR("Signaling channel %d: PWM controller is not available", signalingChannelIndex);
            return false;
        }

        size_t controllerPCA9685Index = signalingChannelIndex + 1;
        const int pulseWidth = static_cast<int>(SERVO_PULSE_RANGE * strengthPercentage / 100.0);
        pSharedControllerPCA9685->setPWM(controllerPCA9685Index, pulseWidth);
        return true;
    };

    return std::make_unique<servo::SignalingChannelShared>(signalingChannelIndex, signalingChannelAccessor);
}

void servo::SignalingControllerPCA9685::dropAllChannels()
{
    for (size_t i = 1; i <= NUMBER_OF_PWM_CHANNELS; ++i)
    {
        this->pControllerPCA9685->setPWM(i, 0);
    }
}
