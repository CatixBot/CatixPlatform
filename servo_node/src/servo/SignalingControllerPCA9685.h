#pragma once

#include "servo/SignalingControllerInterface.h"

#include <PCA9685.h>
#include <mutex>

namespace servo
{
    class SignalingControllerPCA9685 : public ISignalingController
    {
    public:
        SignalingControllerPCA9685();

    public:
        std::unique_ptr<servo::ISignalingChannel> makeSignalingChannel(uint8_t signalingChannelIndex) override;
        void dropAllChannels() override;

    private:
        std::shared_ptr<PCA9685> pControllerPCA9685;
        std::mutex controllerMutex;
    };
}