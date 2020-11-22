#pragma once

#include "platform/PlatformInterface.h"
#include "platform/PlatformParameters.h"
#include "limb/Limb2DofInterface.h"

namespace platform
{
    class Platform8Dof : public IPlatform
    {
    public:
        Platform8Dof(platform::Platform8DofParameters parameters);

    public:
        bool setSpeed(double speedRotateClockwise, double speedMoveForward) override;

    private:
        platform::Platform8DofParameters parameters;
    };
}
