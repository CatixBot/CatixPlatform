#pragma once

#include "limb/Limb2DofInterface.h"

#include <memory>

namespace platform
{
    struct Platform8DofParameters
    {
        std::shared_ptr<limb::ILimb2Dof> frontLeftLeg;
        std::shared_ptr<limb::ILimb2Dof> frontRightLeg;
        std::shared_ptr<limb::ILimb2Dof> rearLeftLeg;
        std::shared_ptr<limb::ILimb2Dof> rearRightLeg;            
    };
}