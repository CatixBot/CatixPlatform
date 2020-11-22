#pragma once

#include "servo/ServoInterface.h"
#include "limb/Limb2DofInterface.h"
#include "limb/LimbSegment.h"

namespace limb
{
    class Leg2Dof : public ILimb2Dof
    {
    public:
        Leg2Dof(limb::LimbSegment lowerSegment, limb::LimbSegment upperSegment);

    public:
        bool setPosition(geometry::PolarCoordinates polarCoordinates) override;

    private:
        limb::LimbSegment lowerSegment;
        limb::LimbSegment upperSegment;
    };
}
