#pragma once

#include "servo/ServoInterface.h"

#include <memory>

namespace limb
{
    struct LimbSegment
    {
        std::shared_ptr<servo::IServo> jointServo;
        double linkLength;
    };
}
