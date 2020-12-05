#pragma once

#include <cmath>

//---------------------------------------------------------------------------

static const double SIGNAL_STRENGTH_MINIMUM_DEFAULT = 0.0;
static const double SIGNAL_STRENGTH_MAXIMUM_DEFAULT = 100.0;

//---------------------------------------------------------------------------

namespace servo
{
    struct ServoParameters
    {
        double signalStrengthOffset = 0.0;
        double signalStrengthToAngleSlope = 0.0;
        double signalStrengthMinimum = SIGNAL_STRENGTH_MINIMUM_DEFAULT;
        double signalStrengthMaximum = SIGNAL_STRENGTH_MAXIMUM_DEFAULT;
    };
}
