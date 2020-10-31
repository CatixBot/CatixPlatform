#pragma once

#include <cmath>

//---------------------------------------------------------------------------

static const double PULSE_WIDTH_POSSIBLE_MINIMUM = 0.0;
static const double PULSE_WIDTH_POSSIBLE_MAXIMUM = 100.0;

static const double ROTATE_ANGLE_LOWER_LIMIT_DEFAULT = - M_PI / 2;
static const double ROTATE_ANGLE_UPPER_LIMIT_DEFAULT = M_PI / 2;

static const double PULSE_WIDTH_TO_ANGLE_SLOPE_DEFAULT = (PULSE_WIDTH_POSSIBLE_MAXIMUM - PULSE_WIDTH_POSSIBLE_MINIMUM) / 
    (ROTATE_ANGLE_UPPER_LIMIT_DEFAULT - ROTATE_ANGLE_LOWER_LIMIT_DEFAULT);
static const double PULSE_WIDTH_OFFSET_DEFAULT = - PULSE_WIDTH_TO_ANGLE_SLOPE_DEFAULT * (PULSE_WIDTH_POSSIBLE_MAXIMUM - PULSE_WIDTH_POSSIBLE_MINIMUM) / 2;

//---------------------------------------------------------------------------

namespace servo
{
    struct ServoParameters
    {
        double pulseWidthOffset = PULSE_WIDTH_OFFSET_DEFAULT;
        double pulseWidthToAngleSlope = PULSE_WIDTH_TO_ANGLE_SLOPE_DEFAULT;
        double pulseWidthMinimum = PULSE_WIDTH_POSSIBLE_MINIMUM;
        double pulseWidthMaximum = PULSE_WIDTH_POSSIBLE_MAXIMUM;
    };
}
