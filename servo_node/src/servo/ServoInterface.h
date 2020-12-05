#pragma once

#include "servo/ServoParameters.h"

namespace servo
{
    /*
     * \brief Servo controlling interface
     */
    class IServo
    {
    public:
        virtual ~IServo() = default;

    public:
        /*
         * \brief Initiate moving to angle in radians
         * \note Method is non-blocking
         */
        virtual bool setAngle(double angleRadians) = 0;

        /*
         * \brief Set parameters used to restore required physical channel state from servo angle
         */
        virtual void setParameters(servo::ServoParameters servoParameters) = 0;
    };
}