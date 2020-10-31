#pragma once

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
    };
}