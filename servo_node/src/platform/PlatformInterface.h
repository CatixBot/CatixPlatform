#pragma once

namespace platform
{
    /*
     * \brief Platform based on horizontal plane that can be moved and rotated 
     */
    class IPlatform
    {
    public:
        virtual ~IPlatform() = default;

    public:
        /*
         * \brief Change the speed of platform in horizontal plane
         * \note Method is blocking
         */
        virtual bool setSpeed(double speedRotateClockwise, double speedMoveForward) = 0;
    };
}
