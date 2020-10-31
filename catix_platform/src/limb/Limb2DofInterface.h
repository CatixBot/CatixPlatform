#pragma once

#include "PolarCoordinates.h"

namespace limb
{
    /*
     * \brief Limb consists of 2 joints and 2 links
     */
    class ILimb2Dof
    {
    public:
        virtual ~ILimb2Dof() = default;

    public:
        /*
         * \brief Initiate moving of a limb to position provided in polar coordinates
         * \note Method is non-blocking
         */
        virtual bool setPosition(geometry::PolarCoordinates targetCoordinates) = 0;
    };
}
