#include "Leg2Dof.h"

#include <cmath>

//------------------------------------------------------------------------

namespace geometry
{
    inline double calcSquare(double value)
    {
        return value*value;
    }

    double calcFirstJointAngle(double firstLinkLength, double secondLinkLength,
        PolarCoordinates targetCoordinates)
    {
        /*
         * phi_1 = phi_t - arccos((l_2^2 - l_1^2 - p_t^2)/(2*l_1*p_t))
         */
        return targetCoordinates.angularCoordinate - std::acos((calcSquare(secondLinkLength) - 
            calcSquare(firstLinkLength) - calcSquare(targetCoordinates.radialCoordinate)) / 
            (2 * firstLinkLength * targetCoordinates.radialCoordinate));
    }

    double calcSecondJointAngle(double firstLinkLength, double secondLinkLength,
        PolarCoordinates targetCoordinates)
    {
        /*
         * phi_1 = arccos((p_t^2 + l_1^2 - l_2^2)/(2*l_1*l_2))
         */
        return std::acos((calcSquare(targetCoordinates.radialCoordinate) - 
            calcSquare(firstLinkLength) - calcSquare(secondLinkLength)) / 
            (2 * firstLinkLength * secondLinkLength));
    }
}

//------------------------------------------------------------------------

limb::Leg2Dof::Leg2Dof(limb::LimbSegment lowerSegment, limb::LimbSegment upperSegment)
    : lowerSegment(lowerSegment)
    , upperSegment(upperSegment)
{
}

bool limb::Leg2Dof::setPosition(geometry::PolarCoordinates targetCoordinates)
{
    const double phi1 = calcFirstJointAngle(this->lowerSegment.linkLength, 
        this->upperSegment.linkLength, targetCoordinates);
    this->lowerSegment.jointServo->setAngle(phi1);

    const double phi2 = calcSecondJointAngle(this->lowerSegment.linkLength, 
        this->upperSegment.linkLength, targetCoordinates);
    this->upperSegment.jointServo->setAngle(phi2);
}
