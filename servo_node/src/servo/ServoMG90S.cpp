#include "servo/ServoMG90S.h"

#include <ros/ros.h>

servo::ServoMG90S::ServoMG90S(uint8_t servoIndex, std::shared_ptr<servo::ISignalingChannel> signalingChannel)
    : servoIndex(servoIndex)
    , signalingChannel(signalingChannel)
{
}

void servo::ServoMG90S::setParameters(servo::ServoParameters servoParameters)
{
    this->servoParameters = servoParameters;
}

bool servo::ServoMG90S::setAngle(double angleRadians)
{
    float pulseWidthPercentage = this->servoParameters.pulseWidthToAngleSlope * angleRadians + 
        servoParameters.pulseWidthOffset;

    if (pulseWidthPercentage > this->servoParameters.pulseWidthMaximum)
    {
        ROS_WARN("Servo %d: Provided angle %frad is greater than possible maximum", 
            this->servoIndex, angleRadians);
        pulseWidthPercentage = this->servoParameters.pulseWidthMaximum;
        ROS_INFO("Servo %d: Result signal strength percentage decreased to %f%%", 
            this->servoIndex, pulseWidthPercentage);
    }

    if (pulseWidthPercentage < this->servoParameters.pulseWidthMinimum)
    {
        ROS_WARN("Servo %d: Provided angle %frad is less than possible minimum", 
            this->servoIndex, angleRadians);
        pulseWidthPercentage = this->servoParameters.pulseWidthMinimum;
        ROS_INFO("Servo %d: Result signal strength percentage increased to %f%%", 
            this->servoIndex, pulseWidthPercentage);
    }

    return this->signalingChannel->setStrength(pulseWidthPercentage);
}
