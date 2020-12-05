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
    float signalStrengthPercentage = this->servoParameters.signalStrengthToAngleSlope * angleRadians + 
        servoParameters.signalStrengthOffset;

    if (signalStrengthPercentage > this->servoParameters.signalStrengthMaximum)
    {
        ROS_WARN("Servo %d: provided angle %frad is greater than possible maximum", 
            this->servoIndex, angleRadians);
        signalStrengthPercentage = this->servoParameters.signalStrengthMaximum;
        ROS_INFO("Servo %d: result signal strength percentage decreased to %f%%", 
            this->servoIndex, signalStrengthPercentage);
    }

    if (signalStrengthPercentage < this->servoParameters.signalStrengthMinimum)
    {
        ROS_WARN("Servo %d: provided angle %frad is less than possible minimum", 
            this->servoIndex, angleRadians);
        signalStrengthPercentage = this->servoParameters.signalStrengthMinimum;
        ROS_INFO("Servo %d: result signal strength percentage increased to %f%%", 
            this->servoIndex, signalStrengthPercentage);
    }

    return this->signalingChannel->setStrength(signalStrengthPercentage);
}
