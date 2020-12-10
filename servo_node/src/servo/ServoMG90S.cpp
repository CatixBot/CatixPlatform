#include "servo/ServoMG90S.h"

#include <ros/ros.h>

//---------------------------------------------------------------

const double MG90S_POSSIBLE_MINIMUM_LIMIT = 5.0; // Based on MG90S datasheet
const double MG90S_POSSIBLE_MAXIMUM_LIMIT = 20.0; // Based on MG90S datasheet

//---------------------------------------------------------------

servo::ServoMG90S::ServoMG90S(uint8_t servoIndex, std::shared_ptr<servo::ISignalingChannel> signalingChannel)
    : servoIndex(servoIndex)
    , signalingChannel(signalingChannel)
{
    this->servoParameters.signalStrengthMinimum = MG90S_POSSIBLE_MINIMUM_LIMIT;
    this->servoParameters.signalStrengthMaximum = MG90S_POSSIBLE_MAXIMUM_LIMIT;
}

void servo::ServoMG90S::setParameters(servo::ServoParameters servoParameters)
{
    ROS_DEBUG("Servo %d: set parameters: [offset: %f%%, slope: %f%%/rad, min: %f%%, max: %f%%]", this->servoIndex,
        servoParameters.signalStrengthOffset, servoParameters.signalStrengthToAngleSlope, servoParameters.signalStrengthMinimum, servoParameters.signalStrengthMaximum);

    this->servoParameters = servoParameters;
    fixIncorrectLimits();
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

void servo::ServoMG90S::fixIncorrectLimits()
{
    if(this->servoParameters.signalStrengthMinimum < MG90S_POSSIBLE_MINIMUM_LIMIT)
    {
        ROS_WARN("Servo %d: provided lower limit parameter value %f%% is lower than possible for MG90S", 
            this->servoIndex, this->servoParameters.signalStrengthMinimum);
        this->servoParameters.signalStrengthMinimum = MG90S_POSSIBLE_MINIMUM_LIMIT;
        ROS_INFO("Servo %d: result lower limit parameter value increased to %f%%", 
            this->servoIndex, this->servoParameters.signalStrengthMinimum);
    }

    if(this->servoParameters.signalStrengthMaximum > MG90S_POSSIBLE_MAXIMUM_LIMIT)
    {
        ROS_WARN("Servo %d: provided upper limit parameter value %f%% is greater than possible for MG90S", 
            this->servoIndex, this->servoParameters.signalStrengthMaximum);
        this->servoParameters.signalStrengthMaximum = MG90S_POSSIBLE_MAXIMUM_LIMIT;
        ROS_INFO("Servo %d: result upper limit parameter value decreased to %f%%", 
            this->servoIndex, this->servoParameters.signalStrengthMaximum);
    }
}
