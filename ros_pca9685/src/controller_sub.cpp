#include "controller_sub.hpp"

#include "PCA9685.h"

#include <cmath>

//---------------------------------------------------------------------------

#define I2C_BUS 1
#define I2C_ADDRESS 0x40
#define SERVO_PULSE_RANGE 4096
#define NUMBER_OF_CHANNELS 16

//---------------------------------------------------------------------------

CatixPlatform::CatixPlatform()
    : pControllerPWM(std::make_unique<PCA9685>(I2C_BUS, I2C_ADDRESS))
    , servoParameters(getDefaultParameters())
{
    this->pControllerPWM->setPWMFreq(60);

    sub_pwm = node.subscribe("CatixPlatform/pwm", 100, &CatixPlatform::chatterPwmState, this);
    ROS_INFO("PWM control is ready...");

    sub_servo = node.subscribe("CatixPlatform/servo", 100, &CatixPlatform::chatterServoState, this);
    ROS_INFO("Servo control is ready...");

    sub_servo = node.subscribe("CatixPlatform/2dofleg", 100, &CatixPlatform::chatterServoState, this);
    ROS_INFO("2DOF leg control is ready...");

    sub_servo = node.subscribe("CatixPlatform/8dofplatform", 100, &CatixPlatform::chatterServoState, this);
    ROS_INFO("8DOF platform control is ready...");
}

void CatixPlatform::listenerPwmState(const CatixMessages::PwmStateConstPtr &pPwmState)
{
    this->setPulseWidth(pPwmState->channel_number, pPwmState->pulse_width_percentage);
    ROS_INFO("PWM %d: [%f%%]", pPwmState->channel_number, pPwmState->pulse_width_percentage);
}

void CatixPlatform::listenerServoState(const CatixMessages::ServoStateConstPtr &pServoState)
{
    const float pulseWidthPercentage = convertAngleToPulseWidth(pServoState->rotate_angle);
    this->setPulseWidth(pServoState->channel_number, pulseWidthPercentage);
    ROS_INFO("Servo %d: [%frad]",  pServoState->channel_number, pServoState->rotate_angle);
}

void CatixPlatform::listenerLegState(const CatixMessages::2DofLegStateConstPtr &pLegState)
{
    // TODO: Calculate angles to rotate links according to the required position
}

void CatixPlatform::listenerPlatformState(const CatixMessages::8DofPlatformStateConstPtr &pPlatformState)
{
    // TODO: Calculate and run trajectory for each leg to move/rotate accordingly
}

servoparameters_t CatixPlatform::getDefaultParameters()
{
    const ServoParameters defaultParameters;
    defaultParameters.pulseWidthOffset = 0.0;
    defaultParameters.pulseWidthToAngleSlope = M_PI / 100.0;
    defaultParameters.pulseWidthMinimum = 0.0;
    defaultParameters.pulseWidthMaximum = 100.0;

    return {NUMBER_OF_CHANNELS, defaultParameters};
}

void CatixPlatform::setPulseWidth(uint8_t channelNumber, float pulseWidthPercentage)
{
    const float pulseWidth = pulseWidthPercentage * SERVO_PULSE_RANGE / 100;
    this->pControllerPWM->setPWM(channelNumber, 0, static_cast<int>(pulseWidth));
}

float CatixPlatform::convertAngleToPulseWidth(float angle, uint8_t channelNumber)
{
    const ServoParameters& rServoParameters = this->servoParameters[channelNumber];
    float pulseWidthPercentage = rServoParameters.pulseWidthToAngleSlope * angle + rServoParameters.pulseWidthOffset;
    if (pulseWidthPercentage > rServoParameters.pulseWidthMaximum)
    {
        ROS_WARN("Servo %d: %frad angle is out of range", channelNumber, angle);
        pulseWidthPercentage = rServoParameters.pulseWidthMaximum;
    }

    if (pulseWidthPercentage < rServoParameters.pulseWidthMinimum)
    {
        ROS_WARN("Servo %d: %frad angle is out of range", channelNumber, angle);
        pulseWidthPercentage = rServoParameters.pulseWidthMinimum;
    }

    return pulseWidthPercentage;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CatixPlatform");
    CatixPlatform catixPlatform;
    ros::spin();
}
