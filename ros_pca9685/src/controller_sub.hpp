#pragma once

#include <PCA9685.h>
#include <ros/ros.h>
#include <catix_messages/EightDofPlatformState.h>
#include <catix_messages/TwoDofLegState.h>
#include <catix_messages/ServoState.h>
#include <catix_messages/PwmState.h>

//------------------------------------------------------------------------

class CatixPlatform 
{
    public:
        CatixPlatform();

    private:
        struct ServoParameters
        {
            ServoParameters()
                : pulseWidthOffset(0.0)
                , pulseWidthToAngleSlope(0.0)
                , pulseWidthMinimum(0.0)
                , pulseWidthMaximum(0.0)
            {
            }

            double pulseWidthOffset;
            double pulseWidthToAngleSlope;
            double pulseWidthMinimum;
            double pulseWidthMaximum;
        };

    private:
        using servoparameters_t = std::vector<ServoParameters>;

    private:
        void listenerPlatformState(const catix_messages::EightDofPlatformStateConstPtr &rPlatformState);
        void listenerLegState(const catix_messages::TwoDofLegStateConstPtr &rLegState);
        void listenerServoState(const catix_messages::ServoStateConstPtr &rServoState);
        void listenerPwmState(const catix_messages::PwmStateConstPtr &rPWMState);

    private:
        servoparameters_t getDefaultParameters();
        void setPulseWidth(uint8_t channelNumber, float pulseWidthPercentage);
        float convertAngleToPulseWidth(float angle, uint8_t channelNumber);

    private:
        std::unique_ptr<PCA9685> pControllerPWM;
        servoparameters_t servoParameters;

        ros::NodeHandle node;
        ros::Subscriber subscriberPlatform;
        ros::Subscriber subscriberLeg;
        ros::Subscriber subscriberServo;
        ros::Subscriber subscriberPWM;
};
