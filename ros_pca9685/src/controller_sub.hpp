#pragma once

#include <ros/ros.h>
#include <pca9685_msgs/ServoState.h>
#include <pca9685_msgs/PwmState.h>

//------------------------------------------------------------------------

class PCA9585;

//------------------------------------------------------------------------

class CatixPlatform 
{
    public:
        CatixPlatform();

    private:
        struct ServoParameters
        {
            const double pulseWidthOffset;
            const double pulseWidthToAngleSlope;
            const double pulseWidthMinimum;
            const double pulseWidthMaximum;
        };

    private:
        using servoparameters_t = std::vector<uint8_t, ServoParameters>;

    private:
        void listenerPlatformState(const CatixMessages::8DofPlatformStateConstPtr &rPlatformState);
        void listenerLegState(const CatixMessages::2DofLegStateConstPtr &rLegState);
        void listenerServoState(const CatixMessages::ServoStateConstPtr &rServoState);
        void listenerPwmState(const CatixMessages::PwmStateConstPtr &rPWMState);

    private:
        servoparameters_t getDefaultParameters();
        void setPulseWidth(uint8_t channelNumber, float pulseWidthPercentage);
        float convertAngleToPulseWidth(float angle);

    private:
        std::unique_ptr<PCA9685> pControllerPWM;
        servoparameters_t servoParameters;

        ros::NodeHandle node;
        ros::Subscriber subscriberPlatform;
        ros::Subscriber subscriberLeg;
        ros::Subscriber subscriberServo;
        ros::Subscriber subscriberPWM;
};
