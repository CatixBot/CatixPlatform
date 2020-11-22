#pragma once

#include "servo/CalibrationTable.h"
#include "servo/ServoInterface.h"
#include "servo/SignalingChannelInterface.h"
#include "servo/SignalingControllerInterface.h"
#include "limb/Limb2DofInterface.h"
#include "platform/PlatformInterface.h"

#include <ros/ros.h>

#include <catix_messages/CalibrationLimitValue.h>
#include <catix_messages/CalibrationPointValue.h>
#include <catix_messages/EightDofPlatformState.h>
#include <catix_messages/TwoDofLegState.h>
#include <catix_messages/ServoState.h>
#include <catix_messages/SignalingChannelState.h>

//------------------------------------------------------------------------

class PlatformNode 
{
    public:
        PlatformNode();

    private:
        void listenerPlatformState(const catix_messages::EightDofPlatformStateConstPtr &rPlatformState);
        void listenerLegState(const catix_messages::TwoDofLegStateConstPtr &rLegState);
        void listenerServoState(const catix_messages::ServoStateConstPtr &rServoState);
        void listenerSignalingChannelState(const catix_messages::SignalingChannelStateConstPtr &signalingChannelState);

        void listenerCalibrationFirstPoint(const catix_messages::CalibrationPointValueConstPtr &rCalibrationAngleValue);
        void listenerCalibrationSecondPoint(const catix_messages::CalibrationPointValueConstPtr &rCalibrationAngleValue);
        void listenerCalibrationLowerLimit(const catix_messages::CalibrationLimitValueConstPtr &rCalibrationLimitValue);
        void listenerCalibrationUpperLimit(const catix_messages::CalibrationLimitValueConstPtr &rCalibrationLimitValue);

    private:
        servo::CalibrationTable calibrationTable;

        ros::NodeHandle node;
        ros::Subscriber subscriberPlatform;
        ros::Subscriber subscriberLeg;
        ros::Subscriber subscriberServo;
        ros::Subscriber subscriberSignalingChannel;

        ros::Subscriber subscriberCalibrationFirstPoint;
        ros::Subscriber subscriberCalibrationSecondPoint;
        ros::Subscriber subscriberCalibrationLowerLimit;
        ros::Subscriber subscriberCalibrationUpperLimit;

    private:
        std::unique_ptr<servo::ISignalingController> signalingController;
        
        std::vector<std::shared_ptr<servo::ISignalingChannel>> channels;
        std::vector<std::shared_ptr<servo::IServo>> servos;
        std::vector<std::shared_ptr<limb::ILimb2Dof>> legs;

        std::unique_ptr<platform::IPlatform> platform;
};
