#pragma once

#include "servo/CalibrationTable.h"
#include "servo/ServoInterface.h"
#include "servo/SignalingChannelInterface.h"
#include "servo/SignalingControllerInterface.h"

#include <ros/ros.h>

#include <catix_messages/CalibrationLimitValue.h>
#include <catix_messages/CalibrationPointValue.h>
#include <catix_messages/ServoState.h>
#include <catix_messages/SignalingChannelState.h>

//------------------------------------------------------------------------

class ServoNode 
{
    public:
        ServoNode();

    private:
        void listenerServoState(const catix_messages::ServoStateConstPtr &rServoState);
        void listenerSignalingChannelState(const catix_messages::SignalingChannelStateConstPtr &signalingChannelState);

        void listenerCalibrationFirstPoint(const catix_messages::CalibrationPointValueConstPtr &rCalibrationAngleValue);
        void listenerCalibrationSecondPoint(const catix_messages::CalibrationPointValueConstPtr &rCalibrationAngleValue);
        void listenerCalibrationLowerLimit(const catix_messages::CalibrationLimitValueConstPtr &rCalibrationLimitValue);
        void listenerCalibrationUpperLimit(const catix_messages::CalibrationLimitValueConstPtr &rCalibrationLimitValue);

    private:
        std::unique_ptr<servo::CalibrationTable> calibrationTable;

        ros::NodeHandle nodeHandle;
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
};
