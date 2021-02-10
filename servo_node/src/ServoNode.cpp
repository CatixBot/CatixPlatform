#include "ServoNode.h"

#include "servo/SignalingControllerPCA9685.h"
#include "servo/SignalingChannelShared.h"
#include "servo/ServoMG90S.h"

#include <cmath>

//---------------------------------------------------------------------------

#define NUMBER_OF_CHANNELS 8

//---------------------------------------------------------------------------

std::unique_ptr<servo::ISignalingController> makeSignalingController()
{
    return std::make_unique<servo::SignalingControllerPCA9685>();
}

std::vector<std::shared_ptr<servo::ISignalingChannel>> makeSignalingChannels(
    size_t numberOfChannels, servo::ISignalingController& signalingController)
{
    std::vector<std::shared_ptr<servo::ISignalingChannel>> channels;

    for (size_t i = 0; i < numberOfChannels; ++i)
    {
        channels.emplace_back(std::shared_ptr<servo::ISignalingChannel>(
            signalingController.makeSignalingChannel(i)));
    }

    return channels;
}

std::vector<std::shared_ptr<servo::IServo>> makeServos(
    const std::vector<std::shared_ptr<servo::ISignalingChannel>>& signalingChannels)
{
    const size_t EXPECTED_NUMBER_OF_CHANNELS = 8;
    const size_t PROVIDED_NUMBER_OF_CHANNELS = signalingChannels.size();

    if (PROVIDED_NUMBER_OF_CHANNELS < EXPECTED_NUMBER_OF_CHANNELS)
    {
        ROS_ERROR("Signaling channels can't be constructed as %d signaling channels provided, \
            but %d signaling channels expected", PROVIDED_NUMBER_OF_CHANNELS, EXPECTED_NUMBER_OF_CHANNELS);
        return {};
    }

    std::vector<std::shared_ptr<servo::IServo>> servos;
    for (size_t i = 0; i < PROVIDED_NUMBER_OF_CHANNELS; ++i)
    {
        servos.emplace_back(std::make_shared<servo::ServoMG90S>(static_cast<uint8_t>(i), signalingChannels[i]));
    }

    return servos;
}

//---------------------------------------------------------------------------

ServoNode::ServoNode()
{
    buildSignalingChannelComponents();
    buildServoComponents();
    buildCalibrationTableComponents();
}

void ServoNode::listenerSignalingChannelState(const catix_messages::SignalingChannelStateConstPtr &signalingChannelState)
{
    const size_t signalingChannelIndex = static_cast<size_t>(signalingChannelState->signaling_channel_index);
    const double signalStrengthPercentage = signalingChannelState->signal_strength_percentage;

    if (signalingChannelIndex >= this->channels.size())
    {
        ROS_ERROR("Signaling channel %d: Can't set signal strength %f%% as channel is not available", 
            signalingChannelIndex, signalStrengthPercentage);
        return;
    }

    this->channels[signalingChannelIndex]->setStrength(signalStrengthPercentage);
    ROS_INFO("Signaling channel %d: [%f%%]", signalingChannelIndex, signalStrengthPercentage);
}

void ServoNode::listenerSignalingDrop(const std_msgs::EmptyConstPtr &dropEventMessage)
{
    if (this->signalingController == nullptr)
    {
        ROS_WARN("Signaling controller: can't drop signaling channels as controller is not available");
        return;
    }

    this->signalingController->dropAllChannels();
}

void ServoNode::listenerServoState(const catix_messages::ServoStateConstPtr &servoState)
{
    const size_t servoIndex = static_cast<size_t>(servoState->servo_index);
    const double rotateAngle = servoState->rotate_angle;

    if (servoIndex >= this->servos.size())
    {
        ROS_ERROR("Servo %d: can't set rotate angle as servo is not available", servoIndex);
        return;
    }

    if(!this->servos[servoIndex]->setAngle(rotateAngle))
    {
        ROS_ERROR("Servo %d: can't set rotate angle %frad value", 
            servoIndex, rotateAngle);
        return;
    }

    ROS_INFO("Servo %d: [%frad]",  servoIndex, rotateAngle);
}

void ServoNode::listenerCalibrationFirstPoint(const catix_messages::CalibrationPointValueConstPtr &calibrationPointValue)
{
    size_t servoIndex = static_cast<size_t>(calibrationPointValue->servo_index);
    auto signalStrengthPercentage = calibrationPointValue->signal_strength_percentage;
    auto rotateAngle = calibrationPointValue->rotate_angle;

    this->calibrationTable->resetPoints(servoIndex);
    ROS_INFO("Servo %d: calibration points reset to undefined state", servoIndex);

    if(!this->calibrationTable->setFirstPoint(servoIndex, signalStrengthPercentage, rotateAngle))
    {
        ROS_ERROR("Servo %d: can't set first calibration point", servoIndex);
        return;
    }

    ROS_INFO("Servo %d: first calibration point set to [%f%%; %frad]",  
        servoIndex, signalStrengthPercentage, rotateAngle);
}

void ServoNode::listenerCalibrationSecondPoint(const catix_messages::CalibrationPointValueConstPtr &calibrationPointValue)
{
    size_t servoIndex = static_cast<size_t>(calibrationPointValue->servo_index);
    auto signalStrengthPercentage = calibrationPointValue->signal_strength_percentage;
    auto rotateAngle = calibrationPointValue->rotate_angle;

    if(!this->calibrationTable->setSecondPoint(servoIndex, signalStrengthPercentage, rotateAngle))
    {
        ROS_ERROR("Servo %d: can't set second calibration point", servoIndex);
        return;
    }

    ROS_INFO("Servo %d: second calibration point set to [%f%%; %frad]",  
        servoIndex, signalStrengthPercentage, rotateAngle);
}

void ServoNode::listenerCalibrationLowerLimit(const catix_messages::CalibrationLimitValueConstPtr &calibrationLimitValue)
{
    const size_t servoIndex = static_cast<size_t>(calibrationLimitValue->servo_index);
    const double signalStrengthPercentage = calibrationLimitValue->signal_strength_percentage;

    if(!this->calibrationTable->setLowerLimit(servoIndex, signalStrengthPercentage))
    {
        ROS_ERROR("Servo %d: can't set lower limit", servoIndex);
        return;
    }

    ROS_INFO("Servo %d: lower limit set to [%f%%]", servoIndex, signalStrengthPercentage);
}

void ServoNode::listenerCalibrationUpperLimit(const catix_messages::CalibrationLimitValueConstPtr &calibrationLimitValue)
{
    const size_t servoIndex = static_cast<size_t>(calibrationLimitValue->servo_index);
    const double signalStrengthPercentage = calibrationLimitValue->signal_strength_percentage;

    if(!this->calibrationTable->setUpperLimit(servoIndex, signalStrengthPercentage))
    {
        ROS_ERROR("Servo %d: can't set upper limit", servoIndex);
        return;
    }
        
    ROS_INFO("Servo %d: upper limit set to [%f%%]", servoIndex, signalStrengthPercentage);
}

void ServoNode::buildSignalingChannelComponents()
{
    this->signalingController = makeSignalingController();
    this->channels = makeSignalingChannels(NUMBER_OF_CHANNELS, *this->signalingController);
    this->subscriberSignalingChannel = nodeHandle.subscribe("Catix/SignalingChannel", 
        10, &ServoNode::listenerSignalingChannelState, this);
    this->subscriberSignalingDrop = nodeHandle.subscribe("Catix/SignalingDrop", 
        1, &ServoNode::listenerSignalingDrop, this);
    ROS_INFO("Signaling channel: components built");
}

void ServoNode::buildServoComponents()
{
    this->servos = makeServos(this->channels);
    subscriberServo = nodeHandle.subscribe("Catix/Servo", 10, &ServoNode::listenerServoState, this);
    ROS_INFO("Servo: listener ready");
}

void ServoNode::buildCalibrationTableComponents()
{
    this->calibrationTable = std::make_unique<servo::CalibrationTable>(nodeHandle, NUMBER_OF_CHANNELS);
    for (size_t i = 0; i < this->servos.size(); ++i)
    {
        this->calibrationTable->subscribeListener(i, this->servos[i]);
    }

    this->calibrationTable->updateListeners();
    ROS_INFO("Calibration table: subscribers ready");

    subscriberCalibrationFirstPoint = nodeHandle.subscribe("Catix/CalibrationFirstPoint", 1, &ServoNode::listenerCalibrationFirstPoint, this);
    ROS_INFO("Calibration table: first point listener ready");

    subscriberCalibrationSecondPoint = nodeHandle.subscribe("Catix/CalibrationSecondPoint", 1, &ServoNode::listenerCalibrationSecondPoint, this);
    ROS_INFO("Calibration table: second point listener ready");

    subscriberCalibrationLowerLimit = nodeHandle.subscribe("Catix/CalibrationLowerLimit", 1, &ServoNode::listenerCalibrationLowerLimit, this);
    ROS_INFO("Calibration table: lower limit listener ready");

    subscriberCalibrationUpperLimit = nodeHandle.subscribe("Catix/CalibrationUpperLimit", 1, &ServoNode::listenerCalibrationUpperLimit, this);
    ROS_INFO("Calibration table: upper limit listener ready");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ServoNode");
    if (!ros::master::check())
    {
        std::cerr << "ROS Master is not running" << std::endl;
        return 1;
    }

    ServoNode ServoNode;
    ros::spin();
    return 0;
}
