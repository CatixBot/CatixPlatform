#include "PlatformNode.h"

#include "platform/PlatformParameters.h"
#include "platform/Platform8Dof.h"
#include "limb/LimbSegment.h"
#include "limb/PolarCoordinates.h"
#include "limb/Leg2Dof.h"
#include "servo/SignalingControllerPCA9685.h"
#include "servo/SignalingChannelShared.h"
#include "servo/ServoMG90S.h"

#include <cmath>

//---------------------------------------------------------------------------

#define NUMBER_OF_CHANNELS 16

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

    if (PROVIDED_NUMBER_OF_CHANNELS != EXPECTED_NUMBER_OF_CHANNELS)
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

std::vector<std::shared_ptr<limb::ILimb2Dof>> makeLegs2Dof(
    const std::vector<std::shared_ptr<servo::IServo>>& servos)
{
    const size_t EXPECTED_NUMBER_OF_SERVOS = 8;
    const size_t PROVIDED_NUMBER_OF_SERVOS = servos.size();

    if (PROVIDED_NUMBER_OF_SERVOS != EXPECTED_NUMBER_OF_SERVOS)
    {
        ROS_ERROR("Legs can't be constructed as %d servos provided, but %d servos expected", 
            PROVIDED_NUMBER_OF_SERVOS, EXPECTED_NUMBER_OF_SERVOS);
        return {};
    }

    std::vector<std::shared_ptr<limb::ILimb2Dof>> legs;

    const double LOWER_SEGMENT_LENGTH_METERS = 0.060;
    const double UPPER_SEGMENT_LENGTH_METERS = 0.049;

    limb::LimbSegment frontLeftLowerSegment;
    frontLeftLowerSegment.jointServo = servos[0];
    frontLeftLowerSegment.linkLength = LOWER_SEGMENT_LENGTH_METERS;
    limb::LimbSegment frontLeftUpperSegment;
    frontLeftUpperSegment.jointServo = servos[1];
    frontLeftUpperSegment.linkLength = UPPER_SEGMENT_LENGTH_METERS;
    auto frontLeftLeg = std::make_shared<limb::Leg2Dof>(frontLeftLowerSegment, frontLeftUpperSegment);
    legs.push_back(frontLeftLeg);

    limb::LimbSegment frontRightLowerSegment;
    frontRightLowerSegment.jointServo = servos[2];
    frontRightLowerSegment.linkLength = LOWER_SEGMENT_LENGTH_METERS;
    limb::LimbSegment frontRightUpperSegment;
    frontRightUpperSegment.jointServo = servos[3];
    frontRightUpperSegment.linkLength = UPPER_SEGMENT_LENGTH_METERS;
    auto frontRightLeg = std::make_shared<limb::Leg2Dof>(frontRightLowerSegment, frontRightUpperSegment);
    legs.push_back(frontRightLeg);

    limb::LimbSegment rearRightLowerSegment;
    rearRightLowerSegment.jointServo = servos[4];
    rearRightLowerSegment.linkLength = LOWER_SEGMENT_LENGTH_METERS;
    limb::LimbSegment rearRightUpperSegment;
    rearRightUpperSegment.jointServo = servos[5];
    rearRightUpperSegment.linkLength = UPPER_SEGMENT_LENGTH_METERS;
    auto rearRightLeg = std::make_shared<limb::Leg2Dof>(rearRightLowerSegment, rearRightUpperSegment);
    legs.push_back(rearRightLeg);

    limb::LimbSegment rearLeftLowerSegment;
    rearLeftLowerSegment.jointServo = servos[6];
    rearLeftLowerSegment.linkLength = LOWER_SEGMENT_LENGTH_METERS;
    limb::LimbSegment rearLeftUpperSegment;
    rearLeftUpperSegment.jointServo = servos[7];
    rearLeftUpperSegment.linkLength = UPPER_SEGMENT_LENGTH_METERS;
    auto rearLeftLeg = std::make_shared<limb::Leg2Dof>(rearLeftLowerSegment, rearLeftUpperSegment);
    legs.push_back(rearLeftLeg);

    return legs;
}

std::unique_ptr<platform::IPlatform> makePlatform8Dof(const std::vector<std::shared_ptr<limb::ILimb2Dof>>& legs)
{
    const size_t EXPECTED_NUMBER_OF_LEGS = 4;
    const size_t PROVIDED_NUMBER_OF_LEGS = legs.size();

    if (PROVIDED_NUMBER_OF_LEGS != EXPECTED_NUMBER_OF_LEGS)
    {
        ROS_ERROR("Platform can't be constructed as %d legs provided, but %d legs expected", 
            PROVIDED_NUMBER_OF_LEGS, EXPECTED_NUMBER_OF_LEGS);
        return nullptr;
    }

    platform::Platform8DofParameters platformParameters;
    platformParameters.frontLeftLeg = legs[0];
    platformParameters.frontRightLeg = legs[1];
    platformParameters.rearRightLeg = legs[2];
    platformParameters.rearLeftLeg = legs[3];

    return std::make_unique<platform::Platform8Dof>(platformParameters);
}

//---------------------------------------------------------------------------

PlatformNode::PlatformNode()
    : calibrationTable(NUMBER_OF_CHANNELS)
{
    this->signalingController = makeSignalingController();
    this->channels = makeSignalingChannels(NUMBER_OF_CHANNELS, *this->signalingController);
    this->subscriberSignalingChannel = node.subscribe("Catix/SignalingChannel", 
        100, &PlatformNode::listenerSignalingChannelState, this);
    ROS_INFO("Signaling channels listener ready...");

    this->servos = makeServos(this->channels);
    subscriberServo = node.subscribe("Catix/Servo", 100, &PlatformNode::listenerServoState, this);
    ROS_INFO("Servos listener ready...");

    this->legs = makeLegs2Dof(this->servos);
    subscriberLeg = node.subscribe("Catix/Leg2Dof", 100, &PlatformNode::listenerLegState, this);
    ROS_INFO("2DOF legs listener is ready...");
 
    this->platform = makePlatform8Dof(this->legs);
    subscriberPlatform = node.subscribe("Catix/Platform8Dof", 100, &PlatformNode::listenerPlatformState, this);
    ROS_INFO("8DOF platform listener is ready...");

    subscriberCalibrationFirstPoint = node.subscribe("Catix/CalibrationFirstPoint", 100, &PlatformNode::listenerCalibrationFirstPoint, this);
    ROS_INFO("Calibration first points listener is ready...");

    subscriberCalibrationSecondPoint = node.subscribe("Catix/CalibrationSecondPoint", 100, &PlatformNode::listenerCalibrationSecondPoint, this);
    ROS_INFO("Calibration second points listener is ready...");

    subscriberCalibrationLowerLimit = node.subscribe("Catix/CalibrationLowerLimit", 100, &PlatformNode::listenerCalibrationLowerLimit, this);
    ROS_INFO("Lower limits listener is ready...");

    subscriberCalibrationUpperLimit = node.subscribe("Catix/CalibrationUpperLimit", 100, &PlatformNode::listenerCalibrationUpperLimit, this);
    ROS_INFO("Upper limits listener is ready...");
}

void PlatformNode::listenerSignalingChannelState(const catix_messages::SignalingChannelStateConstPtr &signalingChannelState)
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

void PlatformNode::listenerServoState(const catix_messages::ServoStateConstPtr &servoState)
{
    const size_t servoIndex = static_cast<size_t>(servoState->servo_index);
    const double rotateAngle = servoState->rotate_angle;

    if (servoIndex >= this->servos.size())
    {
        ROS_ERROR("Servo %d: Can't set rotate angle as servo is not available", servoIndex);
        return;
    }

    if(!this->servos[servoIndex]->setAngle(rotateAngle))
    {
        ROS_ERROR("Servo %d: Settings rotate angle %frad failed", 
            servoIndex, rotateAngle);
        return;
    }

    ROS_INFO("Servo %d: [%frad]",  servoIndex, rotateAngle);
}

void PlatformNode::listenerLegState(const catix_messages::TwoDofLegStateConstPtr &legState)
{
    const size_t legIndex = static_cast<size_t>(legState->leg_index);
    const geometry::PolarCoordinates targetPosition
    {
        legState->radial_coordinate, 
        legState->angular_coordinate 
    };

    if (legIndex >= this->legs.size())
    {
        ROS_ERROR("Leg %d: Can't set position as leg is not available", legIndex);
        return;
    }

    if (!this->legs[legIndex]->setPosition(targetPosition))
    {
        ROS_ERROR("Leg %d: Setting position failed", legIndex);
        return;
    }

    ROS_INFO("Leg %d: [%fm; %frad]",  legIndex, 
        targetPosition.radialCoordinate, targetPosition.angularCoordinate);
}

void PlatformNode::listenerPlatformState(const catix_messages::EightDofPlatformStateConstPtr &platformState)
{
    const double moveForwardSpeed = platformState->move_forward_speed;
    const double rotateClockwiseSpeed = platformState->rotate_clockwise_speed;

    platform->setSpeed(moveForwardSpeed, rotateClockwiseSpeed);
    ROS_INFO("Platform: [%fm/s; %frad/s]",  moveForwardSpeed, rotateClockwiseSpeed);
}

void PlatformNode::listenerCalibrationFirstPoint(const catix_messages::CalibrationPointValueConstPtr &calibrationPointValue)
{
    size_t servoIndex = static_cast<size_t>(calibrationPointValue->servo_index);
    auto signalStrengthPercentage = calibrationPointValue->signal_strength_percentage;
    auto rotateAngle = calibrationPointValue->rotate_angle;

    this->calibrationTable.resetPoints(servoIndex);
    ROS_INFO("Servo %d: Calibration points reset to undefined values", servoIndex);

    if(!calibrationTable.setFirstPoint(servoIndex, signalStrengthPercentage, rotateAngle))
    {
        ROS_ERROR("Servo %d: Can't set first calibration point", servoIndex);
        return;
    }

    ROS_INFO("Servo %d: First calibration point set to [%f%%; %fm/s]",  
        servoIndex, signalStrengthPercentage, rotateAngle);
}

void PlatformNode::listenerCalibrationSecondPoint(const catix_messages::CalibrationPointValueConstPtr &calibrationPointValue)
{
    size_t servoIndex = static_cast<size_t>(calibrationPointValue->servo_index);
    auto signalStrengthPercentage = calibrationPointValue->signal_strength_percentage;
    auto rotateAngle = calibrationPointValue->rotate_angle;

    if(!calibrationTable.setSecondPoint(servoIndex, signalStrengthPercentage, rotateAngle))
    {
        ROS_ERROR("Servo %d: Can't set first calibration point", servoIndex);
        return;
    }

    ROS_INFO("Servo %d: Second calibration point set to [%f%%; %fm/s]",  
        servoIndex, signalStrengthPercentage, rotateAngle);
}

void PlatformNode::listenerCalibrationLowerLimit(const catix_messages::CalibrationLimitValueConstPtr &calibrationLimitValue)
{
    const size_t servoIndex = static_cast<size_t>(calibrationLimitValue->servo_index);
    const double signalStrengthPercentage = calibrationLimitValue->signal_strength_percentage;

    if(!calibrationTable.setLowerLimit(servoIndex, signalStrengthPercentage))
    {
        ROS_ERROR("Servo %d: Can't set lower limit", servoIndex);
        return;
    }

    ROS_INFO("Servo %d: Lower limit set to [%f%%]", servoIndex, signalStrengthPercentage);
}

void PlatformNode::listenerCalibrationUpperLimit(const catix_messages::CalibrationLimitValueConstPtr &calibrationLimitValue)
{
    const size_t servoIndex = static_cast<size_t>(calibrationLimitValue->servo_index);
    const double signalStrengthPercentage = calibrationLimitValue->signal_strength_percentage;

    if(!calibrationTable.setUpperLimit(servoIndex, signalStrengthPercentage))
    {
        ROS_ERROR("Servo %d: Can't set upper limit", servoIndex);
        return;
    }
        
    ROS_INFO("Servo %d: Lower upper set to [%f%%]", servoIndex, signalStrengthPercentage);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CatixPlatform");

    PlatformNode platformNode;
    ros::spin();
}
