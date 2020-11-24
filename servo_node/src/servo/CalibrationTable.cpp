#include "servo/CalibrationTable.h"

#include <cmath>

//---------------------------------------------------------------------------

std::string generateServoKey(size_t servoIndex)
{
    return "servo_" + std::to_string(servoIndex);
}

std::string generateFirstPointPercentageKey(std::string servoKey)
{
    return servoKey + "/FirstPointPercentage";
}

std::string generateFirstPointAngleKey(std::string servoKey)
{
    return servoKey + "/FirstPointAngle";
}

std::string generateSecondPointPercentageKey(std::string servoKey)
{
    return servoKey + "/SecondPointPercentage";
}

std::string generateSecondPointAngleKey(std::string servoKey)
{
    return servoKey + "/SecondPointAngle";
}

//---------------------------------------------------------------------------

servo::CalibrationTable::CalibrationTable(ros::NodeHandle &nodeHandle, size_t tableSize)
    : tableInput(tableSize)
    , tableOutput(tableSize)
    , nodeHandle(nodeHandle)
{
    this->loadAllCalibrationPoints();
    this->calculateAllCalibrations();
}

void servo::CalibrationTable::resetPoints(size_t index)
{
    if (index > this->tableInput.size())
    {
        ROS_ERROR("Servo %d: Can't reset calibration points as index %d is out of table range", 
            index, index);
    }

    this->tableInput[index] = CalibrationPoints{};
}

bool servo::CalibrationTable::setFirstPoint(size_t index, double pulseWidth, double rotateAngle)
{
    if (index > this->tableInput.size())
    {
        ROS_ERROR("Servo %d: Can't set first calibration point as index %d is out of table range", 
            index, index);
    }

    this->tableInput[index].firstPointSignalStrength = pulseWidth;
    this->tableInput[index].firstPointRotateAngle = rotateAngle;
    return true;
}

bool servo::CalibrationTable::setSecondPoint(size_t index, double pulseWidth, double rotateAngle)
{
    if (index > this->tableInput.size())
    {
        ROS_ERROR("Servo %d: Can't set second calibration point as index %d is out of table range", 
            index, index);
        return false;
    }

    this->tableInput[index].secondPointSignalStrength = pulseWidth;
    this->tableInput[index].secondPointRotateAngle = rotateAngle;

    if (!this->calculateIndexCalibration(index, this->tableInput[index]))
    {
        return false;
    }

    storeCalibrationPoints(generateServoKey(index), this->tableInput[index]);
    return true;
}

bool servo::CalibrationTable::setLowerLimit(size_t index, double pulseWidth)
{
    if (index > this->tableOutput.size())
    {
        ROS_ERROR("Servo %d: Can't set lower limit in calibration table as index %d is out of table range", 
            index, index);
        return false;
    }

    this->tableOutput[index].pulseWidthMinimum = pulseWidth;
    return true;
}

bool servo::CalibrationTable::setUpperLimit(size_t index, double pulseWidth)
{
    if (index > this->tableOutput.size())
    {
        ROS_ERROR("Servo %d: Can't set upper limit in calibration table as index %d is out of table range", 
            index, index);
        return false;
    }

    this->tableOutput[index].pulseWidthMaximum = pulseWidth;
    return true;
}

bool servo::CalibrationTable::isCalibrationPointsCorrect(const CalibrationPoints &calibrationPoints)
{
    return calibrationPoints.firstPointSignalStrength >= 0.0 && calibrationPoints.secondPointSignalStrength >= 0.0 &&
        std::abs(calibrationPoints.firstPointRotateAngle - calibrationPoints.secondPointRotateAngle) > 0.0;
}

bool servo::CalibrationTable::calculateAllCalibrations()
{
    for (size_t i = 0; i < this->tableInput.size(); ++i)
    {
        calculateIndexCalibration(i, this->tableInput[i]);
    }
}

bool servo::CalibrationTable::calculateIndexCalibration(size_t index, const CalibrationPoints &calibrationPoints)
{
    if (!this->isCalibrationPointsCorrect(calibrationPoints))
    {
        ROS_WARN("Servo %d: Can't calculate calibration at index %d as its points are incorrect. \
            Repeat calibration procedure starting from the first point", index, index);
        return false;
    }

    CalibrationPoints modifiedCalibrationPoints = calibrationPoints;
    if (modifiedCalibrationPoints.firstPointRotateAngle > modifiedCalibrationPoints.secondPointRotateAngle)
    {
        std::swap(modifiedCalibrationPoints.firstPointRotateAngle, modifiedCalibrationPoints.secondPointRotateAngle);
        std::swap(modifiedCalibrationPoints.firstPointSignalStrength, modifiedCalibrationPoints.secondPointSignalStrength);        
    }

    auto &servoParameters = this->tableOutput[index];
    servoParameters.pulseWidthToAngleSlope = (modifiedCalibrationPoints.secondPointSignalStrength - modifiedCalibrationPoints.firstPointSignalStrength) /
        (modifiedCalibrationPoints.secondPointRotateAngle - modifiedCalibrationPoints.firstPointRotateAngle);
    servoParameters.pulseWidthOffset = modifiedCalibrationPoints.secondPointSignalStrength - 
        (servoParameters.pulseWidthToAngleSlope * modifiedCalibrationPoints.firstPointRotateAngle);
    return true;
}

void servo::CalibrationTable::loadAllCalibrationPoints()
{
    for (size_t i = 0; i < this->tableInput.size(); ++i)
    {
        const auto servoKey = generateServoKey(i);
        if (!this->nodeHandle.hasParam(servoKey))
        {
            storeCalibrationPoints(servoKey, CalibrationPoints{});
        }
        else
        {
            this->tableInput[i] = loadCalibrationPoints(servoKey);
        }
    }
}

void servo::CalibrationTable::storeCalibrationPoints(std::string servoKey, const CalibrationPoints& calibrationPoints)
{
    this->nodeHandle.setParam(generateFirstPointPercentageKey(servoKey), calibrationPoints.firstPointSignalStrength);
    this->nodeHandle.setParam(generateFirstPointAngleKey(servoKey), calibrationPoints.firstPointRotateAngle);
    this->nodeHandle.setParam(generateSecondPointPercentageKey(servoKey), calibrationPoints.secondPointSignalStrength);
    this->nodeHandle.setParam(generateSecondPointAngleKey(servoKey), calibrationPoints.secondPointRotateAngle);
}

servo::CalibrationTable::CalibrationPoints servo::CalibrationTable::loadCalibrationPoints(std::string servoKey)
{
    CalibrationPoints calibrationPoints;
    if (this->nodeHandle.getParam(generateFirstPointPercentageKey(servoKey), calibrationPoints.firstPointSignalStrength) &&
        this->nodeHandle.getParam(generateFirstPointAngleKey(servoKey), calibrationPoints.firstPointRotateAngle) &&
        this->nodeHandle.getParam(generateSecondPointPercentageKey(servoKey), calibrationPoints.secondPointSignalStrength) &&
        this->nodeHandle.getParam(generateSecondPointAngleKey(servoKey), calibrationPoints.secondPointRotateAngle))
    {
        return calibrationPoints;
    }

    ROS_ERROR("Can't load calibration points by key '%s'", servoKey.c_str());
    return CalibrationPoints{};
}
