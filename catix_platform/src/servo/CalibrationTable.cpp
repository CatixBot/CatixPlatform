#include "servo/CalibrationTable.h"

#include <ros/ros.h>

#include <cmath>

//---------------------------------------------------------------------------

servo::CalibrationTable::CalibrationTable(size_t tableSize)
    : tableInput(tableSize)
    , tableOutput(tableSize)
{
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

    this->tableInput[index].firstPointPulseWidth = pulseWidth;
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

    this->tableInput[index].secondPointPulseWidth = pulseWidth;
    this->tableInput[index].secondPointRotateAngle = rotateAngle;

    if (!this->isCalibrationPointsCorrect(this->tableInput[index]))
    {
        ROS_ERROR("Servo %d: Can't calculate calibration at index %d as its points are incorrect. \
            Repeat calibration procedure starting from the first point", index, index);
        return false;
    }

    this->calculateCalibration(index, this->tableInput[index]);
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
    return calibrationPoints.firstPointPulseWidth >= 0.0 && calibrationPoints.secondPointPulseWidth >= 0.0 &&
        std::abs(calibrationPoints.firstPointRotateAngle - calibrationPoints.secondPointRotateAngle) > 0.0;
}

void servo::CalibrationTable::calculateCalibration(size_t index, const CalibrationPoints &calibrationPoints)
{
    CalibrationPoints modifiedCalibrationPoints = calibrationPoints;
    if (modifiedCalibrationPoints.firstPointRotateAngle > modifiedCalibrationPoints.secondPointRotateAngle)
    {
        std::swap(modifiedCalibrationPoints.firstPointRotateAngle, modifiedCalibrationPoints.secondPointRotateAngle);
        std::swap(modifiedCalibrationPoints.firstPointPulseWidth, modifiedCalibrationPoints.secondPointPulseWidth);        
    }

    auto &servoParameters = this->tableOutput[index];
    servoParameters.pulseWidthToAngleSlope = (modifiedCalibrationPoints.secondPointPulseWidth - modifiedCalibrationPoints.firstPointPulseWidth) /
        (modifiedCalibrationPoints.secondPointRotateAngle - modifiedCalibrationPoints.firstPointRotateAngle);
    servoParameters.pulseWidthOffset = modifiedCalibrationPoints.secondPointPulseWidth - 
        (servoParameters.pulseWidthToAngleSlope * modifiedCalibrationPoints.firstPointRotateAngle);
}