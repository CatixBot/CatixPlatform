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

std::string generateLowerLimitKey(std::string servoKey)
{
    return servoKey + "/LowerLimit";
}

std::string generateUpperLimitKey(std::string servoKey)
{
    return servoKey + "/UpperLimit";
}

//---------------------------------------------------------------------------

servo::CalibrationTable::CalibrationTable(ros::NodeHandle &nodeHandle, size_t tableSize)
    : tableInput(tableSize)
    , tableOutput(tableSize)
    , nodeHandle(nodeHandle)
{
    this->loadAllCalibrationPoints();
    this->calculateAllCalibrations();
    this->loadAllLimits();
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

bool servo::CalibrationTable::setFirstPoint(size_t index, double signalStrength, double rotateAngle)
{
    if (index > this->tableInput.size())
    {
        ROS_ERROR("Servo %d: Can't set first calibration point as index %d is out of table range", 
            index, index);
    }

    this->tableInput[index].firstPointSignalStrength = signalStrength;
    this->tableInput[index].firstPointRotateAngle = rotateAngle;
    return true;
}

bool servo::CalibrationTable::setSecondPoint(size_t index, double signalStrength, double rotateAngle)
{
    if (index > this->tableInput.size())
    {
        ROS_ERROR("Servo %d: Can't set second calibration point as index %d is out of table range", 
            index, index);
        return false;
    }

    this->tableInput[index].secondPointSignalStrength = signalStrength;
    this->tableInput[index].secondPointRotateAngle = rotateAngle;

    if (!this->calculateIndexCalibration(index, this->tableInput[index]))
    {
        return false;
    }

    storeCalibrationPoints(generateServoKey(index), this->tableInput[index]);
    return true;
}

bool servo::CalibrationTable::setLowerLimit(size_t index, double signalStrength)
{
    if (index > this->tableOutput.size())
    {
        ROS_ERROR("Servo %d: Can't set lower limit in calibration table as index %d is out of table range", 
            index, index);
        return false;
    }

    this->tableOutput[index].signalStrengthMinimum = signalStrength;
    storeLowerLimit(generateServoKey(index), signalStrength);
    return true;
}

bool servo::CalibrationTable::setUpperLimit(size_t index, double signalStrength)
{
    if (index > this->tableOutput.size())
    {
        ROS_ERROR("Servo %d: Can't set upper limit in calibration table as index %d is out of table range", 
            index, index);
        return false;
    }

    this->tableOutput[index].signalStrengthMaximum = signalStrength;
    storeUpperLimit(generateServoKey(index), signalStrength);
    return true;
}

bool servo::CalibrationTable::isCalibrationPointsCorrect(const CalibrationPoints &calibrationPoints)
{
    return calibrationPoints.firstPointSignalStrength >= 0.0 && calibrationPoints.secondPointSignalStrength >= 0.0 &&
        std::abs(calibrationPoints.firstPointRotateAngle - calibrationPoints.secondPointRotateAngle) > 0.0;
}

bool servo::CalibrationTable::calculateAllCalibrations()
{
    ROS_DEBUG("Calibration table: calculate all calibrations");

    for (size_t i = 0; i < this->tableInput.size(); ++i)
    {
        calculateIndexCalibration(i, this->tableInput[i]);
    }
}

bool servo::CalibrationTable::calculateIndexCalibration(size_t index, const CalibrationPoints &calibrationPoints)
{
    ROS_DEBUG("Calibration table: calculate servo %d calibration", index);

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
    servoParameters.signalStrengthToAngleSlope = (modifiedCalibrationPoints.secondPointSignalStrength - modifiedCalibrationPoints.firstPointSignalStrength) /
        (modifiedCalibrationPoints.secondPointRotateAngle - modifiedCalibrationPoints.firstPointRotateAngle);
    servoParameters.signalStrengthOffset = modifiedCalibrationPoints.secondPointSignalStrength - 
        (servoParameters.signalStrengthToAngleSlope * modifiedCalibrationPoints.firstPointRotateAngle);
    return true;
}

void servo::CalibrationTable::loadAllCalibrationPoints()
{
    ROS_DEBUG("Calibration table: load all calibration points");

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
    ROS_DEBUG("Calibration table: load calibration points by %s key", servoKey.c_str());

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

void servo::CalibrationTable::loadAllLimits()
{
    ROS_DEBUG("Calibration table: load all calibration limits");

    for (size_t i = 0; i < this->tableOutput.size(); ++i)
    {
        const auto servoKey = generateServoKey(i);
        if (!this->nodeHandle.hasParam(servoKey))
        {
            storeLowerLimit(servoKey, SIGNAL_STRENGTH_MINIMUM_DEFAULT);
            storeUpperLimit(servoKey, SIGNAL_STRENGTH_MAXIMUM_DEFAULT);
        }
        else
        {
            this->tableOutput[i].signalStrengthMinimum = loadLowerLimit(servoKey);
            this->tableOutput[i].signalStrengthMaximum = loadUpperLimit(servoKey);
        }
    }
}

void servo::CalibrationTable::storeLowerLimit(std::string servoKey, double lowerLimit)
{
    ROS_DEBUG("Calibration table: load lower limit by '%s' key", servoKey.c_str());

    this->nodeHandle.setParam(generateLowerLimitKey(servoKey), lowerLimit);
}

double servo::CalibrationTable::loadLowerLimit(std::string servoKey)
{
    ROS_DEBUG("Calibration table: load lower limit by '%s' key", servoKey.c_str());

    double lowerLimit = SIGNAL_STRENGTH_MINIMUM_DEFAULT;
    if (this->nodeHandle.getParam(generateLowerLimitKey(servoKey), lowerLimit))
    {
        ROS_DEBUG("Calibration table: lower limit value %f%% loaded", lowerLimit);
    }
    else
    {
        ROS_ERROR("Calibration table: Can't load lower limit by key '%s'. Return %f%% value by default", 
            servoKey.c_str(), lowerLimit);
    }

    return lowerLimit;
}

void servo::CalibrationTable::storeUpperLimit(std::string servoKey, double upperLimit)
{
    ROS_DEBUG("Calibration table: load upper limit by '%s' key", servoKey.c_str());

    this->nodeHandle.setParam(generateUpperLimitKey(servoKey), upperLimit);
}

double servo::CalibrationTable::loadUpperLimit(std::string servoKey)
{
    ROS_DEBUG("Calibration table: load upper limit by '%s' key", servoKey.c_str());

    double upperLimit = SIGNAL_STRENGTH_MAXIMUM_DEFAULT;
    if (this->nodeHandle.getParam(generateUpperLimitKey(servoKey), upperLimit))
    {
        ROS_DEBUG("Calibration table: upper limit value %f%% loaded", upperLimit);
    }
    {
        ROS_ERROR("Calibration table: Can't load upper limit by key '%s'. Return %f%% value by default", 
            servoKey.c_str(), upperLimit);
    }

    return upperLimit;
}
