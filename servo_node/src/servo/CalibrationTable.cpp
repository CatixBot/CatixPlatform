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
    : tableListeners(tableSize)
    , tableInput(tableSize)
    , tableOutput(tableSize)
    , nodeHandle(nodeHandle)
{
    loadAllCalibrationPoints();
    calculateAllCalibrations();
    loadAllLimits();
}

void servo::CalibrationTable::resetPoints(size_t index)
{
    if (index > this->tableInput.size())
    {
        ROS_ERROR("Calibration table: can't reset calibration points as index %d is out of table range", index);
    }

    this->tableInput[index] = CalibrationPoints{};
}

bool servo::CalibrationTable::setFirstPoint(size_t index, double signalStrength, double rotateAngle)
{
    if (index > this->tableInput.size())
    {
        ROS_ERROR("Calibration table: can't set first calibration point as index %d is out of table range", index);
    }

    this->tableInput[index].firstPointSignalStrength = signalStrength;
    this->tableInput[index].firstPointRotateAngle = rotateAngle;
    return true;
}

bool servo::CalibrationTable::setSecondPoint(size_t index, double signalStrength, double rotateAngle)
{
    if (index > this->tableInput.size())
    {
        ROS_ERROR("Calibration table: can't set second calibration point as index %d is out of table range", index);
        return false;
    }

    this->tableInput[index].secondPointSignalStrength = signalStrength;
    this->tableInput[index].secondPointRotateAngle = rotateAngle;

    if (!this->calculateIndexCalibration(index, this->tableInput[index]))
    {
        ROS_ERROR("Calibration table: can't calculate calibration at index %d", index);
        return false;
    }

    storeCalibrationPoints(generateServoKey(index), this->tableInput[index]);
    notifyServoParameters(index);
    return true;
}

bool servo::CalibrationTable::setLowerLimit(size_t index, double signalStrength)
{
    if (index > this->tableOutput.size())
    {
        ROS_ERROR("Calibration table: can't set lower limit in calibration table as index %d is out of table range", index);
        return false;
    }

    this->tableOutput[index].signalStrengthMinimum = signalStrength;
    storeLowerLimit(generateServoKey(index), signalStrength);
    notifyServoParameters(index);
    return true;
}

bool servo::CalibrationTable::setUpperLimit(size_t index, double signalStrength)
{
    if (index > this->tableOutput.size())
    {
        ROS_ERROR("Calibration table: can't set upper limit in calibration table as index %d is out of table range", index);
        return false;
    }

    this->tableOutput[index].signalStrengthMaximum = signalStrength;
    storeUpperLimit(generateServoKey(index), signalStrength);
    notifyServoParameters(index);
    return true;
}

void servo::CalibrationTable::subscribeListener(size_t index, std::shared_ptr<servo::IServo> servo)
{
    if (index > this->tableListeners.size())
    {
        ROS_ERROR("Calibration table: can't subscribe listener for calibration parameters "
            "as index %d is out of table range", index);
        return;
    }

    tableListeners[index] = servo;
}

void servo::CalibrationTable::updateListeners()
{
    for (size_t i = 0; i < this->tableListeners.size(); ++i)
    {
        notifyServoParameters(i);
    }
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
        ROS_WARN("Calibration table: can't calculate calibration at index %d as its points are incorrect. "
            "Repeat calibration procedure starting from the first point", index);
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
    servoParameters.signalStrengthOffset = modifiedCalibrationPoints.firstPointSignalStrength - 
        (servoParameters.signalStrengthToAngleSlope * modifiedCalibrationPoints.firstPointRotateAngle);
    return true;
}

void servo::CalibrationTable::loadAllCalibrationPoints()
{
    ROS_DEBUG("Calibration table: load all calibration points");

    for (size_t i = 0; i < this->tableInput.size(); ++i)
    {
        const auto servoKey = generateServoKey(i);
        if (!this->nodeHandle.hasParam(generateFirstPointPercentageKey(servoKey)) ||
            !this->nodeHandle.hasParam(generateFirstPointAngleKey(servoKey)) ||
            !this->nodeHandle.hasParam(generateSecondPointPercentageKey(servoKey)) ||
            !this->nodeHandle.hasParam(generateSecondPointAngleKey(servoKey)))
        {
            ROS_DEBUG("Calibration table: initialize parameter storage by default calibration points for '%s' key", servoKey.c_str());
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
    ROS_DEBUG("Calibration table: load calibration points by '%s' key", servoKey.c_str());

    CalibrationPoints calibrationPoints;
    if (this->nodeHandle.getParam(generateFirstPointPercentageKey(servoKey), calibrationPoints.firstPointSignalStrength) &&
        this->nodeHandle.getParam(generateFirstPointAngleKey(servoKey), calibrationPoints.firstPointRotateAngle) &&
        this->nodeHandle.getParam(generateSecondPointPercentageKey(servoKey), calibrationPoints.secondPointSignalStrength) &&
        this->nodeHandle.getParam(generateSecondPointAngleKey(servoKey), calibrationPoints.secondPointRotateAngle))
    {
        return calibrationPoints;
    }

    ROS_ERROR("Calibration table: can't load calibration points by '%s' key from ROS", servoKey.c_str());
    return CalibrationPoints{};
}

void servo::CalibrationTable::loadAllLimits()
{
    ROS_DEBUG("Calibration table: load all calibration limits");

    for (size_t i = 0; i < this->tableOutput.size(); ++i)
    {
        const auto servoKey = generateServoKey(i);
        if (!this->nodeHandle.hasParam(generateLowerLimitKey(servoKey)) ||
            !this->nodeHandle.hasParam(generateUpperLimitKey(servoKey)))
        {
            ROS_DEBUG("Calibration table: initialize parameter storage by default limits for '%s' key", servoKey.c_str());
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
        ROS_ERROR("Calibration table: can't load lower limit by '%s' key from ROS. Return %f%% value by default", 
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
    else
    {
        ROS_ERROR("Calibration table: can't load upper limit by key '%s'. Return %f%% value by default", 
            servoKey.c_str(), upperLimit);
    }

    return upperLimit;
}

void servo::CalibrationTable::notifyServoParameters(size_t index)
{
    if (this->tableListeners[index] == nullptr)
    {
        ROS_WARN("Calibration table: can't notify listener by index '%d' as it is not available", index);
        return;
    }

    this->tableListeners[index]->setParameters(tableOutput[index]);
}
