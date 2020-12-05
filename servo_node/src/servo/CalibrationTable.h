#pragma once

#include "servo/ServoParameters.h"
#include "servo/ServoInterface.h"

#include <ros/ros.h>
#include <vector>

namespace servo
{
    class CalibrationTable
    {
    public:
        CalibrationTable(ros::NodeHandle &nodeHandle, size_t tableSize);

    public:
        void resetPoints(size_t index);

        bool setFirstPoint(size_t index, double signalStrength, double rotateAngle);
        bool setSecondPoint(size_t index, double signalStrength, double rotateAngle);
        bool setLowerLimit(size_t index, double signalStrength);
        bool setUpperLimit(size_t index, double signalStrength);

        void subscribeListener(size_t index, std::shared_ptr<servo::IServo> servo);
        void updateListeners();

    private:
        struct CalibrationPoints
        {
            double firstPointSignalStrength = -1.0;
            double firstPointRotateAngle = 0.0;
            double secondPointSignalStrength = -1.0;
            double secondPointRotateAngle = 0.0;
        };

    private:
        bool isCalibrationPointsCorrect(const CalibrationPoints &calibrationPoints);
        bool calculateIndexCalibration(size_t index, const CalibrationPoints &calibrationPoints);
        bool calculateAllCalibrations();

        void loadAllCalibrationPoints();
        void storeCalibrationPoints(std::string servoKey, const CalibrationPoints& calibrationPoints);
        CalibrationPoints loadCalibrationPoints(std::string servoKey);

        void loadAllLimits();
        void storeLowerLimit(std::string servoKey, double lowerLimit);
        double loadLowerLimit(std::string servoKey);
        void storeUpperLimit(std::string servoKey, double upperLimit);
        double loadUpperLimit(std::string servoKey);

        void notifyServoParameters(size_t index);

    private:
        ros::NodeHandle &nodeHandle;

        std::vector<std::shared_ptr<servo::IServo>> tableListeners;
        std::vector<servo::ServoParameters> tableOutput;
        std::vector<CalibrationPoints> tableInput;
    };
}
