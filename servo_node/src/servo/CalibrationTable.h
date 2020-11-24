#pragma once

#include "servo/ServoParameters.h"

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

        bool setFirstPoint(size_t index, double pulseWidth, double rotateAngle);
        bool setSecondPoint(size_t index, double pulseWidth, double rotateAngle);
        bool setLowerLimit(size_t index, double pulseWidth);
        bool setUpperLimit(size_t index, double pulseWidth);

        bool isCalibrationRecordProvided();
        servo::ServoParameters getCalibrationRecord();

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

    private:
        ros::NodeHandle &nodeHandle;

        std::vector<servo::ServoParameters> tableOutput;
        std::vector<CalibrationPoints> tableInput;
    };
}
