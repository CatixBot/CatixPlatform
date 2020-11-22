#pragma once

#include "servo/ServoParameters.h"

#include <vector>

namespace servo
{
    class CalibrationTable
    {
    public:
        CalibrationTable(size_t tableSize);

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
            double firstPointPulseWidth = -1.0;
            double firstPointRotateAngle = 0.0;
            double secondPointPulseWidth = -1.0;
            double secondPointRotateAngle = 0.0;
        };

    private:
        bool isCalibrationPointsCorrect(const CalibrationPoints &calibrationPoints);
        void calculateCalibration(size_t index, const CalibrationPoints &calibrationPoints);

    private:
        std::vector<servo::ServoParameters> tableOutput;
        std::vector<CalibrationPoints> tableInput;
    };
}