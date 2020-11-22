#pragma once

namespace servo
{
    /*
     * \brief Servo signaling channel
     */
    class ISignalingChannel
    {
    public:
        virtual ~ISignalingChannel() = default;

    public:
        /*
         * \brief Set signal strength in percentage
         * \note Method is blocking
         */
        virtual bool setStrength(double strengthInPercentage) = 0;
    };
}
