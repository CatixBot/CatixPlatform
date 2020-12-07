#pragma once

#include <memory>

//---------------------------------------------------------------------

namespace servo
{
    class ISignalingChannel;
}

//---------------------------------------------------------------------

namespace servo
{
    /*
     * \brief Provide channels of physical device
     */
    class ISignalingController
    {
    public:
        virtual ~ISignalingController() = default;

    public:
        /*
         * \brief Make new signaling channel
         * \note Method is blocking
         */
        virtual std::unique_ptr<ISignalingChannel> makeSignalingChannel(uint8_t channelNumber) = 0;

        /*
         * \brief Drop signal strength for each channel
         */
        virtual void dropAllChannels() = 0;
    };
}