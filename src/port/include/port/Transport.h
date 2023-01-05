#pragma once
#include "port/WMJProtocol.h"

namespace wmj
{
    class Port
    {
    public:
        Port() {}
        virtual ~Port() {}
        virtual bool sendFrame(Buffer &) = 0;
        virtual Buffer readFrame(int) = 0;
        static void checkPort();
        bool canUseThisPort = true;
        bool m_clear_shoot_speed = false;

    private:
    };
}
