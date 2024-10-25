#pragma once

#include "arx_hardware_interface/canbase/CanBaseDef.hpp"

namespace arx
{
    namespace hw_interface
    {
        class DlcBase
        {
        public:
            virtual void CanAnalyze(CanFrame *frame) = 0;
        };
    }
}