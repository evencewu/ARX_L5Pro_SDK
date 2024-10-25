#pragma once

#include <cmath>
#include <iostream>
#include <vector>

namespace arx
{
    namespace solve
    {
        void ScurveInterpolation(double *get_pos, double *now_pos ,double *send_pos, double *send_vel, double max_acc, double max_vel, double frequency);
        double Linearinterpolation(double now_position, double target_position, double ramp_k, double frequency);
        std::vector<double> l5_get_Tau(double q_[6]);
    }
}
