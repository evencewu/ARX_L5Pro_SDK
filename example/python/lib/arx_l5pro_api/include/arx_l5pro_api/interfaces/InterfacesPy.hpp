#pragma once

#include "arx_l5pro_api/interfaces/InterfacesThread.hpp"

namespace arx
{
    class L5ProInterfacesPy : public L5ProIneterfacesThread
    {
    public:
        bool get_joint_names();//useless
        bool go_home();

        bool set_joint_positions();//useless
        bool set_joint_velocities();//useless
        bool set_ee_pose(std::vector<double> pose);
        
        void set_arm_state_g();

        std::vector<double> get_joint_positions();
        std::vector<double> get_joint_velocities();
        std::vector<double> get_ee_pose();
    };
}

