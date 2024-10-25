#pragma once
#include "arx_hardware_interface/canbase/CanBaseDef.hpp"
#include "arx_hardware_interface/canbase/SocketCan.hpp"
#include "arx_hardware_interface/can_motor_dlc/MotorBase.hpp"
#include "arx_hardware_interface/can_motor_dlc/Mtp1Dlc.hpp"
#include "arx_hardware_interface/can_motor_dlc/Mtp2Dlc.hpp"

#include "arx_l5pro_api/solver/solver.hpp"

#include <arx_l5pro_api/Eigen/Dense>
#include <arx_l5pro_api/Eigen/Geometry> // 包含变换和位姿处理

#include <vector>
#include <memory>
#include <chrono>
#include <utility>
#include <cmath>

namespace arx
{
    class L5ProIneterfacesBase
    {
    public:
        L5ProIneterfacesBase() = default;

        L5ProIneterfacesBase(const std::string &can_name);

        ~L5ProIneterfacesBase();

        void Init();

        void Update();
        void loadJointStatus();

        // 如果有跨线程取用数据的需求，应重写以下get函数使其指向非共享参数
        std::vector<hw_interface::HybridJointStatus> getJointStatus();
        std::vector<double> getJointPositons();
        std::vector<double> getJointVelocities();
        Eigen::Isometry3d getEndPose();

        void setJointPositions();
        void setJointPelocities();
        void setEndPose();

        void setClampPos();
        void setEnableMotor();
        void setDisableMotor();

        void ResetClampZeroPoint();

        // can
        void ExchangeMotorMsg();
        void CanCallback(hw_interface::CanFrame *frame);

        /// @brief 状态枚举
        enum state
        {
            SOFT,
            GO_HOME,
            PROTECT,
            POSITION,
            G_COMPENSATION,
            END_CONTROL
        };

    protected:
        std::atomic<int> state_machine_;

        /// @brief 关节零点
        std::vector<double> zero_point_position_ = {0, 0, 0, 0, 0, 0, 0};

        /// @brief 关节状态
        std::vector<hw_interface::HybridJointStatus> joint_status_{7};

        Eigen::Isometry3d ee_command_;
        Eigen::Isometry3d ee_state_;

        std::string can_name_;

    private:
        template <typename T>
        T limit(const T &v, const T &lb, const T &ub)
        {
            if (v < lb)
            {
                return lb;
            }
            else if (v > ub)
            {
                return ub;
            }
            else
            {
                return v;
            }
        }

        //
        double lower_Joint[7] = {-3.14, -0.1, -0.1, -1.571, -1.571, -6.28, 0};
        double upper_Joint[7] = {2.618, 3.6, 3.14, 1.4, 1.571, 6.28, 5};

        double limit(double input, double max, double min);

        // hw interfaces
        hw_interface::SocketCan arx_can;
        std::vector<std::shared_ptr<hw_interface::MotorBase>> joint_ptr_ =
            {
                std::make_shared<hw_interface::MotorType1>(1),
                std::make_shared<hw_interface::MotorType1>(2),
                std::make_shared<hw_interface::MotorType1>(4),
                std::make_shared<hw_interface::MotorType2>(5),
                std::make_shared<hw_interface::MotorType2>(6),
                std::make_shared<hw_interface::MotorType2>(7),
                std::make_shared<hw_interface::MotorType2>(8),
        };

        /// @brief 重力补偿数值
        std::vector<double> gravity_compensation_torqe_ = {0, 0, 0, 0, 0, 0};

        void stateGoHome();
        void stateSoft();
        void stateProtect();
        void stateGravityCompensation();
        void stateEndControl();

        std::stringstream getLog();

        double frequency_ = 200;
    };
}