#include "arx_l5pro_api/interfaces/InterfacesBase.hpp"
#include <mutex>

namespace arx
{
    class L5ProIneterfacesThread : public L5ProIneterfacesBase
    {
    public:
        L5ProIneterfacesThread(const std::string &can_name);

        std::vector<hw_interface::HybridJointStatus> getJointStatus();
        std::vector<double> getJointPositons();
        std::vector<double> getJointVelocities();
        Eigen::Isometry3d getEndPose();

        void setArmStatus(int state);
        void setEndPose(Eigen::Isometry3d input);
        void setJointPositons();

        // 指令
        Eigen::Isometry3d ee_command_safe_;

        // 状态
        std::vector<hw_interface::HybridJointStatus> joint_status_safe_{7};
        Eigen::Isometry3d ee_state_safe_;

    private:
        static void *ArmThreadWrapper(void *arg);
        void ArmThread();
        void exchangeData();

        void startThread();

        void StopThread();

        /*交换区数据*/

        // 指令
        Eigen::Isometry3d ee_command_exchange_;

        // 状态
        std::vector<hw_interface::HybridJointStatus> joint_status_exchange_{7};
        Eigen::Isometry3d ee_state_exchange_;
        
        /*多线程*/
    
        pthread_t thread_id;
        //多线程信号量
        std::atomic<bool> atomic_thread_stop_flag_;
        std::atomic<bool> atomic_thread_finish_flag_;

        std::mutex mutex_exchange_data_;
    };
}
