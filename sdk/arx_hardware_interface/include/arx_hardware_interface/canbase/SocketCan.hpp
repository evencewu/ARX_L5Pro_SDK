 #pragma once

#include "arx_hardware_interface/canbase/CanBaseDef.hpp"
#include "arx_hardware_interface/typedef/HybridJointTypeDef.hpp"

// #include "CANAdapter.h"
#include <stdbool.h>
#include <net/if.h>
#include <pthread.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>

#include <linux/can.h>

#include <sys/select.h>
#include <sys/socket.h>
// SIOCGIFINDEX
#include <sys/ioctl.h>

#include <mutex>
#include <memory>
#include <functional>
#include <string.h>

#include <atomic>

namespace arx
{
    namespace hw_interface
    {
        typedef enum
        {
            ADAPTER_NONE,
            ADAPTER_SOCKETCAN,
            ADAPTER_SLCAN,
            ADAPTER_LOGFILE,
        } can_adapter_t;

        typedef struct ifreq interface_request_t;
        typedef struct sockaddr_can can_socket_address_t;
        typedef void (*reception_handler_t)(CanFrame *, void *);

        /// @brief 继承CanBase并使用SocketCan进行数据通信
        class SocketCan : public CanBase
        {
        public:
            SocketCan();
            ~SocketCan();

            bool Init(const char *interface) override;
            bool ExchangeData(CanFrame *frame) override;
            bool ReadData();
            bool WriteData();
            bool IsOpen();

            void setCallBackFunction(const std::function<void(CanFrame *frame)> func);
            void setGetMsgContentFunction(const std::function<void()> func);

            CanFrame GetCanFrame();
            void LoadMutexMsg();

            //线程间数据交换

        protected:
            // LOGIC
            void Close();

            // SOCKET CAN
            can_adapter_t adapter_type;

            std::atomic<bool> terminate_receiver_thread_;
            std::atomic<bool> receiver_thread_running_; 

            int sockfd = -1;

            interface_request_t if_request;
            can_socket_address_t addr;

            // 线程管理
            pthread_t thread_id;

            void startThread();
            void ReceiveThread();

            static void *ReciveThreadWrapper(void *arg)
            {
                // 将传递的void*指针转换为类的实例
                SocketCan *instance = static_cast<SocketCan *>(arg);
                // 调用实际的成员方法
                instance->ReceiveThread();
                return nullptr;
            }

            std::function<void(CanFrame *frame)> CallBack;
            std::function<void()> GetMsgContent;
            
            CanFrame rx_frame_;

            /// @brief 读取帧信息时使用的互斥锁
            std::mutex mutex_frame_;
            std::mutex mutex_get_data_;

            //std::shared_ptr<DlcBase> Analyze;
        };
    }
}