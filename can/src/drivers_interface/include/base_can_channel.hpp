#pragma once
#include <cstdint>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <thread>
#include <functional>
#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include "can_config_data_path.h"
namespace can
{
    class BaseCanChannel
    {
    public:
        BaseCanChannel(std::string yaml_path_name)
        {
            can_yaml_ = YAML::LoadFile(yaml_path_name); //也可以这样读取文件
        }
        virtual bool is_connected() = 0;
        virtual void start_asyn_read() = 0;
        virtual void register_asyn_read_cb(std::function<void(can_frame &frame)> fun) = 0;
        virtual bool write_frame(can_frame &frame) = 0;

    protected:
        virtual void asyn_read() = 0;
        std::string can_name_;
        bool connected_ = false;
        long timeout_;
        std::shared_ptr<std::thread> t_ptr_;
        std::vector<std::function<void(can_frame &frame)>> cb_fun_vector_;
        YAML::Node can_yaml_;
    };
} // namespace can
