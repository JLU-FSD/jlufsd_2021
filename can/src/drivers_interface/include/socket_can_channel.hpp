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
#include "base_can_channel.hpp"

namespace can
{
    class SocketCANChannel : public BaseCanChannel
    {
    public:
        SocketCANChannel(std::string yaml_path_name);
        ~SocketCANChannel();
        virtual bool is_connected();
        virtual void start_asyn_read();
        virtual void register_asyn_read_cb(std::function<void(can_frame &frame)> fun);
        virtual bool write_frame(can_frame &frame);

    private:
        virtual void asyn_read();
        void init();
        int socket_;
    };
} // namespace can

//SOCKET_CAN_SOCKET_CAN_HPP
//std::function<void(void)> ff2 = std::bind(&Foo::f1, &foo);