#include <socket_can_channel.hpp>

namespace can
{
SocketCANChannel::SocketCANChannel(std::string yaml_file_name) : BaseCanChannel(yaml_file_name)
{

	can_name_ = can_yaml_["can_name"].as<std::string>();
	connected_ = false;
	timeout_ = can_yaml_["time_out"].as<int>();
	init();
}

SocketCANChannel::~SocketCANChannel()
{
	if (close(socket_) < 0)
	{
		perror("Closing: ");
		printf("Error: %d", errno);
	}
}

void SocketCANChannel::init()
{
	if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		perror("Error while opening socket");
		return;
	}

	struct ifreq ifr
	{
	};
	strcpy(ifr.ifr_name, can_name_.c_str());
	ioctl(socket_, SIOCGIFINDEX, &ifr);

	struct sockaddr_can addr
	{
	};
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	printf("%s at index %d\n", can_name_.c_str(), ifr.ifr_ifindex);

	if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		perror("Error in socket bind");
		return;
	}

	int error = 0;
	socklen_t len = sizeof(error);
	int retval = getsockopt(socket_, SOL_SOCKET, SO_ERROR, &error, &len);
	if (retval != 0)
	{
		/* there was a problem getting the error code */
		printf("Error getting socket error code: %s\n", strerror(retval));
		return;
	}

	if (error != 0)
	{
		/* socket has a non zero error status */
		printf("Socket error: %s\n", strerror(error));
		return;
	}

	struct timeval timeout
	{
	};
	timeout.tv_sec = (timeout_ / 1000);
	timeout.tv_usec = (timeout_ % 1000) * 1000;

	if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
	{
		perror("Setting timeout failed");
	}

	connected_ = true;
}

bool SocketCANChannel::is_connected()
{
	return connected_;
}

void SocketCANChannel::register_asyn_read_cb(std::function<void(can_frame &frame)> fun)
{
	cb_fun_vector_.push_back(fun);
}

void SocketCANChannel::start_asyn_read()
{
	t_ptr_ = std::make_shared<std::thread>(&SocketCANChannel::asyn_read, this);
	t_ptr_->detach();
}
void SocketCANChannel::asyn_read()
{
	while (1)
	{
		struct can_frame frame = {0};
		int nbytes = read(socket_, &frame, sizeof(frame));
		if (nbytes < 0)
		{
			printf("receive buff is empty\n");
		}
		else if (nbytes == 16)
		{
			/*printf("message received\n");
			printf("the nbytes:%d\n", nbytes);
			printf("length:%d", sizeof(frame));
			printf("ID=0x%X DLC=%d\n", frame.can_id, frame.can_dlc);
			printf("data0=0x%02x\n", frame.data[0]);
			printf("data1=0x%02x\n", frame.data[1]);
			printf("data2=0x%02x\n", frame.data[2]);
			printf("data3=0x%02x\n", frame.data[3]);
			printf("data4=0x%02x\n", frame.data[4]);
			printf("data5=0x%02x\n", frame.data[5]);
			printf("data6=0x%02x\n", frame.data[6]);
			printf("data7=0x%02x\n", frame.data[7]);*/
			for (int i = 0; i < cb_fun_vector_.size(); i++)
			{
				cb_fun_vector_[i](frame);
			}
		}
		else
		{
			printf("fatal error ,message received failed\n");
		}
	}
}
bool SocketCANChannel::write_frame(can_frame &frame)
{
	auto num_bytes = write(socket_, &frame, sizeof(frame));
	return num_bytes > 0;
}
} // namespace can