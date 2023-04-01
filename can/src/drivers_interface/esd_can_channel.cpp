#include <esd_can_channel.hpp>

namespace can
{
    EsdCanChannel::EsdCanChannel(std::string yaml_path_name) : BaseCanChannel(yaml_path_name)
    {
        init();
    }

    EsdCanChannel::~EsdCanChannel()
    {
        int retvalue = canClose(can_handler_);
        if (retvalue != NTCAN_SUCCESS)
            printf("canClose failed with error %d!\n", retvalue);
        else
            printf("canClose() returned OK !\n");
    }

    void EsdCanChannel::init()
    {

        //open can
        int retvalue = canOpen(can_yaml_["can_channel"].as<int>(),
                               0,
                               can_yaml_["txqueuesize"].as<int>(),
                               can_yaml_["rxqueuesize"].as<int>(),
                               can_yaml_["tx_timeout"].as<int>(),
                               can_yaml_["rx_timeout"].as<int>(),
                               &can_handler_);
        CAN_IF_STATUS cstat;
        canStatus(can_handler_,&cstat);
        std::cout<<std::hex<<cstat.features<<std::endl;
        if (retvalue != NTCAN_SUCCESS)
        {
            printf("canOpen() failed with error %d!\n", retvalue);
            return;
        }
        printf("function canOpen() returned OK !\n");
        retvalue = canSetBaudrate(can_handler_, can_yaml_["baud"].as<unsigned int>());
        if (retvalue != 0)
        {
            printf("canSetBaudrate() failed with error %d!\n", retvalue);
            canClose(can_handler_);
            return;
        }
        printf("function canSetBaudrate() returned OK !\n");
        connected_ = true;
    }
    bool EsdCanChannel::add_ids(std::vector<unsigned int> &ids_vec)
    {       
        for (int i = 0; i < ids_vec.size(); i++)
        {
            if(ids_vec[i]<=0x7FFF)
            {
                int retvalue = canIdAdd(can_handler_, ids_vec[i]);         
                if(retvalue != NTCAN_SUCCESS)
                {
                    std::cout<<"canIdAdd failed:0x"<<std::hex<<ids_vec[i]<<std::endl;
                    printf("canIdAdd() failed with error %d!\n", retvalue);
                    return false;
                }
            }
            else if(ids_vec[i]<=0x3FFFFFFF)
            {
                int count = 1;
                int retvalue = canIdRegionAdd(can_handler_, ids_vec[i]|NTCAN_20B_BASE,&count); /* Enable CAN-ID 0 */
                if (retvalue != NTCAN_SUCCESS)
                {
                    std::cout<<"canIdAdd failed:0x"<<std::hex<<ids_vec[i]<<std::endl;
                    printf("canIdAdd() failed with error %d!\n", retvalue);
                    return false;
                }
            }
            else
            {
                std::cout<<"\033[31m canIdAdd failed, id out of 29 bits,id is:0x \033[0m"
                <<std::hex<<ids_vec[i]<<std::endl;
            }     
        }
            printf("function canIdAdd() returned OK !\n");
            
    }
    bool EsdCanChannel::is_connected()
    {
        return connected_;
    }

    void EsdCanChannel::register_asyn_read_cb(std::function<void(can_frame &frame)> fun)
    {
        cb_fun_vector_.push_back(fun);
    }

    void EsdCanChannel::start_asyn_read()
    {
        t_ptr_ = std::make_shared<std::thread>(&EsdCanChannel::asyn_read, this);
        t_ptr_->detach();
    }
    void EsdCanChannel::asyn_read()
    {        
        do
        {   
            
            CMSG cmsg;
            int len = 1;
            int retvalue = canRead(can_handler_, &cmsg, &len, NULL);
            if (retvalue == NTCAN_RX_TIMEOUT)
            {
                printf("canRead() returned timeout\n");
                continue;
            }
            else if (retvalue != NTCAN_SUCCESS)
            {
                printf("canRead() failed with error %d!\n", retvalue);
            }
            else 
            {                
                can_frame temp_msg;
                temp_msg.can_dlc = cmsg.len;
                temp_msg.can_id = cmsg.id & (NTCAN_20B_BASE -1);
                for (int i = 0; i < (cmsg.len); i++)
                {
                    temp_msg.data[i]=cmsg.data[i];
                }
                for (int i = 0; i < cb_fun_vector_.size(); i++)
			    {
				    cb_fun_vector_[i](temp_msg);
			    }
                if(temp_msg.can_id ==0x403)
                {
                    printf("function canRead() returned OK !\n");
                    printf("Id of received message :%x!\n", cmsg.id);
                    printf("Len of received message :%d!\n", (cmsg.len));
                    for (int i = 0; i < (cmsg.len & 0x0f); i++)
                        printf("Byte %d of received message :%x!\n", i, cmsg.data[i]);
                }                
                
            }
            
        } while (1);
    }

    bool EsdCanChannel::write_frame(can_frame &frame)
    {
        CMSG cmsg[8];
        cmsg[0].id = frame.can_id;
        cmsg[0].len = frame.can_dlc;
        for (int i = 0; i < frame.can_dlc; i++)
            cmsg[0].data[i] = frame.data[i];
        int num_frames = 1;
        int retvalue =
            /* Number of valid messages in cmsg buffer*/
            canWrite(can_handler_, &cmsg[0], &num_frames, NULL);
        if (retvalue != NTCAN_SUCCESS)
            printf("canWrite failed() with error %d!\n", retvalue);
        else
            printf("function canWrite() returned OK !\n");
        /* ############################################################### */
    }

} // namespace can
