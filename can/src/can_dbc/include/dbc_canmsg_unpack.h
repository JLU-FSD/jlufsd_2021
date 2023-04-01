

#ifndef _DBC_CANMSG_UNPACK_H_
#define _DBC_CANMSG_UNPACK_H_

#include <map>
#include <string>
#include <string.h>
#include "canmsg_define.h"
#include <linux/can.h>	   //use linux can
#include <linux/can/raw.h> //use linux can
#include <vector>

namespace can_util
{

/** the dbc unpack used to transform the can msg to value
* @param in_ [struct Message] the message that you want to unpack
* @param in_ [struct Canmsg] the canmsg
* @param in_ [unsigned int] the size of the value list
* @param out [double array] the array of the values
* @return [int] stat
*/
int unpackCanmsg(const Message &m, const Canmsg &msg, const size_t valueSize, double *value);
int unpackCanmsg(const Message &m, const can_frame &msg, const size_t valueSize, double *value);
int unpackCanmsg(const Message &m, const can_frame &msg, std::vector<double> &signals_value);

/**
* pack one Signal
* @param in_ [struct Signal] the Signal that you want to unpack
* @param in_ [unsigned char array] the data array of the can msg
* @return [double] the value that you unpack from the Signal
*/
real64_T unpackSignal(const Signal &s, const uint8_T *data);

} // namespace can_util

#endif //_DBC_CANMSG_UNPACK_H_
