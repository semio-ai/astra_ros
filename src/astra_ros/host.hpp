#ifndef _ASTRA_ROS_HOST_HPP_
#define _ASTRA_ROS_HOST_HPP_

// Endianess
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define ASTRA_ROS_LITTLE_ENDIAN
#else
#define ASTRA_ROS_BIG_ENDIAN
#endif

#endif