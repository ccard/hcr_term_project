/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/gateway_msgs/msg/ErrorCodes.msg
 *
 */


#ifndef GATEWAY_MSGS_MESSAGE_ERRORCODES_H
#define GATEWAY_MSGS_MESSAGE_ERRORCODES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace gateway_msgs
{
template <class ContainerAllocator>
struct ErrorCodes_
{
  typedef ErrorCodes_<ContainerAllocator> Type;

  ErrorCodes_()
    {
    }
  ErrorCodes_(const ContainerAllocator& _alloc)
    {
    }




    enum { SUCCESS = 0 };
     enum { NO_HUB_CONNECTION = 1 };
     enum { HUB_CONNECTION_ALREADY_EXISTS = 11 };
     enum { HUB_CONNECTION_UNRESOLVABLE = 12 };
     enum { HUB_CONNECTION_BLACKLISTED = 13 };
     enum { HUB_CONNECTION_FAILED = 14 };
     enum { HUB_CONNECTION_NOT_IN_NONEMPTY_WHITELIST = 15 };
     enum { HUB_NAME_NOT_FOUND = 16 };
     enum { HUB_CONNECTION_LOST = 17 };
     enum { HUB_UNKNOWN_ERROR = 19 };
     enum { FLIP_RULE_ALREADY_EXISTS = 22 };
     enum { FLIP_PATTERN_ALREDY_EXISTS = 23 };
     enum { FLIP_REMOTE_GATEWAY_FIREWALLING = 24 };
     enum { ADVERTISEMENT_EXISTS = 31 };
     enum { ADVERTISEMENT_NOT_FOUND = 32 };
     enum { UNKNOWN_ADVERTISEMENT_ERROR = 39 };
     enum { PULL_RULE_ALREADY_EXISTS = 41 };
     enum { REMOTE_GATEWAY_NOT_VISIBLE = 51 };
     enum { REMOTE_GATEWAY_SELF_IS_NOT = 52 };
     enum { REMOTE_GATEWAY_TARGET_HAS_MULTIPLE_MATCHES = 53 };
 

  typedef boost::shared_ptr< ::gateway_msgs::ErrorCodes_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gateway_msgs::ErrorCodes_<ContainerAllocator> const> ConstPtr;

}; // struct ErrorCodes_

typedef ::gateway_msgs::ErrorCodes_<std::allocator<void> > ErrorCodes;

typedef boost::shared_ptr< ::gateway_msgs::ErrorCodes > ErrorCodesPtr;
typedef boost::shared_ptr< ::gateway_msgs::ErrorCodes const> ErrorCodesConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gateway_msgs::ErrorCodes_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gateway_msgs::ErrorCodes_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace gateway_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'gateway_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/gateway_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::gateway_msgs::ErrorCodes_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gateway_msgs::ErrorCodes_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gateway_msgs::ErrorCodes_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gateway_msgs::ErrorCodes_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gateway_msgs::ErrorCodes_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gateway_msgs::ErrorCodes_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gateway_msgs::ErrorCodes_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cc2f69c68d11f625f99f07d82c572d47";
  }

  static const char* value(const ::gateway_msgs::ErrorCodes_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcc2f69c68d11f625ULL;
  static const uint64_t static_value2 = 0xf99f07d82c572d47ULL;
};

template<class ContainerAllocator>
struct DataType< ::gateway_msgs::ErrorCodes_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gateway_msgs/ErrorCodes";
  }

  static const char* value(const ::gateway_msgs::ErrorCodes_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gateway_msgs::ErrorCodes_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Error types for the gateway ros api\n\
\n\
# General\n\
int8 SUCCESS = 0\n\
int8 NO_HUB_CONNECTION = 1\n\
\n\
# Hub\n\
int8 HUB_CONNECTION_ALREADY_EXISTS = 11\n\
int8 HUB_CONNECTION_UNRESOLVABLE = 12\n\
int8 HUB_CONNECTION_BLACKLISTED = 13\n\
int8 HUB_CONNECTION_FAILED = 14\n\
int8 HUB_CONNECTION_NOT_IN_NONEMPTY_WHITELIST = 15\n\
int8 HUB_NAME_NOT_FOUND = 16\n\
int8 HUB_CONNECTION_LOST = 17\n\
int8 HUB_UNKNOWN_ERROR = 19\n\
\n\
# Flipping\n\
int8 FLIP_RULE_ALREADY_EXISTS = 22\n\
int8 FLIP_PATTERN_ALREDY_EXISTS = 23\n\
int8 FLIP_REMOTE_GATEWAY_FIREWALLING = 24\n\
\n\
# Advertise\n\
int8 ADVERTISEMENT_EXISTS = 31\n\
int8 ADVERTISEMENT_NOT_FOUND = 32\n\
int8 UNKNOWN_ADVERTISEMENT_ERROR = 39\n\
\n\
# Pulling\n\
int8 PULL_RULE_ALREADY_EXISTS = 41\n\
\n\
# Remotes\n\
int8 REMOTE_GATEWAY_NOT_VISIBLE = 51\n\
int8 REMOTE_GATEWAY_SELF_IS_NOT = 52\n\
int8 REMOTE_GATEWAY_TARGET_HAS_MULTIPLE_MATCHES = 53\n\
";
  }

  static const char* value(const ::gateway_msgs::ErrorCodes_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gateway_msgs::ErrorCodes_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct ErrorCodes_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gateway_msgs::ErrorCodes_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::gateway_msgs::ErrorCodes_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // GATEWAY_MSGS_MESSAGE_ERRORCODES_H
