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
 * Auto-generated by genmsg_cpp from file /home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_std_msgs/srv/GetPlatformInfo.srv
 *
 */


#ifndef ROCON_STD_MSGS_MESSAGE_GETPLATFORMINFOREQUEST_H
#define ROCON_STD_MSGS_MESSAGE_GETPLATFORMINFOREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rocon_std_msgs
{
template <class ContainerAllocator>
struct GetPlatformInfoRequest_
{
  typedef GetPlatformInfoRequest_<ContainerAllocator> Type;

  GetPlatformInfoRequest_()
    {
    }
  GetPlatformInfoRequest_(const ContainerAllocator& _alloc)
    {
    }






  typedef boost::shared_ptr< ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetPlatformInfoRequest_

typedef ::rocon_std_msgs::GetPlatformInfoRequest_<std::allocator<void> > GetPlatformInfoRequest;

typedef boost::shared_ptr< ::rocon_std_msgs::GetPlatformInfoRequest > GetPlatformInfoRequestPtr;
typedef boost::shared_ptr< ::rocon_std_msgs::GetPlatformInfoRequest const> GetPlatformInfoRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rocon_std_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'rocon_std_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_std_msgs/msg', '/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/devel/share/rocon_std_msgs/msg'], 'rocon_service_pair_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_service_pair_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'uuid_msgs': ['/opt/ros/indigo/share/uuid_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rocon_std_msgs/GetPlatformInfoRequest";
  }

  static const char* value(const ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
";
  }

  static const char* value(const ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct GetPlatformInfoRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::rocon_std_msgs::GetPlatformInfoRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // ROCON_STD_MSGS_MESSAGE_GETPLATFORMINFOREQUEST_H
