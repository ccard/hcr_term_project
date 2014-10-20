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
 * Auto-generated by genmsg_cpp from file /home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/devel/share/concert_service_msgs/msg/CaptureResourceRequest.msg
 *
 */


#ifndef CONCERT_SERVICE_MSGS_MESSAGE_CAPTURERESOURCEREQUEST_H
#define CONCERT_SERVICE_MSGS_MESSAGE_CAPTURERESOURCEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace concert_service_msgs
{
template <class ContainerAllocator>
struct CaptureResourceRequest_
{
  typedef CaptureResourceRequest_<ContainerAllocator> Type;

  CaptureResourceRequest_()
    : rocon_uri()
    , release(false)  {
    }
  CaptureResourceRequest_(const ContainerAllocator& _alloc)
    : rocon_uri(_alloc)
    , release(false)  {
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _rocon_uri_type;
  _rocon_uri_type rocon_uri;

   typedef uint8_t _release_type;
  _release_type release;




  typedef boost::shared_ptr< ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator> const> ConstPtr;

}; // struct CaptureResourceRequest_

typedef ::concert_service_msgs::CaptureResourceRequest_<std::allocator<void> > CaptureResourceRequest;

typedef boost::shared_ptr< ::concert_service_msgs::CaptureResourceRequest > CaptureResourceRequestPtr;
typedef boost::shared_ptr< ::concert_service_msgs::CaptureResourceRequest const> CaptureResourceRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace concert_service_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'rocon_std_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_std_msgs/msg', '/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/devel/share/rocon_std_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'concert_service_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/devel/share/concert_service_msgs/msg'], 'rocon_service_pair_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_service_pair_msgs/msg'], 'uuid_msgs': ['/opt/ros/indigo/share/uuid_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9ef24df11d8c425fa509296ab551e5d4";
  }

  static const char* value(const ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9ef24df11d8c425fULL;
  static const uint64_t static_value2 = 0xa509296ab551e5d4ULL;
};

template<class ContainerAllocator>
struct DataType< ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "concert_service_msgs/CaptureResourceRequest";
  }

  static const char* value(const ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM A SERVICE PAIR DEFINITION ======\n\
\n\
# Used by turtle_concert/teleop service to handle requests to capture\n\
# teleopable robots.\n\
\n\
# Usually the rocon uri provided to the remocon in the list of available\n\
# teleopable robots.\n\
string rocon_uri\n\
# Capture or release the robot - pythonic default is False (i.e. capture) \n\
bool release\n\
";
  }

  static const char* value(const ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.rocon_uri);
      stream.next(m.release);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct CaptureResourceRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::concert_service_msgs::CaptureResourceRequest_<ContainerAllocator>& v)
  {
    s << indent << "rocon_uri: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.rocon_uri);
    s << indent << "release: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.release);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONCERT_SERVICE_MSGS_MESSAGE_CAPTURERESOURCEREQUEST_H
