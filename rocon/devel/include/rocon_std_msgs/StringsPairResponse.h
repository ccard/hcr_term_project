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
 * Auto-generated by genmsg_cpp from file /home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/devel/share/rocon_std_msgs/msg/StringsPairResponse.msg
 *
 */


#ifndef ROCON_STD_MSGS_MESSAGE_STRINGSPAIRRESPONSE_H
#define ROCON_STD_MSGS_MESSAGE_STRINGSPAIRRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <uuid_msgs/UniqueID.h>
#include <rocon_std_msgs/StringsResponse.h>

namespace rocon_std_msgs
{
template <class ContainerAllocator>
struct StringsPairResponse_
{
  typedef StringsPairResponse_<ContainerAllocator> Type;

  StringsPairResponse_()
    : id()
    , response()  {
    }
  StringsPairResponse_(const ContainerAllocator& _alloc)
    : id(_alloc)
    , response(_alloc)  {
    }



   typedef  ::uuid_msgs::UniqueID_<ContainerAllocator>  _id_type;
  _id_type id;

   typedef  ::rocon_std_msgs::StringsResponse_<ContainerAllocator>  _response_type;
  _response_type response;




  typedef boost::shared_ptr< ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator> const> ConstPtr;

}; // struct StringsPairResponse_

typedef ::rocon_std_msgs::StringsPairResponse_<std::allocator<void> > StringsPairResponse;

typedef boost::shared_ptr< ::rocon_std_msgs::StringsPairResponse > StringsPairResponsePtr;
typedef boost::shared_ptr< ::rocon_std_msgs::StringsPairResponse const> StringsPairResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rocon_std_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'rocon_std_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_std_msgs/msg', '/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/devel/share/rocon_std_msgs/msg'], 'rocon_service_pair_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_service_pair_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'uuid_msgs': ['/opt/ros/indigo/share/uuid_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7b20492548347a7692aa8c5680af8d1b";
  }

  static const char* value(const ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7b20492548347a76ULL;
  static const uint64_t static_value2 = 0x92aa8c5680af8d1bULL;
};

template<class ContainerAllocator>
struct DataType< ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rocon_std_msgs/StringsPairResponse";
  }

  static const char* value(const ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM A SERVICE PAIR DEFINITION ======\n\
\n\
uuid_msgs/UniqueID id\n\
StringsResponse response\n\
\n\
================================================================================\n\
MSG: uuid_msgs/UniqueID\n\
# A universally unique identifier (UUID).\n\
#\n\
#  http://en.wikipedia.org/wiki/Universally_unique_identifier\n\
#  http://tools.ietf.org/html/rfc4122.html\n\
\n\
uint8[16] uuid\n\
\n\
================================================================================\n\
MSG: rocon_std_msgs/StringsResponse\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM A SERVICE PAIR DEFINITION ======\n\
\n\
string data\n\
\n\
";
  }

  static const char* value(const ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.response);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct StringsPairResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rocon_std_msgs::StringsPairResponse_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    s << std::endl;
    Printer< ::uuid_msgs::UniqueID_<ContainerAllocator> >::stream(s, indent + "  ", v.id);
    s << indent << "response: ";
    s << std::endl;
    Printer< ::rocon_std_msgs::StringsResponse_<ContainerAllocator> >::stream(s, indent + "  ", v.response);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROCON_STD_MSGS_MESSAGE_STRINGSPAIRRESPONSE_H
