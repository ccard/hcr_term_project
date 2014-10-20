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
 * Auto-generated by genmsg_cpp from file /home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_service_pair_msgs/msg/TestiesPairResponse.msg
 *
 */


#ifndef ROCON_SERVICE_PAIR_MSGS_MESSAGE_TESTIESPAIRRESPONSE_H
#define ROCON_SERVICE_PAIR_MSGS_MESSAGE_TESTIESPAIRRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <uuid_msgs/UniqueID.h>
#include <rocon_service_pair_msgs/TestiesResponse.h>

namespace rocon_service_pair_msgs
{
template <class ContainerAllocator>
struct TestiesPairResponse_
{
  typedef TestiesPairResponse_<ContainerAllocator> Type;

  TestiesPairResponse_()
    : id()
    , response()  {
    }
  TestiesPairResponse_(const ContainerAllocator& _alloc)
    : id(_alloc)
    , response(_alloc)  {
    }



   typedef  ::uuid_msgs::UniqueID_<ContainerAllocator>  _id_type;
  _id_type id;

   typedef  ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator>  _response_type;
  _response_type response;




  typedef boost::shared_ptr< ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator> const> ConstPtr;

}; // struct TestiesPairResponse_

typedef ::rocon_service_pair_msgs::TestiesPairResponse_<std::allocator<void> > TestiesPairResponse;

typedef boost::shared_ptr< ::rocon_service_pair_msgs::TestiesPairResponse > TestiesPairResponsePtr;
typedef boost::shared_ptr< ::rocon_service_pair_msgs::TestiesPairResponse const> TestiesPairResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rocon_service_pair_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'rocon_service_pair_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_service_pair_msgs/msg'], 'uuid_msgs': ['/opt/ros/indigo/share/uuid_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "05404c9fe275eda57650fdfced8cf402";
  }

  static const char* value(const ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x05404c9fe275eda5ULL;
  static const uint64_t static_value2 = 0x7650fdfced8cf402ULL;
};

template<class ContainerAllocator>
struct DataType< ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rocon_service_pair_msgs/TestiesPairResponse";
  }

  static const char* value(const ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uuid_msgs/UniqueID id\n\
TestiesResponse response\n\
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
MSG: rocon_service_pair_msgs/TestiesResponse\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM A SERVICE PAIR DEFINITION ======\n\
uuid_msgs/UniqueID id\n\
string data\n\
";
  }

  static const char* value(const ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.response);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct TestiesPairResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rocon_service_pair_msgs::TestiesPairResponse_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    s << std::endl;
    Printer< ::uuid_msgs::UniqueID_<ContainerAllocator> >::stream(s, indent + "  ", v.id);
    s << indent << "response: ";
    s << std::endl;
    Printer< ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator> >::stream(s, indent + "  ", v.response);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROCON_SERVICE_PAIR_MSGS_MESSAGE_TESTIESPAIRRESPONSE_H
