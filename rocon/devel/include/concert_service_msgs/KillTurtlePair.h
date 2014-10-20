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
 * Auto-generated by genmsg_cpp from file /home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/devel/share/concert_service_msgs/msg/KillTurtlePair.msg
 *
 */


#ifndef CONCERT_SERVICE_MSGS_MESSAGE_KILLTURTLEPAIR_H
#define CONCERT_SERVICE_MSGS_MESSAGE_KILLTURTLEPAIR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <concert_service_msgs/KillTurtlePairRequest.h>
#include <concert_service_msgs/KillTurtlePairResponse.h>

namespace concert_service_msgs
{
template <class ContainerAllocator>
struct KillTurtlePair_
{
  typedef KillTurtlePair_<ContainerAllocator> Type;

  KillTurtlePair_()
    : pair_request()
    , pair_response()  {
    }
  KillTurtlePair_(const ContainerAllocator& _alloc)
    : pair_request(_alloc)
    , pair_response(_alloc)  {
    }



   typedef  ::concert_service_msgs::KillTurtlePairRequest_<ContainerAllocator>  _pair_request_type;
  _pair_request_type pair_request;

   typedef  ::concert_service_msgs::KillTurtlePairResponse_<ContainerAllocator>  _pair_response_type;
  _pair_response_type pair_response;




  typedef boost::shared_ptr< ::concert_service_msgs::KillTurtlePair_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::concert_service_msgs::KillTurtlePair_<ContainerAllocator> const> ConstPtr;

}; // struct KillTurtlePair_

typedef ::concert_service_msgs::KillTurtlePair_<std::allocator<void> > KillTurtlePair;

typedef boost::shared_ptr< ::concert_service_msgs::KillTurtlePair > KillTurtlePairPtr;
typedef boost::shared_ptr< ::concert_service_msgs::KillTurtlePair const> KillTurtlePairConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::concert_service_msgs::KillTurtlePair_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::concert_service_msgs::KillTurtlePair_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::concert_service_msgs::KillTurtlePair_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::concert_service_msgs::KillTurtlePair_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::concert_service_msgs::KillTurtlePair_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::concert_service_msgs::KillTurtlePair_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::concert_service_msgs::KillTurtlePair_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::concert_service_msgs::KillTurtlePair_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::concert_service_msgs::KillTurtlePair_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d7f05d5e41a7cb8b3cbf2ace791fdbe4";
  }

  static const char* value(const ::concert_service_msgs::KillTurtlePair_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd7f05d5e41a7cb8bULL;
  static const uint64_t static_value2 = 0x3cbf2ace791fdbe4ULL;
};

template<class ContainerAllocator>
struct DataType< ::concert_service_msgs::KillTurtlePair_<ContainerAllocator> >
{
  static const char* value()
  {
    return "concert_service_msgs/KillTurtlePair";
  }

  static const char* value(const ::concert_service_msgs::KillTurtlePair_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::concert_service_msgs::KillTurtlePair_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM A SERVICE PAIR DEFINITION ======\n\
\n\
KillTurtlePairRequest pair_request\n\
KillTurtlePairResponse pair_response\n\
\n\
================================================================================\n\
MSG: concert_service_msgs/KillTurtlePairRequest\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM A SERVICE PAIR DEFINITION ======\n\
\n\
uuid_msgs/UniqueID id\n\
KillTurtleRequest request\n\
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
MSG: concert_service_msgs/KillTurtleRequest\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM A SERVICE PAIR DEFINITION ======\n\
\n\
string name\n\
\n\
================================================================================\n\
MSG: concert_service_msgs/KillTurtlePairResponse\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM A SERVICE PAIR DEFINITION ======\n\
\n\
uuid_msgs/UniqueID id\n\
KillTurtleResponse response\n\
\n\
================================================================================\n\
MSG: concert_service_msgs/KillTurtleResponse\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM A SERVICE PAIR DEFINITION ======\n\
\n\
\n\
";
  }

  static const char* value(const ::concert_service_msgs::KillTurtlePair_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::concert_service_msgs::KillTurtlePair_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pair_request);
      stream.next(m.pair_response);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct KillTurtlePair_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::concert_service_msgs::KillTurtlePair_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::concert_service_msgs::KillTurtlePair_<ContainerAllocator>& v)
  {
    s << indent << "pair_request: ";
    s << std::endl;
    Printer< ::concert_service_msgs::KillTurtlePairRequest_<ContainerAllocator> >::stream(s, indent + "  ", v.pair_request);
    s << indent << "pair_response: ";
    s << std::endl;
    Printer< ::concert_service_msgs::KillTurtlePairResponse_<ContainerAllocator> >::stream(s, indent + "  ", v.pair_response);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONCERT_SERVICE_MSGS_MESSAGE_KILLTURTLEPAIR_H
