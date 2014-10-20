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
 * Auto-generated by genmsg_cpp from file /home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_interaction_msgs/srv/RequestInteraction.srv
 *
 */


#ifndef ROCON_INTERACTION_MSGS_MESSAGE_REQUESTINTERACTIONRESPONSE_H
#define ROCON_INTERACTION_MSGS_MESSAGE_REQUESTINTERACTIONRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rocon_interaction_msgs
{
template <class ContainerAllocator>
struct RequestInteractionResponse_
{
  typedef RequestInteractionResponse_<ContainerAllocator> Type;

  RequestInteractionResponse_()
    : result(false)
    , error_code(0)
    , message()  {
    }
  RequestInteractionResponse_(const ContainerAllocator& _alloc)
    : result(false)
    , error_code(0)
    , message(_alloc)  {
    }



   typedef uint8_t _result_type;
  _result_type result;

   typedef int8_t _error_code_type;
  _error_code_type error_code;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _message_type;
  _message_type message;




  typedef boost::shared_ptr< ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator> const> ConstPtr;

}; // struct RequestInteractionResponse_

typedef ::rocon_interaction_msgs::RequestInteractionResponse_<std::allocator<void> > RequestInteractionResponse;

typedef boost::shared_ptr< ::rocon_interaction_msgs::RequestInteractionResponse > RequestInteractionResponsePtr;
typedef boost::shared_ptr< ::rocon_interaction_msgs::RequestInteractionResponse const> RequestInteractionResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rocon_interaction_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'rocon_std_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_std_msgs/msg', '/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/devel/share/rocon_std_msgs/msg'], 'rocon_service_pair_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_service_pair_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'rocon_interaction_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_interaction_msgs/msg'], 'uuid_msgs': ['/opt/ros/indigo/share/uuid_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "59c96bfec0e66f629d852df5b53676e1";
  }

  static const char* value(const ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x59c96bfec0e66f62ULL;
  static const uint64_t static_value2 = 0x9d852df5b53676e1ULL;
};

template<class ContainerAllocator>
struct DataType< ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rocon_interaction_msgs/RequestInteractionResponse";
  }

  static const char* value(const ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
bool result\n\
\n\
\n\
int8 error_code\n\
\n\
\n\
string message\n\
\n\
";
  }

  static const char* value(const ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
      stream.next(m.error_code);
      stream.next(m.message);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct RequestInteractionResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rocon_interaction_msgs::RequestInteractionResponse_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.result);
    s << indent << "error_code: ";
    Printer<int8_t>::stream(s, indent + "  ", v.error_code);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.message);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROCON_INTERACTION_MSGS_MESSAGE_REQUESTINTERACTIONRESPONSE_H
