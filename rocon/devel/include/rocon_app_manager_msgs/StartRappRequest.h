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
 * Auto-generated by genmsg_cpp from file /home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_app_manager_msgs/srv/StartRapp.srv
 *
 */


#ifndef ROCON_APP_MANAGER_MSGS_MESSAGE_STARTRAPPREQUEST_H
#define ROCON_APP_MANAGER_MSGS_MESSAGE_STARTRAPPREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <rocon_std_msgs/Remapping.h>
#include <rocon_std_msgs/KeyValue.h>

namespace rocon_app_manager_msgs
{
template <class ContainerAllocator>
struct StartRappRequest_
{
  typedef StartRappRequest_<ContainerAllocator> Type;

  StartRappRequest_()
    : name()
    , remappings()
    , parameters()  {
    }
  StartRappRequest_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , remappings(_alloc)
    , parameters(_alloc)  {
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef std::vector< ::rocon_std_msgs::Remapping_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::rocon_std_msgs::Remapping_<ContainerAllocator> >::other >  _remappings_type;
  _remappings_type remappings;

   typedef std::vector< ::rocon_std_msgs::KeyValue_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::rocon_std_msgs::KeyValue_<ContainerAllocator> >::other >  _parameters_type;
  _parameters_type parameters;




  typedef boost::shared_ptr< ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator> const> ConstPtr;

}; // struct StartRappRequest_

typedef ::rocon_app_manager_msgs::StartRappRequest_<std::allocator<void> > StartRappRequest;

typedef boost::shared_ptr< ::rocon_app_manager_msgs::StartRappRequest > StartRappRequestPtr;
typedef boost::shared_ptr< ::rocon_app_manager_msgs::StartRappRequest const> StartRappRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rocon_app_manager_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'rocon_std_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_std_msgs/msg', '/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/devel/share/rocon_std_msgs/msg'], 'rocon_service_pair_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_service_pair_msgs/msg'], 'rocon_app_manager_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_app_manager_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'uuid_msgs': ['/opt/ros/indigo/share/uuid_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cb167056946b89b371dab6e226563482";
  }

  static const char* value(const ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcb167056946b89b3ULL;
  static const uint64_t static_value2 = 0x71dab6e226563482ULL;
};

template<class ContainerAllocator>
struct DataType< ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rocon_app_manager_msgs/StartRappRequest";
  }

  static const char* value(const ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
string name\n\
rocon_std_msgs/Remapping[] remappings\n\
\n\
\n\
rocon_std_msgs/KeyValue[] parameters\n\
\n\
================================================================================\n\
MSG: rocon_std_msgs/Remapping\n\
# Describes your typical ros remapping\n\
\n\
string remap_from\n\
string remap_to\n\
\n\
================================================================================\n\
MSG: rocon_std_msgs/KeyValue\n\
string key\n\
string value\n\
";
  }

  static const char* value(const ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.remappings);
      stream.next(m.parameters);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct StartRappRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rocon_app_manager_msgs::StartRappRequest_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "remappings[]" << std::endl;
    for (size_t i = 0; i < v.remappings.size(); ++i)
    {
      s << indent << "  remappings[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::rocon_std_msgs::Remapping_<ContainerAllocator> >::stream(s, indent + "    ", v.remappings[i]);
    }
    s << indent << "parameters[]" << std::endl;
    for (size_t i = 0; i < v.parameters.size(); ++i)
    {
      s << indent << "  parameters[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::rocon_std_msgs::KeyValue_<ContainerAllocator> >::stream(s, indent + "    ", v.parameters[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROCON_APP_MANAGER_MSGS_MESSAGE_STARTRAPPREQUEST_H
