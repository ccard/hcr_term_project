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
 * Auto-generated by genmsg_cpp from file /home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/kobuki/devel/share/yocs_msgs/msg/NavigateToFeedback.msg
 *
 */


#ifndef YOCS_MSGS_MESSAGE_NAVIGATETOFEEDBACK_H
#define YOCS_MSGS_MESSAGE_NAVIGATETOFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace yocs_msgs
{
template <class ContainerAllocator>
struct NavigateToFeedback_
{
  typedef NavigateToFeedback_<ContainerAllocator> Type;

  NavigateToFeedback_()
    : message()
    , distance(0.0)
    , remain_time(0.0)
    , status(0)  {
    }
  NavigateToFeedback_(const ContainerAllocator& _alloc)
    : message(_alloc)
    , distance(0.0)
    , remain_time(0.0)
    , status(0)  {
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _message_type;
  _message_type message;

   typedef float _distance_type;
  _distance_type distance;

   typedef float _remain_time_type;
  _remain_time_type remain_time;

   typedef int8_t _status_type;
  _status_type status;


    enum { STATUS_RETRY = 21 };
     enum { STATUS_INPROGRESS = 22 };
     enum { STATUS_ERROR = 23 };
 

  typedef boost::shared_ptr< ::yocs_msgs::NavigateToFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::yocs_msgs::NavigateToFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct NavigateToFeedback_

typedef ::yocs_msgs::NavigateToFeedback_<std::allocator<void> > NavigateToFeedback;

typedef boost::shared_ptr< ::yocs_msgs::NavigateToFeedback > NavigateToFeedbackPtr;
typedef boost::shared_ptr< ::yocs_msgs::NavigateToFeedback const> NavigateToFeedbackConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::yocs_msgs::NavigateToFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::yocs_msgs::NavigateToFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace yocs_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'yocs_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/kobuki/src/yocs_msgs/msg', '/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/kobuki/devel/share/yocs_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::yocs_msgs::NavigateToFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::yocs_msgs::NavigateToFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::yocs_msgs::NavigateToFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::yocs_msgs::NavigateToFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::yocs_msgs::NavigateToFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::yocs_msgs::NavigateToFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::yocs_msgs::NavigateToFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b7dbba42e365116e557ccba16fa0414a";
  }

  static const char* value(const ::yocs_msgs::NavigateToFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb7dbba42e365116eULL;
  static const uint64_t static_value2 = 0x557ccba16fa0414aULL;
};

template<class ContainerAllocator>
struct DataType< ::yocs_msgs::NavigateToFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "yocs_msgs/NavigateToFeedback";
  }

  static const char* value(const ::yocs_msgs::NavigateToFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::yocs_msgs::NavigateToFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Feedback\n\
string  message\n\
float32 distance\n\
float32 remain_time\n\
int8    status\n\
\n\
int8 STATUS_RETRY       = 21\n\
int8 STATUS_INPROGRESS  = 22\n\
int8 STATUS_ERROR       = 23\n\
\n\
";
  }

  static const char* value(const ::yocs_msgs::NavigateToFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::yocs_msgs::NavigateToFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.message);
      stream.next(m.distance);
      stream.next(m.remain_time);
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct NavigateToFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::yocs_msgs::NavigateToFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::yocs_msgs::NavigateToFeedback_<ContainerAllocator>& v)
  {
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.message);
    s << indent << "distance: ";
    Printer<float>::stream(s, indent + "  ", v.distance);
    s << indent << "remain_time: ";
    Printer<float>::stream(s, indent + "  ", v.remain_time);
    s << indent << "status: ";
    Printer<int8_t>::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // YOCS_MSGS_MESSAGE_NAVIGATETOFEEDBACK_H
