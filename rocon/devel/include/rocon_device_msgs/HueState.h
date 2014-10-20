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
 * Auto-generated by genmsg_cpp from file /home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_device_msgs/msg/HueState.msg
 *
 */


#ifndef ROCON_DEVICE_MSGS_MESSAGE_HUESTATE_H
#define ROCON_DEVICE_MSGS_MESSAGE_HUESTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rocon_device_msgs
{
template <class ContainerAllocator>
struct HueState_
{
  typedef HueState_<ContainerAllocator> Type;

  HueState_()
    : on(false)
    , xy()
    , hue(0)
    , sat(0)
    , bri(0)
    , ct(0)
    , mode()
    , transitiontime(0)
    , color_mode()
    , reachable(false)  {
    }
  HueState_(const ContainerAllocator& _alloc)
    : on(false)
    , xy(_alloc)
    , hue(0)
    , sat(0)
    , bri(0)
    , ct(0)
    , mode(_alloc)
    , transitiontime(0)
    , color_mode(_alloc)
    , reachable(false)  {
    }



   typedef uint8_t _on_type;
  _on_type on;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _xy_type;
  _xy_type xy;

   typedef uint16_t _hue_type;
  _hue_type hue;

   typedef uint8_t _sat_type;
  _sat_type sat;

   typedef uint8_t _bri_type;
  _bri_type bri;

   typedef uint16_t _ct_type;
  _ct_type ct;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _mode_type;
  _mode_type mode;

   typedef int32_t _transitiontime_type;
  _transitiontime_type transitiontime;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _color_mode_type;
  _color_mode_type color_mode;

   typedef uint8_t _reachable_type;
  _reachable_type reachable;


    static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  NONE;
     static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  COLOR_LOOP;
     static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  SELECT;
     static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  LSELECT;
     static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  HS;
     static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  XY;
     static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  CT;
 

  typedef boost::shared_ptr< ::rocon_device_msgs::HueState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rocon_device_msgs::HueState_<ContainerAllocator> const> ConstPtr;

}; // struct HueState_

typedef ::rocon_device_msgs::HueState_<std::allocator<void> > HueState;

typedef boost::shared_ptr< ::rocon_device_msgs::HueState > HueStatePtr;
typedef boost::shared_ptr< ::rocon_device_msgs::HueState const> HueStateConstPtr;

// constants requiring out of line definition

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      HueState_<ContainerAllocator>::NONE =
        
          "none"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      HueState_<ContainerAllocator>::COLOR_LOOP =
        
          "colorloop"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      HueState_<ContainerAllocator>::SELECT =
        
          "select"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      HueState_<ContainerAllocator>::LSELECT =
        
          "lselect"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      HueState_<ContainerAllocator>::HS =
        
          "hs #hsv color space"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      HueState_<ContainerAllocator>::XY =
        
          "xy #cie color space"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      HueState_<ContainerAllocator>::CT =
        
          "ct #color temperature space"
        
        ;
   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rocon_device_msgs::HueState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rocon_device_msgs::HueState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rocon_device_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'rocon_device_msgs': ['/home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_device_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rocon_device_msgs::HueState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rocon_device_msgs::HueState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_device_msgs::HueState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_device_msgs::HueState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_device_msgs::HueState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_device_msgs::HueState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rocon_device_msgs::HueState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7e4ad09b859196f23a5883df58e12e77";
  }

  static const char* value(const ::rocon_device_msgs::HueState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7e4ad09b859196f2ULL;
  static const uint64_t static_value2 = 0x3a5883df58e12e77ULL;
};

template<class ContainerAllocator>
struct DataType< ::rocon_device_msgs::HueState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rocon_device_msgs/HueState";
  }

  static const char* value(const ::rocon_device_msgs::HueState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rocon_device_msgs::HueState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#specific effect\n\
string NONE = none\n\
#effect mode\n\
\n\
#the light will cycle through all hues using the current brightness and saturation settings.\n\
string COLOR_LOOP =  colorloop \n\
# alert mode\n\
\n\
# The light is performing one breathe cycle.\n\
string SELECT = select  \n\
# he light is performing breathe cycles for 30 seconds or mode is none\n\
string LSELECT = lselect \n\
\n\
#color mode\n\
string HS = hs #hsv color space\n\
string XY = xy #cie color space\n\
string CT = ct #color temperature space\n\
\n\
#state\n\
bool on #light on/off flag true:on /false:off\n\
\n\
#color coordination in CIE color space \n\
#http://developers.meethue.com/coreconcepts.html#color_gets_more_complicated\n\
float32[] xy #xy \n\
\n\
#color coordination in HSV color space \n\
#http://en.wikipedia.org/wiki/HSL_and_HSV\n\
uint16 hue #h\n\
uint8 sat #s\n\
uint8 bri #v\n\
\n\
#color temperature  \n\
#http://en.wikipedia.org/wiki/Mired\n\
#capable of 153 (6500K) to 500 (2000K)\n\
uint16 ct \n\
\n\
#specific effect\n\
string mode\n\
\n\
#transition time\n\
int32 transitiontime\n\
\n\
\n\
#config\n\
string color_mode\n\
bool reachable\n\
";
  }

  static const char* value(const ::rocon_device_msgs::HueState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rocon_device_msgs::HueState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.on);
      stream.next(m.xy);
      stream.next(m.hue);
      stream.next(m.sat);
      stream.next(m.bri);
      stream.next(m.ct);
      stream.next(m.mode);
      stream.next(m.transitiontime);
      stream.next(m.color_mode);
      stream.next(m.reachable);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct HueState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rocon_device_msgs::HueState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rocon_device_msgs::HueState_<ContainerAllocator>& v)
  {
    s << indent << "on: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.on);
    s << indent << "xy[]" << std::endl;
    for (size_t i = 0; i < v.xy.size(); ++i)
    {
      s << indent << "  xy[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.xy[i]);
    }
    s << indent << "hue: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.hue);
    s << indent << "sat: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.sat);
    s << indent << "bri: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.bri);
    s << indent << "ct: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.ct);
    s << indent << "mode: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.mode);
    s << indent << "transitiontime: ";
    Printer<int32_t>::stream(s, indent + "  ", v.transitiontime);
    s << indent << "color_mode: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.color_mode);
    s << indent << "reachable: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reachable);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROCON_DEVICE_MSGS_MESSAGE_HUESTATE_H
