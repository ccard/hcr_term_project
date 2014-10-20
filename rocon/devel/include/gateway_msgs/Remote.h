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
 * Auto-generated by gensrv_cpp from file /home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/gateway_msgs/srv/Remote.srv
 *
 */


#ifndef GATEWAY_MSGS_MESSAGE_REMOTE_H
#define GATEWAY_MSGS_MESSAGE_REMOTE_H

#include <ros/service_traits.h>


#include <gateway_msgs/RemoteRequest.h>
#include <gateway_msgs/RemoteResponse.h>


namespace gateway_msgs
{

struct Remote
{

typedef RemoteRequest Request;
typedef RemoteResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Remote
} // namespace gateway_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::gateway_msgs::Remote > {
  static const char* value()
  {
    return "d2170021bdea1c8bfca632d3d43a6993";
  }

  static const char* value(const ::gateway_msgs::Remote&) { return value(); }
};

template<>
struct DataType< ::gateway_msgs::Remote > {
  static const char* value()
  {
    return "gateway_msgs/Remote";
  }

  static const char* value(const ::gateway_msgs::Remote&) { return value(); }
};


// service_traits::MD5Sum< ::gateway_msgs::RemoteRequest> should match 
// service_traits::MD5Sum< ::gateway_msgs::Remote > 
template<>
struct MD5Sum< ::gateway_msgs::RemoteRequest>
{
  static const char* value()
  {
    return MD5Sum< ::gateway_msgs::Remote >::value();
  }
  static const char* value(const ::gateway_msgs::RemoteRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::gateway_msgs::RemoteRequest> should match 
// service_traits::DataType< ::gateway_msgs::Remote > 
template<>
struct DataType< ::gateway_msgs::RemoteRequest>
{
  static const char* value()
  {
    return DataType< ::gateway_msgs::Remote >::value();
  }
  static const char* value(const ::gateway_msgs::RemoteRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::gateway_msgs::RemoteResponse> should match 
// service_traits::MD5Sum< ::gateway_msgs::Remote > 
template<>
struct MD5Sum< ::gateway_msgs::RemoteResponse>
{
  static const char* value()
  {
    return MD5Sum< ::gateway_msgs::Remote >::value();
  }
  static const char* value(const ::gateway_msgs::RemoteResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::gateway_msgs::RemoteResponse> should match 
// service_traits::DataType< ::gateway_msgs::Remote > 
template<>
struct DataType< ::gateway_msgs::RemoteResponse>
{
  static const char* value()
  {
    return DataType< ::gateway_msgs::Remote >::value();
  }
  static const char* value(const ::gateway_msgs::RemoteResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // GATEWAY_MSGS_MESSAGE_REMOTE_H
