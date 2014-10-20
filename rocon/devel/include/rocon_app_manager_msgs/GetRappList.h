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
 * Auto-generated by gensrv_cpp from file /home/chris/Documents/mines/mines_grad/csci598/hcr_term_project/rocon/src/rocon_msgs/rocon_app_manager_msgs/srv/GetRappList.srv
 *
 */


#ifndef ROCON_APP_MANAGER_MSGS_MESSAGE_GETRAPPLIST_H
#define ROCON_APP_MANAGER_MSGS_MESSAGE_GETRAPPLIST_H

#include <ros/service_traits.h>


#include <rocon_app_manager_msgs/GetRappListRequest.h>
#include <rocon_app_manager_msgs/GetRappListResponse.h>


namespace rocon_app_manager_msgs
{

struct GetRappList
{

typedef GetRappListRequest Request;
typedef GetRappListResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetRappList
} // namespace rocon_app_manager_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rocon_app_manager_msgs::GetRappList > {
  static const char* value()
  {
    return "68cf30e80880962b92b2794f1f078c7f";
  }

  static const char* value(const ::rocon_app_manager_msgs::GetRappList&) { return value(); }
};

template<>
struct DataType< ::rocon_app_manager_msgs::GetRappList > {
  static const char* value()
  {
    return "rocon_app_manager_msgs/GetRappList";
  }

  static const char* value(const ::rocon_app_manager_msgs::GetRappList&) { return value(); }
};


// service_traits::MD5Sum< ::rocon_app_manager_msgs::GetRappListRequest> should match 
// service_traits::MD5Sum< ::rocon_app_manager_msgs::GetRappList > 
template<>
struct MD5Sum< ::rocon_app_manager_msgs::GetRappListRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rocon_app_manager_msgs::GetRappList >::value();
  }
  static const char* value(const ::rocon_app_manager_msgs::GetRappListRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rocon_app_manager_msgs::GetRappListRequest> should match 
// service_traits::DataType< ::rocon_app_manager_msgs::GetRappList > 
template<>
struct DataType< ::rocon_app_manager_msgs::GetRappListRequest>
{
  static const char* value()
  {
    return DataType< ::rocon_app_manager_msgs::GetRappList >::value();
  }
  static const char* value(const ::rocon_app_manager_msgs::GetRappListRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rocon_app_manager_msgs::GetRappListResponse> should match 
// service_traits::MD5Sum< ::rocon_app_manager_msgs::GetRappList > 
template<>
struct MD5Sum< ::rocon_app_manager_msgs::GetRappListResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rocon_app_manager_msgs::GetRappList >::value();
  }
  static const char* value(const ::rocon_app_manager_msgs::GetRappListResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rocon_app_manager_msgs::GetRappListResponse> should match 
// service_traits::DataType< ::rocon_app_manager_msgs::GetRappList > 
template<>
struct DataType< ::rocon_app_manager_msgs::GetRappListResponse>
{
  static const char* value()
  {
    return DataType< ::rocon_app_manager_msgs::GetRappList >::value();
  }
  static const char* value(const ::rocon_app_manager_msgs::GetRappListResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROCON_APP_MANAGER_MSGS_MESSAGE_GETRAPPLIST_H
