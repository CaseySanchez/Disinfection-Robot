// Generated by gencpp from file uvc/set_lamp.msg
// DO NOT EDIT!


#ifndef UVC_MESSAGE_SET_LAMP_H
#define UVC_MESSAGE_SET_LAMP_H

#include <ros/service_traits.h>


#include <uvc/set_lampRequest.h>
#include <uvc/set_lampResponse.h>


namespace uvc
{

struct set_lamp
{

typedef set_lampRequest Request;
typedef set_lampResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct set_lamp
} // namespace uvc


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::uvc::set_lamp > {
  static const char* value()
  {
    return "fa6cae5645c43d390aecc4758f0073c7";
  }

  static const char* value(const ::uvc::set_lamp&) { return value(); }
};

template<>
struct DataType< ::uvc::set_lamp > {
  static const char* value()
  {
    return "uvc/set_lamp";
  }

  static const char* value(const ::uvc::set_lamp&) { return value(); }
};


// service_traits::MD5Sum< ::uvc::set_lampRequest> should match
// service_traits::MD5Sum< ::uvc::set_lamp >
template<>
struct MD5Sum< ::uvc::set_lampRequest>
{
  static const char* value()
  {
    return MD5Sum< ::uvc::set_lamp >::value();
  }
  static const char* value(const ::uvc::set_lampRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::uvc::set_lampRequest> should match
// service_traits::DataType< ::uvc::set_lamp >
template<>
struct DataType< ::uvc::set_lampRequest>
{
  static const char* value()
  {
    return DataType< ::uvc::set_lamp >::value();
  }
  static const char* value(const ::uvc::set_lampRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::uvc::set_lampResponse> should match
// service_traits::MD5Sum< ::uvc::set_lamp >
template<>
struct MD5Sum< ::uvc::set_lampResponse>
{
  static const char* value()
  {
    return MD5Sum< ::uvc::set_lamp >::value();
  }
  static const char* value(const ::uvc::set_lampResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::uvc::set_lampResponse> should match
// service_traits::DataType< ::uvc::set_lamp >
template<>
struct DataType< ::uvc::set_lampResponse>
{
  static const char* value()
  {
    return DataType< ::uvc::set_lamp >::value();
  }
  static const char* value(const ::uvc::set_lampResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // UVC_MESSAGE_SET_LAMP_H
