// Generated by gencpp from file rbt_baxter_msgs/OpenCamera.msg
// DO NOT EDIT!


#ifndef RBT_BAXTER_MSGS_MESSAGE_OPENCAMERA_H
#define RBT_BAXTER_MSGS_MESSAGE_OPENCAMERA_H

#include <ros/service_traits.h>


#include <rbt_baxter_msgs/OpenCameraRequest.h>
#include <rbt_baxter_msgs/OpenCameraResponse.h>


namespace rbt_baxter_msgs
{

struct OpenCamera
{

typedef OpenCameraRequest Request;
typedef OpenCameraResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct OpenCamera
} // namespace rbt_baxter_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rbt_baxter_msgs::OpenCamera > {
  static const char* value()
  {
    return "73eacff63d5f9cca2d986614515a5c8c";
  }

  static const char* value(const ::rbt_baxter_msgs::OpenCamera&) { return value(); }
};

template<>
struct DataType< ::rbt_baxter_msgs::OpenCamera > {
  static const char* value()
  {
    return "rbt_baxter_msgs/OpenCamera";
  }

  static const char* value(const ::rbt_baxter_msgs::OpenCamera&) { return value(); }
};


// service_traits::MD5Sum< ::rbt_baxter_msgs::OpenCameraRequest> should match 
// service_traits::MD5Sum< ::rbt_baxter_msgs::OpenCamera > 
template<>
struct MD5Sum< ::rbt_baxter_msgs::OpenCameraRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rbt_baxter_msgs::OpenCamera >::value();
  }
  static const char* value(const ::rbt_baxter_msgs::OpenCameraRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rbt_baxter_msgs::OpenCameraRequest> should match 
// service_traits::DataType< ::rbt_baxter_msgs::OpenCamera > 
template<>
struct DataType< ::rbt_baxter_msgs::OpenCameraRequest>
{
  static const char* value()
  {
    return DataType< ::rbt_baxter_msgs::OpenCamera >::value();
  }
  static const char* value(const ::rbt_baxter_msgs::OpenCameraRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rbt_baxter_msgs::OpenCameraResponse> should match 
// service_traits::MD5Sum< ::rbt_baxter_msgs::OpenCamera > 
template<>
struct MD5Sum< ::rbt_baxter_msgs::OpenCameraResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rbt_baxter_msgs::OpenCamera >::value();
  }
  static const char* value(const ::rbt_baxter_msgs::OpenCameraResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rbt_baxter_msgs::OpenCameraResponse> should match 
// service_traits::DataType< ::rbt_baxter_msgs::OpenCamera > 
template<>
struct DataType< ::rbt_baxter_msgs::OpenCameraResponse>
{
  static const char* value()
  {
    return DataType< ::rbt_baxter_msgs::OpenCamera >::value();
  }
  static const char* value(const ::rbt_baxter_msgs::OpenCameraResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // RBT_BAXTER_MSGS_MESSAGE_OPENCAMERA_H
