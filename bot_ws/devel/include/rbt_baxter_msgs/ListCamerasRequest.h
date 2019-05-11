// Generated by gencpp from file rbt_baxter_msgs/ListCamerasRequest.msg
// DO NOT EDIT!


#ifndef RBT_BAXTER_MSGS_MESSAGE_LISTCAMERASREQUEST_H
#define RBT_BAXTER_MSGS_MESSAGE_LISTCAMERASREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rbt_baxter_msgs
{
template <class ContainerAllocator>
struct ListCamerasRequest_
{
  typedef ListCamerasRequest_<ContainerAllocator> Type;

  ListCamerasRequest_()
    {
    }
  ListCamerasRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }






  typedef boost::shared_ptr< ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ListCamerasRequest_

typedef ::rbt_baxter_msgs::ListCamerasRequest_<std::allocator<void> > ListCamerasRequest;

typedef boost::shared_ptr< ::rbt_baxter_msgs::ListCamerasRequest > ListCamerasRequestPtr;
typedef boost::shared_ptr< ::rbt_baxter_msgs::ListCamerasRequest const> ListCamerasRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rbt_baxter_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'rbt_baxter_msgs': ['/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rbt_baxter_msgs/ListCamerasRequest";
  }

  static const char* value(const ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
";
  }

  static const char* value(const ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ListCamerasRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::rbt_baxter_msgs::ListCamerasRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // RBT_BAXTER_MSGS_MESSAGE_LISTCAMERASREQUEST_H
