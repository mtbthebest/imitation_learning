// Generated by gencpp from file rbt_baxter_msgs/DigitalOutputCommand.msg
// DO NOT EDIT!


#ifndef RBT_BAXTER_MSGS_MESSAGE_DIGITALOUTPUTCOMMAND_H
#define RBT_BAXTER_MSGS_MESSAGE_DIGITALOUTPUTCOMMAND_H


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
struct DigitalOutputCommand_
{
  typedef DigitalOutputCommand_<ContainerAllocator> Type;

  DigitalOutputCommand_()
    : name()
    , value(false)  {
    }
  DigitalOutputCommand_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , value(false)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef uint8_t _value_type;
  _value_type value;




  typedef boost::shared_ptr< ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator> const> ConstPtr;

}; // struct DigitalOutputCommand_

typedef ::rbt_baxter_msgs::DigitalOutputCommand_<std::allocator<void> > DigitalOutputCommand;

typedef boost::shared_ptr< ::rbt_baxter_msgs::DigitalOutputCommand > DigitalOutputCommandPtr;
typedef boost::shared_ptr< ::rbt_baxter_msgs::DigitalOutputCommand const> DigitalOutputCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rbt_baxter_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'rbt_baxter_msgs': ['/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "23f05028c1a699fb83e22401228c3a9e";
  }

  static const char* value(const ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x23f05028c1a699fbULL;
  static const uint64_t static_value2 = 0x83e22401228c3a9eULL;
};

template<class ContainerAllocator>
struct DataType< ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rbt_baxter_msgs/DigitalOutputCommand";
  }

  static const char* value(const ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "##the name of the output\n\
string name  \n\
##the value to set output \n\
bool value   \n\
";
  }

  static const char* value(const ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DigitalOutputCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rbt_baxter_msgs::DigitalOutputCommand_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "value: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RBT_BAXTER_MSGS_MESSAGE_DIGITALOUTPUTCOMMAND_H
