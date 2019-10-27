// Generated by gencpp from file rosmon_msgs/StartStopResponse.msg
// DO NOT EDIT!


#ifndef ROSMON_MSGS_MESSAGE_STARTSTOPRESPONSE_H
#define ROSMON_MSGS_MESSAGE_STARTSTOPRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rosmon_msgs
{
template <class ContainerAllocator>
struct StartStopResponse_
{
  typedef StartStopResponse_<ContainerAllocator> Type;

  StartStopResponse_()
    {
    }
  StartStopResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::rosmon_msgs::StartStopResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosmon_msgs::StartStopResponse_<ContainerAllocator> const> ConstPtr;

}; // struct StartStopResponse_

typedef ::rosmon_msgs::StartStopResponse_<std::allocator<void> > StartStopResponse;

typedef boost::shared_ptr< ::rosmon_msgs::StartStopResponse > StartStopResponsePtr;
typedef boost::shared_ptr< ::rosmon_msgs::StartStopResponse const> StartStopResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rosmon_msgs::StartStopResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rosmon_msgs::StartStopResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rosmon_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'rosmon_msgs': ['/tmp/binarydeb/ros-melodic-rosmon-msgs-2.1.1/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rosmon_msgs::StartStopResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosmon_msgs::StartStopResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosmon_msgs::StartStopResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosmon_msgs::StartStopResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosmon_msgs::StartStopResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosmon_msgs::StartStopResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rosmon_msgs::StartStopResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::rosmon_msgs::StartStopResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::rosmon_msgs::StartStopResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rosmon_msgs/StartStopResponse";
  }

  static const char* value(const ::rosmon_msgs::StartStopResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rosmon_msgs::StartStopResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::rosmon_msgs::StartStopResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rosmon_msgs::StartStopResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StartStopResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rosmon_msgs::StartStopResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::rosmon_msgs::StartStopResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // ROSMON_MSGS_MESSAGE_STARTSTOPRESPONSE_H
