// Generated by gencpp from file wiimote/IrSourceInfo.msg
// DO NOT EDIT!


#ifndef WIIMOTE_MESSAGE_IRSOURCEINFO_H
#define WIIMOTE_MESSAGE_IRSOURCEINFO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace wiimote
{
template <class ContainerAllocator>
struct IrSourceInfo_
{
  typedef IrSourceInfo_<ContainerAllocator> Type;

  IrSourceInfo_()
    : x(0.0)
    , y(0.0)
    , ir_size(0)  {
    }
  IrSourceInfo_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , ir_size(0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef int64_t _ir_size_type;
  _ir_size_type ir_size;





  typedef boost::shared_ptr< ::wiimote::IrSourceInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::wiimote::IrSourceInfo_<ContainerAllocator> const> ConstPtr;

}; // struct IrSourceInfo_

typedef ::wiimote::IrSourceInfo_<std::allocator<void> > IrSourceInfo;

typedef boost::shared_ptr< ::wiimote::IrSourceInfo > IrSourceInfoPtr;
typedef boost::shared_ptr< ::wiimote::IrSourceInfo const> IrSourceInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::wiimote::IrSourceInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::wiimote::IrSourceInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace wiimote

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'wiimote': ['/tmp/binarydeb/ros-melodic-wiimote-1.13.0/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::wiimote::IrSourceInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::wiimote::IrSourceInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::wiimote::IrSourceInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::wiimote::IrSourceInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::wiimote::IrSourceInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::wiimote::IrSourceInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::wiimote::IrSourceInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "95274ca88b9f008b99984b9a61d2772e";
  }

  static const char* value(const ::wiimote::IrSourceInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x95274ca88b9f008bULL;
  static const uint64_t static_value2 = 0x99984b9a61d2772eULL;
};

template<class ContainerAllocator>
struct DataType< ::wiimote::IrSourceInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "wiimote/IrSourceInfo";
  }

  static const char* value(const ::wiimote::IrSourceInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::wiimote::IrSourceInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Sensor data pertaining to the Wiimote infrared camera.\n"
"# This message contains data for one of the four infrared \n"
"# light sources that the camera detects.\n"
"#\n"
"# Each light is specified with a 2D position and \n"
"# a 'source magnitude' (ir_size). If the x dimension\n"
"# is set to INVALID_FLOAT, then no light was detected for \n"
"# the respective light. The Wiimote handles up to\n"
"# four light sources, and the wiimote_node.py software\n"
"# is written to that limit as well.\n"
"#\n"
"# I am unsure what the 'ir_size' values represent. \n"
"# They are described as 'source magnitude' in some places. I\n"
"# *assume* this is signal amplitude, but it's unclear. \n"
"# Note that current lowest level cwiid driver does not \n"
"# seem to pass the ir_size value to the cwiid Wiimote.c. \n"
"# For now this size will therefore be set INVALID\n"
"\n"
"float64 x \n"
"float64 y \n"
"int64 ir_size\n"
;
  }

  static const char* value(const ::wiimote::IrSourceInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::wiimote::IrSourceInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.ir_size);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct IrSourceInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::wiimote::IrSourceInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::wiimote::IrSourceInfo_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "ir_size: ";
    Printer<int64_t>::stream(s, indent + "  ", v.ir_size);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WIIMOTE_MESSAGE_IRSOURCEINFO_H
