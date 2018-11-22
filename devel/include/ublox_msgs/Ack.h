// Generated by gencpp from file ublox_msgs/Ack.msg
// DO NOT EDIT!


#ifndef UBLOX_MSGS_MESSAGE_ACK_H
#define UBLOX_MSGS_MESSAGE_ACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ublox_msgs
{
template <class ContainerAllocator>
struct Ack_
{
  typedef Ack_<ContainerAllocator> Type;

  Ack_()
    : clsID(0)
    , msgID(0)  {
    }
  Ack_(const ContainerAllocator& _alloc)
    : clsID(0)
    , msgID(0)  {
  (void)_alloc;
    }



   typedef uint8_t _clsID_type;
  _clsID_type clsID;

   typedef uint8_t _msgID_type;
  _msgID_type msgID;



  enum {
    CLASS_ID = 5u,
    NACK_MESSAGE_ID = 0u,
    ACK_MESSAGE_ID = 1u,
  };


  typedef boost::shared_ptr< ::ublox_msgs::Ack_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_msgs::Ack_<ContainerAllocator> const> ConstPtr;

}; // struct Ack_

typedef ::ublox_msgs::Ack_<std::allocator<void> > Ack;

typedef boost::shared_ptr< ::ublox_msgs::Ack > AckPtr;
typedef boost::shared_ptr< ::ublox_msgs::Ack const> AckConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_msgs::Ack_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_msgs::Ack_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ublox_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'ublox_msgs': ['/home/nvidia/catkin_ws/src/ublox_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::Ack_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::Ack_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::Ack_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::Ack_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::Ack_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::Ack_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_msgs::Ack_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fc3270816d86d7c962dbc4b52a6c5386";
  }

  static const char* value(const ::ublox_msgs::Ack_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfc3270816d86d7c9ULL;
  static const uint64_t static_value2 = 0x62dbc4b52a6c5386ULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_msgs::Ack_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_msgs/Ack";
  }

  static const char* value(const ::ublox_msgs::Ack_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_msgs::Ack_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ACK (0x05)\n\
# Message Acknowledged / Not-Acknowledged\n\
#\n\
# Output upon processing of an input message\n\
#\n\
\n\
uint8 CLASS_ID = 5\n\
uint8 NACK_MESSAGE_ID = 0\n\
uint8 ACK_MESSAGE_ID = 1\n\
\n\
uint8 clsID   # Class ID of the (Not-)Acknowledged Message\n\
uint8 msgID   # Message ID of the (Not-)Acknowledged Message\n\
";
  }

  static const char* value(const ::ublox_msgs::Ack_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_msgs::Ack_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.clsID);
      stream.next(m.msgID);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Ack_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_msgs::Ack_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_msgs::Ack_<ContainerAllocator>& v)
  {
    s << indent << "clsID: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.clsID);
    s << indent << "msgID: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.msgID);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_MSGS_MESSAGE_ACK_H
