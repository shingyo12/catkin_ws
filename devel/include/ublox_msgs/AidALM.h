// Generated by gencpp from file ublox_msgs/AidALM.msg
// DO NOT EDIT!


#ifndef UBLOX_MSGS_MESSAGE_AIDALM_H
#define UBLOX_MSGS_MESSAGE_AIDALM_H


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
struct AidALM_
{
  typedef AidALM_<ContainerAllocator> Type;

  AidALM_()
    : svid(0)
    , week(0)
    , dwrd()  {
    }
  AidALM_(const ContainerAllocator& _alloc)
    : svid(0)
    , week(0)
    , dwrd(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _svid_type;
  _svid_type svid;

   typedef uint32_t _week_type;
  _week_type week;

   typedef std::vector<uint32_t, typename ContainerAllocator::template rebind<uint32_t>::other >  _dwrd_type;
  _dwrd_type dwrd;



  enum {
    CLASS_ID = 11u,
    MESSAGE_ID = 48u,
  };


  typedef boost::shared_ptr< ::ublox_msgs::AidALM_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_msgs::AidALM_<ContainerAllocator> const> ConstPtr;

}; // struct AidALM_

typedef ::ublox_msgs::AidALM_<std::allocator<void> > AidALM;

typedef boost::shared_ptr< ::ublox_msgs::AidALM > AidALMPtr;
typedef boost::shared_ptr< ::ublox_msgs::AidALM const> AidALMConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_msgs::AidALM_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_msgs::AidALM_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ublox_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'ublox_msgs': ['/home/nvidia/catkin_ws/src/ublox_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::AidALM_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::AidALM_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::AidALM_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::AidALM_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::AidALM_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::AidALM_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_msgs::AidALM_<ContainerAllocator> >
{
  static const char* value()
  {
    return "de5ab2550e698fc8acfb7263c7c55fa2";
  }

  static const char* value(const ::ublox_msgs::AidALM_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xde5ab2550e698fc8ULL;
  static const uint64_t static_value2 = 0xacfb7263c7c55fa2ULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_msgs::AidALM_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_msgs/AidALM";
  }

  static const char* value(const ::ublox_msgs::AidALM_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_msgs::AidALM_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# AID-ALM (0x0B 0x30)\n\
# GPS Aiding Almanach Input/Output Message\n\
#\n\
# All UBX-AID messages are deprecated; use UBX-MGA messages instead\n\
# - If the WEEK Value is 0, DWRD0 to DWRD7 are not sent as the almanach is not \n\
#   available for the given SV. This may happen even if NAV-SVINFO and RXM-SVSI \n\
#   are indicating almanac availability as the internal data may not represent \n\
#   the content of an original broadcast almanac (or only parts thereof).\n\
# - DWORD0 to DWORD7 contain the 8 words following the Hand-Over Word ( HOW )\n\
#   from the GPS navigation message, either pages 1 to 24 of sub-frame 5 or \n\
#   pages 2 to 10 of subframe 4. See IS-GPS-200 for a full description of the \n\
#   contents of the Almanac pages.\n\
# - In DWORD0 to DWORD7, the parity bits have been removed, and the 24 bits of \n\
#   data are located in Bits 0 to 23. Bits 24 to 31 shall be ignored.\n\
# - Example: Parameter e (Eccentricity) from Almanach Subframe 4/5, Word 3, \n\
#   Bits 69-84 within the subframe can be found in DWRD0, Bits 15-0 whereas \n\
#   Bit 0 is the LSB.\n\
\n\
uint8 CLASS_ID = 11\n\
uint8 MESSAGE_ID = 48\n\
\n\
uint32 svid             # SV ID for which the receiver shall return its \n\
                        # Almanac Data (Valid Range: 1 .. 32 or 51, 56, 63).\n\
uint32 week             # Issue Date of Almanach (GPS week number)\n\
\n\
# Start of optional block\n\
uint32[] dwrd           # Almanach Words\n\
# End of optional block\n\
";
  }

  static const char* value(const ::ublox_msgs::AidALM_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_msgs::AidALM_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.svid);
      stream.next(m.week);
      stream.next(m.dwrd);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AidALM_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_msgs::AidALM_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_msgs::AidALM_<ContainerAllocator>& v)
  {
    s << indent << "svid: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.svid);
    s << indent << "week: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.week);
    s << indent << "dwrd[]" << std::endl;
    for (size_t i = 0; i < v.dwrd.size(); ++i)
    {
      s << indent << "  dwrd[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.dwrd[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_MSGS_MESSAGE_AIDALM_H
