// Generated by gencpp from file ublox_msgs/CfgCFG.msg
// DO NOT EDIT!


#ifndef UBLOX_MSGS_MESSAGE_CFGCFG_H
#define UBLOX_MSGS_MESSAGE_CFGCFG_H


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
struct CfgCFG_
{
  typedef CfgCFG_<ContainerAllocator> Type;

  CfgCFG_()
    : clearMask(0)
    , saveMask(0)
    , loadMask(0)
    , deviceMask(0)  {
    }
  CfgCFG_(const ContainerAllocator& _alloc)
    : clearMask(0)
    , saveMask(0)
    , loadMask(0)
    , deviceMask(0)  {
  (void)_alloc;
    }



   typedef uint32_t _clearMask_type;
  _clearMask_type clearMask;

   typedef uint32_t _saveMask_type;
  _saveMask_type saveMask;

   typedef uint32_t _loadMask_type;
  _loadMask_type loadMask;

   typedef uint8_t _deviceMask_type;
  _deviceMask_type deviceMask;



  enum {
    CLASS_ID = 6u,
    MESSAGE_ID = 9u,
    MASK_IO_PORT = 1u,
    MASK_MSG_CONF = 2u,
    MASK_INF_MSG = 4u,
    MASK_NAV_CONF = 8u,
    MASK_RXM_CONF = 16u,
    MASK_SEN_CONF = 256u,
    MASK_RINV_CONF = 512u,
    MASK_ANT_CONF = 1024u,
    MASK_LOG_CONF = 2048u,
    MASK_FTS_CONF = 4096u,
    DEV_BBR = 1u,
    DEV_FLASH = 2u,
    DEV_EEPROM = 4u,
    DEV_SPI_FLASH = 16u,
  };


  typedef boost::shared_ptr< ::ublox_msgs::CfgCFG_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_msgs::CfgCFG_<ContainerAllocator> const> ConstPtr;

}; // struct CfgCFG_

typedef ::ublox_msgs::CfgCFG_<std::allocator<void> > CfgCFG;

typedef boost::shared_ptr< ::ublox_msgs::CfgCFG > CfgCFGPtr;
typedef boost::shared_ptr< ::ublox_msgs::CfgCFG const> CfgCFGConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_msgs::CfgCFG_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_msgs::CfgCFG_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ublox_msgs::CfgCFG_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::CfgCFG_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::CfgCFG_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::CfgCFG_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::CfgCFG_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::CfgCFG_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_msgs::CfgCFG_<ContainerAllocator> >
{
  static const char* value()
  {
    return "82e4847c642bca8fb5b8af4595e063a1";
  }

  static const char* value(const ::ublox_msgs::CfgCFG_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x82e4847c642bca8fULL;
  static const uint64_t static_value2 = 0xb5b8af4595e063a1ULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_msgs::CfgCFG_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_msgs/CfgCFG";
  }

  static const char* value(const ::ublox_msgs::CfgCFG_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_msgs::CfgCFG_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# CFG-CFG (0x06 0x09)\n\
# Clear, Save and Load configurations\n\
#\n\
\n\
uint8 CLASS_ID = 6\n\
uint8 MESSAGE_ID = 9\n\
\n\
uint32 clearMask          # Mask with configuration sub-sections to Clear\n\
                          # (=Load Default Configurations to Permanent\n\
                          # Configurations in non-volatile memory)\n\
uint32 saveMask           # Mask with configuration sub-section to Save\n\
                          # (=Save Current Configuration to Non-volatile\n\
                          # Memory)\n\
uint32 loadMask           # Mask with configuration sub-sections to Load\n\
                          # (=Load Permanent Configurations from\n\
                          # Non-volatile Memory to Current Configurations)\n\
\n\
uint32 MASK_IO_PORT = 1       # Communications port settings. Modifying this \n\
                              # sub-section results in an IO system reset. \n\
                              # Because of this undefined data may be output \n\
                              # for a short period of time after receiving the\n\
                              # message.\n\
uint32 MASK_MSG_CONF = 2      # Message Configuration\n\
uint32 MASK_INF_MSG = 4       # INF Message Configuration\n\
uint32 MASK_NAV_CONF = 8      # Navigation Configuration\n\
uint32 MASK_RXM_CONF = 16     # Receiver Manager Configuration\n\
uint32 MASK_SEN_CONF = 256    # Sensor Interface Configuration, protocol >= 19\n\
uint32 MASK_RINV_CONF = 512   # Remote Inventory Configuration\n\
uint32 MASK_ANT_CONF = 1024   # Antenna Configuration\n\
uint32 MASK_LOG_CONF = 2048   # Logging Configuration\n\
uint32 MASK_FTS_CONF = 4096   # FTS Configuration. Only applicable to the \n\
                              # FTS product variant.\n\
\n\
uint8 deviceMask          # Mask which selects the devices for this command\n\
uint8 DEV_BBR = 1             # device battery backed RAM\n\
uint8 DEV_FLASH = 2           # device Flash\n\
uint8 DEV_EEPROM = 4          # device EEPROM\n\
uint8 DEV_SPI_FLASH = 16      # device SPI Flash\n\
";
  }

  static const char* value(const ::ublox_msgs::CfgCFG_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_msgs::CfgCFG_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.clearMask);
      stream.next(m.saveMask);
      stream.next(m.loadMask);
      stream.next(m.deviceMask);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CfgCFG_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_msgs::CfgCFG_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_msgs::CfgCFG_<ContainerAllocator>& v)
  {
    s << indent << "clearMask: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.clearMask);
    s << indent << "saveMask: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.saveMask);
    s << indent << "loadMask: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.loadMask);
    s << indent << "deviceMask: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.deviceMask);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_MSGS_MESSAGE_CFGCFG_H
