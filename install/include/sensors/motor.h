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
 * Auto-generated by genmsg_cpp from file /home/py/ROSTryphon_Organized/src/sensors/msg/motor.msg
 *
 */


#ifndef SENSORS_MESSAGE_MOTOR_H
#define SENSORS_MESSAGE_MOTOR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace sensors
{
template <class ContainerAllocator>
struct motor_
{
  typedef motor_<ContainerAllocator> Type;

  motor_()
    : id(0)
    , rpm(0)
    , temp(0)
    , volt(0)
    , curr(0)
    , dir(0)
    , speed(0)  {
    }
  motor_(const ContainerAllocator& _alloc)
    : id(0)
    , rpm(0)
    , temp(0)
    , volt(0)
    , curr(0)
    , dir(0)
    , speed(0)  {
    }



   typedef int32_t _id_type;
  _id_type id;

   typedef int32_t _rpm_type;
  _rpm_type rpm;

   typedef int32_t _temp_type;
  _temp_type temp;

   typedef int32_t _volt_type;
  _volt_type volt;

   typedef int32_t _curr_type;
  _curr_type curr;

   typedef int32_t _dir_type;
  _dir_type dir;

   typedef int32_t _speed_type;
  _speed_type speed;




  typedef boost::shared_ptr< ::sensors::motor_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sensors::motor_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct motor_

typedef ::sensors::motor_<std::allocator<void> > motor;

typedef boost::shared_ptr< ::sensors::motor > motorPtr;
typedef boost::shared_ptr< ::sensors::motor const> motorConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sensors::motor_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sensors::motor_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace sensors

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/hydro/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/hydro/share/geometry_msgs/cmake/../msg'], 'sensors': ['/home/py/ROSTryphon_Organized/src/sensors/msg'], 'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::sensors::motor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sensors::motor_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sensors::motor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sensors::motor_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sensors::motor_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sensors::motor_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sensors::motor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1b82f96a00d42a296623e233e7741a45";
  }

  static const char* value(const ::sensors::motor_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1b82f96a00d42a29ULL;
  static const uint64_t static_value2 = 0x6623e233e7741a45ULL;
};

template<class ContainerAllocator>
struct DataType< ::sensors::motor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sensors/motor";
  }

  static const char* value(const ::sensors::motor_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sensors::motor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 id\n\
int32 rpm\n\
int32 temp\n\
int32 volt\n\
int32 curr\n\
int32 dir\n\
int32 speed\n\
";
  }

  static const char* value(const ::sensors::motor_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sensors::motor_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.rpm);
      stream.next(m.temp);
      stream.next(m.volt);
      stream.next(m.curr);
      stream.next(m.dir);
      stream.next(m.speed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct motor_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sensors::motor_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sensors::motor_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "rpm: ";
    Printer<int32_t>::stream(s, indent + "  ", v.rpm);
    s << indent << "temp: ";
    Printer<int32_t>::stream(s, indent + "  ", v.temp);
    s << indent << "volt: ";
    Printer<int32_t>::stream(s, indent + "  ", v.volt);
    s << indent << "curr: ";
    Printer<int32_t>::stream(s, indent + "  ", v.curr);
    s << indent << "dir: ";
    Printer<int32_t>::stream(s, indent + "  ", v.dir);
    s << indent << "speed: ";
    Printer<int32_t>::stream(s, indent + "  ", v.speed);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SENSORS_MESSAGE_MOTOR_H
