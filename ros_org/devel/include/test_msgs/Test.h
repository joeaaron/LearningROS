// Generated by gencpp from file test_msgs/Test.msg
// DO NOT EDIT!


#ifndef TEST_MSGS_MESSAGE_TEST_H
#define TEST_MSGS_MESSAGE_TEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>

namespace test_msgs
{
template <class ContainerAllocator>
struct Test_
{
  typedef Test_<ContainerAllocator> Type;

  Test_()
    : data()
    , vel(0.0)
    , pose()
    , name()  {
    }
  Test_(const ContainerAllocator& _alloc)
    : data(_alloc)
    , vel(0.0)
    , pose(_alloc)
    , name(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _data_type;
  _data_type data;

   typedef float _vel_type;
  _vel_type vel;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;





  typedef boost::shared_ptr< ::test_msgs::Test_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::test_msgs::Test_<ContainerAllocator> const> ConstPtr;

}; // struct Test_

typedef ::test_msgs::Test_<std::allocator<void> > Test;

typedef boost::shared_ptr< ::test_msgs::Test > TestPtr;
typedef boost::shared_ptr< ::test_msgs::Test const> TestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::test_msgs::Test_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::test_msgs::Test_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace test_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'test_msgs': ['/home/aaron/JA/code/LearningROS/ros_org/src/test_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::test_msgs::Test_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_msgs::Test_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_msgs::Test_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_msgs::Test_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_msgs::Test_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_msgs::Test_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::test_msgs::Test_<ContainerAllocator> >
{
  static const char* value()
  {
    return "57d24e0384e259f3b08f5a699fa7b1a0";
  }

  static const char* value(const ::test_msgs::Test_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x57d24e0384e259f3ULL;
  static const uint64_t static_value2 = 0xb08f5a699fa7b1a0ULL;
};

template<class ContainerAllocator>
struct DataType< ::test_msgs::Test_<ContainerAllocator> >
{
  static const char* value()
  {
    return "test_msgs/Test";
  }

  static const char* value(const ::test_msgs::Test_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::test_msgs::Test_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[] data\n\
float32 vel\n\
geometry_msgs/Pose pose\n\
string name\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::test_msgs::Test_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::test_msgs::Test_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
      stream.next(m.vel);
      stream.next(m.pose);
      stream.next(m.name);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Test_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::test_msgs::Test_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::test_msgs::Test_<ContainerAllocator>& v)
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.data[i]);
    }
    s << indent << "vel: ";
    Printer<float>::stream(s, indent + "  ", v.vel);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TEST_MSGS_MESSAGE_TEST_H
