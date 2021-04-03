// Generated by gencpp from file demo_msgs/Team2.msg
// DO NOT EDIT!


#ifndef DEMO_MSGS_MESSAGE_TEAM2_H
#define DEMO_MSGS_MESSAGE_TEAM2_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <demo_msgs/Student.h>
#include <std_msgs/String.h>

namespace demo_msgs
{
template <class ContainerAllocator>
struct Team2_
{
  typedef Team2_<ContainerAllocator> Type;

  Team2_()
    : name()
    , leader()
    , intro()  {
    }
  Team2_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , leader(_alloc)
    , intro(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef  ::demo_msgs::Student_<ContainerAllocator>  _leader_type;
  _leader_type leader;

   typedef  ::std_msgs::String_<ContainerAllocator>  _intro_type;
  _intro_type intro;





  typedef boost::shared_ptr< ::demo_msgs::Team2_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::demo_msgs::Team2_<ContainerAllocator> const> ConstPtr;

}; // struct Team2_

typedef ::demo_msgs::Team2_<std::allocator<void> > Team2;

typedef boost::shared_ptr< ::demo_msgs::Team2 > Team2Ptr;
typedef boost::shared_ptr< ::demo_msgs::Team2 const> Team2ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::demo_msgs::Team2_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::demo_msgs::Team2_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::demo_msgs::Team2_<ContainerAllocator1> & lhs, const ::demo_msgs::Team2_<ContainerAllocator2> & rhs)
{
  return lhs.name == rhs.name &&
    lhs.leader == rhs.leader &&
    lhs.intro == rhs.intro;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::demo_msgs::Team2_<ContainerAllocator1> & lhs, const ::demo_msgs::Team2_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace demo_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::demo_msgs::Team2_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::demo_msgs::Team2_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::demo_msgs::Team2_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::demo_msgs::Team2_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::demo_msgs::Team2_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::demo_msgs::Team2_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::demo_msgs::Team2_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8d49b9b867129a9e436a004b0d6e3743";
  }

  static const char* value(const ::demo_msgs::Team2_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8d49b9b867129a9eULL;
  static const uint64_t static_value2 = 0x436a004b0d6e3743ULL;
};

template<class ContainerAllocator>
struct DataType< ::demo_msgs::Team2_<ContainerAllocator> >
{
  static const char* value()
  {
    return "demo_msgs/Team2";
  }

  static const char* value(const ::demo_msgs::Team2_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::demo_msgs::Team2_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string name\n"
"Student leader \n"
"std_msgs/String intro\n"
"\n"
"================================================================================\n"
"MSG: demo_msgs/Student\n"
"string name\n"
"int32 age\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/String\n"
"string data\n"
;
  }

  static const char* value(const ::demo_msgs::Team2_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::demo_msgs::Team2_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.leader);
      stream.next(m.intro);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Team2_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::demo_msgs::Team2_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::demo_msgs::Team2_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "leader: ";
    s << std::endl;
    Printer< ::demo_msgs::Student_<ContainerAllocator> >::stream(s, indent + "  ", v.leader);
    s << indent << "intro: ";
    s << std::endl;
    Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "  ", v.intro);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DEMO_MSGS_MESSAGE_TEAM2_H
