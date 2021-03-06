// Generated by gencpp from file planner/AddRobotMessage.msg
// DO NOT EDIT!


#ifndef PLANNER_MESSAGE_ADDROBOTMESSAGE_H
#define PLANNER_MESSAGE_ADDROBOTMESSAGE_H

#include <ros/service_traits.h>


#include <planner/AddRobotMessageRequest.h>
#include <planner/AddRobotMessageResponse.h>


namespace planner
{

struct AddRobotMessage
{

typedef AddRobotMessageRequest Request;
typedef AddRobotMessageResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct AddRobotMessage
} // namespace planner


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::planner::AddRobotMessage > {
  static const char* value()
  {
    return "33ea4e5aeb30f5913da681ca459d55f3";
  }

  static const char* value(const ::planner::AddRobotMessage&) { return value(); }
};

template<>
struct DataType< ::planner::AddRobotMessage > {
  static const char* value()
  {
    return "planner/AddRobotMessage";
  }

  static const char* value(const ::planner::AddRobotMessage&) { return value(); }
};


// service_traits::MD5Sum< ::planner::AddRobotMessageRequest> should match
// service_traits::MD5Sum< ::planner::AddRobotMessage >
template<>
struct MD5Sum< ::planner::AddRobotMessageRequest>
{
  static const char* value()
  {
    return MD5Sum< ::planner::AddRobotMessage >::value();
  }
  static const char* value(const ::planner::AddRobotMessageRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::planner::AddRobotMessageRequest> should match
// service_traits::DataType< ::planner::AddRobotMessage >
template<>
struct DataType< ::planner::AddRobotMessageRequest>
{
  static const char* value()
  {
    return DataType< ::planner::AddRobotMessage >::value();
  }
  static const char* value(const ::planner::AddRobotMessageRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::planner::AddRobotMessageResponse> should match
// service_traits::MD5Sum< ::planner::AddRobotMessage >
template<>
struct MD5Sum< ::planner::AddRobotMessageResponse>
{
  static const char* value()
  {
    return MD5Sum< ::planner::AddRobotMessage >::value();
  }
  static const char* value(const ::planner::AddRobotMessageResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::planner::AddRobotMessageResponse> should match
// service_traits::DataType< ::planner::AddRobotMessage >
template<>
struct DataType< ::planner::AddRobotMessageResponse>
{
  static const char* value()
  {
    return DataType< ::planner::AddRobotMessage >::value();
  }
  static const char* value(const ::planner::AddRobotMessageResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PLANNER_MESSAGE_ADDROBOTMESSAGE_H
