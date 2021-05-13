#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "param_sample");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  XmlRpc::XmlRpcValue member_list;
  pnh.getParam("member_list", member_list);
  ROS_ASSERT(member_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_INFO("member size: %i", (int)member_list.size());
  for (int32_t i = 0; i < member_list.size(); ++i)
  {
    ROS_INFO("read [%i]", i);
    int id = 0;
    std::string name = "";
    if (!member_list[i]["id"].valid() || !member_list[i]["name"].valid())
    {
      ROS_WARN("No id or name");
      continue;
    }
    if (member_list[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      id = static_cast<int>(member_list[i]["id"]);
    if (member_list[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString)
      name = static_cast<std::string>(member_list[i]["name"]);
    ROS_INFO("[%i] id: %i, name: %s", i, id, name.c_str());
  }
  return 0;
}
