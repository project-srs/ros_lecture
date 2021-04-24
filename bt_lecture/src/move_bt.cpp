#include <unistd.h>
#include <deque>
#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>

#include "navigation_node.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_bt");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::deque<geometry_msgs::PoseStamped> pose_queue;
  ros::Subscriber goal_sub =
      nh.subscribe<geometry_msgs::PoseStamped>("user_goal", 1, [&](const geometry_msgs::PoseStampedConstPtr pose) {
        std::cout << "pose_queue push_back " << std::endl;
        pose_queue.push_back(*pose);
      });

  Client client("/dtw_robot1/move_base", true);
  move_base_msgs::MoveBaseGoal current_goal;
  bool sent_goal = false;

  geometry_msgs::PoseStamped home_pose;
  home_pose.header.frame_id = "dtw_robot1/map";
  home_pose.pose.position.x = 0.0;
  home_pose.pose.position.y = 0.0;
  home_pose.pose.orientation.w = 1.0;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_(tfBuffer_);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<MoveObject>("MoveObject");
  factory.registerNodeType<ReturnObject>("ReturnObject");
  factory.registerNodeType<WaitObject>("WaitObject");
  factory.registerNodeType<StandbyObject>("StandbyObject");

  Blackboard::Ptr my_blackboard = Blackboard::create();
  my_blackboard->set("pose_queue_size", (unsigned int)0);
  my_blackboard->set("move_base_finished", false);
  my_blackboard->set("move_base_idle", false);
  my_blackboard->set("home_pose_far", false);
  my_blackboard->set("action", "NONE");
  my_blackboard->debugMessage();

  std::string bt_filepath = "default_tree.xml";
  pnh.getParam("bt_filepath", bt_filepath);
  ROS_INFO("tree file: %s\n", bt_filepath.c_str());
  auto tree = factory.createTreeFromFile(bt_filepath, my_blackboard);
  PublisherZMQ publisher_zmq(tree);
  FileLogger logger_file(tree, "bt_trace.fbl");

  while (ros::ok())
  {
    // update input
    bool move_base_finished = false;
    bool move_base_idle = false;
    if (client.isServerConnected())
    {
      switch (client.getState().state_)
      {
        case actionlib::SimpleClientGoalState::REJECTED:
        case actionlib::SimpleClientGoalState::PREEMPTED:
        case actionlib::SimpleClientGoalState::ABORTED:
        case actionlib::SimpleClientGoalState::LOST:
          move_base_idle = true;
          break;
        case actionlib::SimpleClientGoalState::SUCCEEDED:
          move_base_finished = true;
          move_base_idle = true;
          break;
        case actionlib::SimpleClientGoalState::PENDING:
        case actionlib::SimpleClientGoalState::ACTIVE:
        case actionlib::SimpleClientGoalState::RECALLED:
        default:
          break;
      }
    }

    bool home_distance_far = false;
    try
    {
      geometry_msgs::TransformStamped transformStamped;
      transformStamped = tfBuffer_.lookupTransform("dtw_robot1/map", "dtw_robot1/base_link", ros::Time(0));
      auto& trans = transformStamped.transform.translation;
      float home_dx = (trans.x - home_pose.pose.position.x);
      float home_dy = (trans.y - home_pose.pose.position.y);
      home_distance_far = 0.2 < sqrt(home_dx * home_dx + home_dy * home_dy);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }

    my_blackboard->set("pose_queue_size", (unsigned int)pose_queue.size());
    my_blackboard->set("move_base_finished", move_base_finished);
    my_blackboard->set("move_base_idle", move_base_idle);
    my_blackboard->set("home_pose_far", home_distance_far);

    // bt
    auto result = tree.tickRoot();
    std::string action;
    my_blackboard->get(std::string("action"), action);

    // output
    if (client.isServerConnected())
    {
      if (action == "RETURN_COMMAND")
      {
        current_goal.target_pose = home_pose;
        client.sendGoal(current_goal);
        ROS_INFO("RETURN");
      }
      else if (action == "MOVE_COMMAND")
      {
        if (!pose_queue.empty())
        {
          auto top = pose_queue.front();
          pose_queue.pop_front();
          current_goal.target_pose = top;
          client.sendGoal(current_goal);
          ROS_INFO("MOVE");
        }
      }
    }
    ros::spinOnce();
    sleep(1);
  }
  return 0;
}