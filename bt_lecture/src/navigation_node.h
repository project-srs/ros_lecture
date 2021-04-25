#include <vector>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>

using namespace BT;

class MoveObject : public BT::CoroActionNode
{
public:
  MoveObject(const std::string& name, const NodeConfiguration& config) : BT::CoroActionNode(name, config)
  {
    std::cout << this->name() << ": constructor" << std::endl;
  }

  static PortsList providedPorts()
  {
    return { InputPort<bool>("move_base_finished"), InputPort<bool>("move_base_idle"),
             InputPort<bool>("pose_queue_size"), OutputPort<std::string>("action") };
  }

  BT::NodeStatus tick() override
  {
    auto pose_queue_size = getInput<bool>("pose_queue_size");
    if (!pose_queue_size || pose_queue_size.value() == 0)
    {
      return BT::NodeStatus::FAILURE;
    }

    setOutput<std::string>("action", "MOVE_COMMAND");
    std::cout << name() << ": MOVE_COMMAND Yield" << std::endl;
    setStatusRunningAndYield();

    while (true)
    {
      auto move_base_idle = getInput<bool>("move_base_idle");

      auto move_base_finished = getInput<bool>("move_base_finished");
      if (move_base_finished && move_base_finished.value())
      {
        std::cout << name() << ": move_base is finidshed: SUCCESS" << std::endl;
        setOutput<std::string>("action", "NONE");
        return BT::NodeStatus::SUCCESS;
      }

      if (move_base_idle && move_base_idle.value())
      {
        std::cout << name() << ": move_base is idle: FAILURE" << std::endl;
        setOutput<std::string>("action", "NONE");
        return BT::NodeStatus::FAILURE;
      }

      setOutput<std::string>("action", "NONE");
      setStatusRunningAndYield();
    }
  }

  void halt() override
  {
    std::cout << this->name() << ": halt" << std::endl;
    CoroActionNode::halt();
  }
};

class ReturnObject : public BT::CoroActionNode
{
public:
  ReturnObject(const std::string& name, const NodeConfiguration& config) : BT::CoroActionNode(name, config)
  {
    std::cout << this->name() << ": constructor" << std::endl;
  }

  static PortsList providedPorts()
  {
    return { InputPort<bool>("move_base_finished"), InputPort<bool>("move_base_idle"), InputPort<bool>("home_pose_far"),
             OutputPort<std::string>("action") };
  }

  BT::NodeStatus tick() override
  {
    auto home_pose_far = getInput<bool>("home_pose_far");
    if (!home_pose_far || !home_pose_far.value())
    {
      return BT::NodeStatus::FAILURE;
    }

    setOutput<std::string>("action", "RETURN_COMMAND");
    std::cout << name() << ": RETURN_COMMAND Yield" << std::endl;
    setStatusRunningAndYield();

    while (true)
    {
      auto move_base_idle = getInput<bool>("move_base_idle");

      auto move_base_finished = getInput<bool>("move_base_finished");
      if (move_base_finished && move_base_finished.value())
      {
        std::cout << name() << ": move_base is finidshed: SUCCESS" << std::endl;
        setOutput<std::string>("action", "NONE");
        return BT::NodeStatus::SUCCESS;
      }

      if (move_base_idle && move_base_idle.value())
      {
        std::cout << name() << ": move_base is idle: FAILURE" << std::endl;
        setOutput<std::string>("action", "NONE");
        return BT::NodeStatus::FAILURE;
      }

      setOutput<std::string>("action", "NONE");
      setStatusRunningAndYield();
    }
  }

  void halt() override
  {
    std::cout << this->name() << ": halt" << std::endl;
    CoroActionNode::halt();
  }
};

class WaitObject : public BT::CoroActionNode
{
public:
  WaitObject(const std::string& name, const NodeConfiguration& config) : BT::CoroActionNode(name, config)
  {
    std::cout << this->name() << ": constructor" << std::endl;
  }

  static PortsList providedPorts()
  {
    return { InputPort<int>("wait_duration"), OutputPort<std::string>("action") };
  }

  BT::NodeStatus tick() override
  {
    std::cout << name() << ": tick start" << std::endl;
    float wait_duration = 1.0;
    auto input_wait_duration = getInput<int>("wait_duration");
    if (wait_duration)
    {
      wait_duration = input_wait_duration.value();
      std::cout << name() << ": wait " << wait_duration << " s" << std::endl;
    }

    ros::Time initial_time = ros::Time::now();
    ros::Time end_time = initial_time + ros::Duration(wait_duration);

    while (true)
    {
      ros::Time now = ros::Time::now();
      if (end_time <= now)
      {
        std::cout << name() << ": break loop " << std::endl;
        break;
      }
      setOutput<std::string>("action", "NONE");
      setStatusRunningAndYield();
    }
    std::cout << name() << ": tick end" << std::endl;
    setOutput<std::string>("action", "NONE");
    return BT::NodeStatus::SUCCESS;
  }
  void halt() override
  {
    std::cout << this->name() << ": halt" << std::endl;
    CoroActionNode::halt();
  }
};

class StandbyObject : public BT::SyncActionNode
{
public:
  StandbyObject(const std::string& name, const NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
    std::cout << this->name() << ": constructor" << std::endl;
  }

  static PortsList providedPorts()
  {
    return { OutputPort<std::string>("action") };
  }

  BT::NodeStatus tick() override
  {
    setOutput<std::string>("action", "NONE");
    return BT::NodeStatus::SUCCESS;
  }
};
