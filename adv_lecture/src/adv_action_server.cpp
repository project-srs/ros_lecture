#include <ros/ros.h>
#include <adv_lecture/TaskAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<adv_lecture::TaskAction> Server;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "do_dishes_server");
  ros::NodeHandle nh;
  Server server(nh, "do_dishes", false);
  server.start();

  ros::Time start_time;
  ros::Rate loop_rate(2);
  adv_lecture::TaskGoalConstPtr current_goal;
  while (ros::ok()){
    if(server.isNewGoalAvailable()){
      current_goal=server.acceptNewGoal();
      start_time=ros::Time::now();
      printf("Update Goal\n");
    }

    if(server.isActive()){
      if(server.isPreemptRequested()){
        server.setPreempted();
        printf("Preempt Goal\n");
      }
      else{
        if(start_time+ros::Duration(current_goal->duration)<ros::Time::now()){
          server.setSucceeded();
          //server.setAborted();
          printf("Active: publish result id:%i\n", current_goal->task_id);
        }
        else{
          adv_lecture::TaskFeedback feedback;
          feedback.rate=(ros::Time::now()-start_time).toSec()/current_goal->duration;
          server.publishFeedback(feedback);
          printf("Active: publish feedback id:%i\n", current_goal->task_id);
        }
      }
    }  
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}