#include <ros/ros.h>
#include <adv_lecture/TaskAction.h>
#include <actionlib/client/simple_action_client.h>

#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>

struct termios save_settings;
void set_input_interactive(void){
  //set no-echo-back and non-blocking
	struct termios settings;

	tcgetattr(0,&save_settings);
	settings = save_settings;

	settings.c_lflag &= ~(ECHO|ICANON);
	settings.c_cc[VTIME] = 0;
	settings.c_cc[VMIN] = 1;
	tcsetattr(0,TCSANOW,&settings);
	fcntl(0,F_SETFL,O_NONBLOCK);

}
void reset_input(void){
  tcsetattr(0,TCSANOW,&save_settings);
}

typedef actionlib::SimpleActionClient<adv_lecture::TaskAction> Client;

int main(int argc, char** argv){
  ros::init(argc, argv, "do_dishes_client");
  ros::NodeHandle nh;
  set_input_interactive();

  Client client("do_dishes", true);
  
  printf("a: send goal (in 1s)\n");
  printf("s: send goal (in 5s)\n");
  printf("c: cancel goal\n");  	
  printf("q: quit\n");  	

  int task_id=0;
  ros::Rate loop_rate(2);
	while(ros::ok()){
    if(client.isServerConnected()){
      char c = getchar();
      
      if (c == 'a'){
        adv_lecture::TaskGoal goal;
        goal.task_id=task_id;
        task_id++;
        goal.duration=1.0;
        client.sendGoal(goal);
        printf("publish goal id:%i, duration:%f\n", goal.task_id, goal.duration);
      }
      else if (c == 's'){
        adv_lecture::TaskGoal goal;
        goal.task_id=task_id;
        task_id++;
        goal.duration=5.0;
        client.sendGoal(goal);
        printf("publish goal id:%i, duration:%f\n", goal.task_id, goal.duration);
      }
      else if (c == 'c'){
        client.cancelGoal();
        printf("publish cancel\n");
      }
      else if (c == 'q'){
        break;
      }
      printf("Current State: %s\n", client.getState().toString().c_str());
    }
    fflush(stdout);
		ros::spinOnce();
		loop_rate.sleep();
	}
  reset_input();
	
	return 0;
}