#include "ros/ros.h"
  
#include "math.h"
#include "jsk_rviz_plugins/OverlayMenu.h"

#include <string>
#include <iostream>
#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vis_overlay_menu_publisher");
	ros::NodeHandle n;

	//publisher
	ros::Publisher menu_pub = n.advertise<jsk_rviz_plugins::OverlayMenu>("menu", 1);	

	ros::Rate loop_rate(2);
	bool flip=false;
	while (ros::ok()){
		jsk_rviz_plugins::OverlayMenu menu;
		menu.action=jsk_rviz_plugins::OverlayMenu::ACTION_SELECT;
		if(flip)menu.current_index=0;
		else    menu.current_index=1;
		menu.menus.resize(2);
		menu.menus[0]="item0";
		menu.menus[1]="item1";
		menu.title="Sample Menu";
		menu_pub.publish(menu);
		flip=!flip;
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}
