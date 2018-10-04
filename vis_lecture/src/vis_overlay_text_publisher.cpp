#include "ros/ros.h"
  
#include "math.h"
#include "jsk_rviz_plugins/OverlayText.h"

#include <string>
#include <iostream>
#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vis_overlay_text_publisher");
	ros::NodeHandle n;

	//publisher
	ros::Publisher text_pub = n.advertise<jsk_rviz_plugins::OverlayText>("text", 1);	

	ros::Rate loop_rate(2);
	while (ros::ok()){
		jsk_rviz_plugins::OverlayText text;
		text.action=jsk_rviz_plugins::OverlayText::ADD;
		text.width=400;
		text.height=100;
		text.left=0;
		text.top=0;
		
		std_msgs::ColorRGBA color1, color2;
	    color1.r = 0;
    	color1.g = 0;
    	color1.b = 0;
    	color1.a = 0.4;
		text.bg_color=color1;
		
		color2.r = 25.0/255;
    	color2.g = 255.0/255;
    	color2.b = 240.0/255;
    	color2.a = 0.8;
		text.fg_color=color2;
		
		text.line_width=1;
		text.text_size=14;
		text.font="Ubuntu";
		text.text="hello";

		text_pub.publish(text);
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}
