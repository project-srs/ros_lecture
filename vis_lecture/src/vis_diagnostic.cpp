#include "ros/ros.h"
#include "std_msgs/String.h"
#include "diagnostic_updater/diagnostic_updater.h"

int diagnostic_counter=0;
void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat){
	if(diagnostic_counter%30<10){
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "publish OK");
    }
    else if(diagnostic_counter%30<20){
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "publish Warn");
    }
	else{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "publish Error");
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "vis_diagnostic");
	ros::NodeHandle n;
	
	//Diagnostic
	diagnostic_updater::Updater updater;
	updater.setHardwareID("DummyDevice");
	updater.add("Dummy", diagnostic0);
	
	ros::Rate loop_rate(10); 
	while (ros::ok()){
		updater.update();	
    	ros::spinOnce();
		loop_rate.sleep();
        diagnostic_counter++;
	} 
 	return 0;
}

