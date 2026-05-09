#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <jaka_controller_tcp/Command.h>
#include <cmath>

const double rad2deg=180/M_PI;
jaka_controller_tcp::Command command_msg;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> ActionServer;

ros::Publisher command_pub;
void executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, ActionServer* as)
{
	ROS_INFO("Get Moveit Planning Result:");
	trajectory_msgs::JointTrajectory msg = goal->trajectory; //get the traj from moveit
	const int points_count = msg.points.size();
	if (points_count == 0 || msg.points.front().positions.size() < 6) {
        ROS_WARN("[L] Received invalid trajectory");
        as->setAborted();
        return;
    }

	command_msg.joint.assign(6, 0.0);
	ROS_INFO("Start Spline Intercept");
	ROS_INFO("[L] Received trajectory points: %d", points_count);
	ros::Rate loop_rate1(20); 
	for (int j=0; j <points_count; j++)
	{
		const auto& position = msg.points[j].positions;
		command_msg.joint[0]=position[0]*10000.0/4.0*1000.0;
		command_msg.joint[1]=position[1]*10000.0/4.75*1000.0;
		command_msg.joint[2]=position[2]*10000.0/2.0*1000.0;
		command_msg.joint[3]=position[3]*rad2deg*1000.0;
		command_msg.joint[4]=position[4]*rad2deg*1000.0*5.0/3.0;
		command_msg.joint[5]=position[5]*rad2deg*1000.0*20.0/9.0-command_msg.joint[4];
		command_msg.io=0;
		command_pub.publish(command_msg);             
		loop_rate1.sleep();
	}
	command_msg.io=1;
	command_pub.publish(command_msg);
	as->setSucceeded();
	ros::Duration(0.5).sleep();
	ROS_INFO("SERVOJ DISABLED");
}
int main(int argc, char** argv)
{

	ros::init(argc, argv, "jaka_control_node_L");
	ros::NodeHandle nh;
	command_pub=nh.advertise<jaka_controller_tcp::Command>("command_L",1);
	//Start the ActionServer for JointTrajectoryActions and GripperCommandActions from MoveIT
	//ActionServer action_server(nh, "arm_controller/follow_joint_trajectory", boost::bind(&executeTrajectory, _1, &action_server), false);
  	ActionServer action_server(nh, "/Double_arm_robot/l_arm/follow_joint_trajectory", boost::bind(&executeTrajectory, _1, &action_server), false);
	ROS_INFO("TrajectoryActionServer: Starting");
  	action_server.start();
    ros::spin();
	return(0);
}
