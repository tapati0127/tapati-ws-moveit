#include<ros/ros.h>
#include<moveit/move_group_interface/move_group_interface.h>
#include<moveit_visual_tools/moveit_visual_tools.h>
#include<rviz_visual_tools/rviz_visual_tools.h>
#include<moveit/robot_model/joint_model_group.h>
#include<moveit/robot_model_loader/robot_model_loader.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc,char** argv)
{
	ros::init(argc,argv,"my_cartesian_path_node");//ROS node init with name <my_control_node>
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	static const std::string PLANNING_GROUP = "moto_mini";
	static const std::string END_EFFECTOR_ = "link_6_t";
	static const std::string FRAME = "base_link";

	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	robot_model_loader::RobotModelLoader robot_model("robot_description");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::core::RobotModelPtr  robotmodelptr = robot_model.getModel();
	robot_state::JointModelGroup* joint_model_group = robotmodelptr->getJointModelGroup(PLANNING_GROUP);
    

	geometry_msgs::PoseStamped current_pose,goal_pose;
	current_pose = move_group.getCurrentPose(END_EFFECTOR_);
    current_pose.pose.position.x += 0.05;
    current_pose.pose.position.y += 0.10;
    current_pose.pose.position.z -= 0.05;
    goal_pose = current_pose;
    std::vector<geometry_msgs::Pose> goal_list;
	goal_list.push_back(goal_pose.pose);

	goal_pose.pose.position.x -= 0.15;
	goal_list.push_back(goal_pose.pose);

	goal_pose.pose.position.z += 0.15;
	goal_list.push_back(goal_pose.pose);
   
    moveit_msgs::RobotTrajectory my_trajectory;
	move_group.setPlanningTime(10.0);
    double cartesian = move_group.computeCartesianPath(goal_list,0.01,0.0,my_trajectory);
	if(cartesian==-1.0)
		ROS_INFO("Fail!");
	else ROS_INFO("OK!");

	for (int i = 0; i < my_trajectory.joint_trajectory.points.size(); i++)
		std::cout << my_trajectory.joint_trajectory.points.at(i);
	
    
	moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
	visual_tools.deleteAllMarkers();

	geometry_msgs::Pose text_position;
	text_position.position.z = 0.5;
	text_position.orientation.w = 1;

	visual_tools.publishText(text_position,"Cartesian Path",rviz_visual_tools::WHITE,rviz_visual_tools::XLARGE);
    visual_tools.publishTrajectoryLine(my_trajectory,joint_model_group);
	visual_tools.trigger();
	
}


