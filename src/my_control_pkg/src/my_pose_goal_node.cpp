#include<ros/ros.h>
#include<moveit/move_group_interface/move_group_interface.h>
#include<moveit_visual_tools/moveit_visual_tools.h>
#include<rviz_visual_tools/rviz_visual_tools.h>
#include<moveit/robot_model/joint_model_group.h>
#include<moveit/robot_model_loader/robot_model_loader.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc,char** argv)
{
	ros::init(argc,argv,"my_pose_goal_node");//ROS node init with name <my_control_node>
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
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
	visual_tools.deleteAllMarkers();

	geometry_msgs::PoseStamped current_pose;
	std::vector<geometry_msgs::PoseStamped> goal_list;
	current_pose = move_group.getCurrentPose(END_EFFECTOR_);
    current_pose.pose.position.x += 0.03;
    current_pose.pose.position.y += 0.21;
    current_pose.pose.position.z -= 0.05;
    

    const geometry_msgs::PoseStamped goal_pose = current_pose;
	visual_tools.publishAxisLabeled(goal_pose.pose, "goal_pose");

	moveit::planning_interface::MoveGroupInterface::Plan myplan;
    move_group.setPoseTarget(goal_pose,END_EFFECTOR_);
	move_group.setPlanningTime(5.0);
    
    ROS_INFO("Planning result:");
	if(move_group.plan(myplan)==moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
		ROS_INFO("OK, Planned");
		}
	else ROS_INFO("Oh no! Something got wrongs!");

	
	

	geometry_msgs::Pose text_position;
	text_position.position.z = 0.5;
	text_position.orientation.w = 1;
	visual_tools.publishText(text_position,"Moving robot to goal pose!",rviz_visual_tools::WHITE,rviz_visual_tools::XLARGE);
	visual_tools.publishTrajectoryLine(myplan.trajectory_,joint_model_group,rviz_visual_tools::GREEN);
	visual_tools.trigger();
}


