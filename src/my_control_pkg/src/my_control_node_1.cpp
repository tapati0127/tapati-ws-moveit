#include<ros/ros.h>
#include<moveit/move_group_interface/move_group_interface.h>
#include<geometry_msgs/PoseStamped.h>
#include<moveit_visual_tools/moveit_visual_tools.h>
#include<rviz_visual_tools/rviz_visual_tools.h>
#include<moveit/robot_model/joint_model_group.h>
#include<moveit/robot_model_loader/robot_model_loader.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc,char** argv)
{
	ros::init(argc,argv,"my_control_node_1");//ROS node init with name <my_control_node>
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	static const std::string PLANNING_GROUP = "moto_mini";
	static const std::string END_EFFECTOR_ = "link_6_t";
	static const std::string FRAME = "base_link";
	// static const std::string PLANNING_GROUP = "panda_arm";
	// static const std::string END_EFFECTOR_ = "panda_link8";
	// static const std::string FRAME = "world";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	// move_group.setEndEffector("end_effector");
	
	//move_group.setGoalTolerance(0.5);

	

	robot_model_loader::RobotModelLoader robot_model("robot_description");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	moveit::core::RobotModelPtr  robotmodelptr = robot_model.getModel();
	robot_state::JointModelGroup* joint_model_group = robotmodelptr->getJointModelGroup(PLANNING_GROUP);

	std::cout << "Get End-Effector Name:" << std::endl;
	std::cout << joint_model_group->getEndEffectorName() << std::endl;
	
	// We can print the name of the reference frame for this robot.
  	ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

 	 // We can also print the name of the end-effector link for this group.
  	ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

 	 // We can get a list of all the groups in the robot:
  	ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  	std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),

            std::ostream_iterator<std::string>(std::cout, ", "));


	geometry_msgs::PoseStamped current_pose,goal_pose;
	current_pose = move_group.getCurrentPose(END_EFFECTOR_);

	ROS_INFO("This is the current pose:");
	ROS_INFO("Frame ID:");
	std::cout << current_pose.header.frame_id << std::endl;
	ROS_INFO("Position:");
	std::cout << current_pose.pose.position << std::endl;
	ROS_INFO("Orientation:");
	std::cout << current_pose.pose.orientation << std::endl;


	moveit::planning_interface::MoveGroupInterface::Plan myplan;
	goal_pose = move_group.getRandomPose(END_EFFECTOR_);

	move_group.setJointValueTarget(goal_pose,END_EFFECTOR_);

	ROS_INFO("This is the goal pose:");
	ROS_INFO("Frame ID:");
	std::cout << goal_pose.header.frame_id << std::endl;
	ROS_INFO("Position:");
	std::cout << goal_pose.pose.position << std::endl;
	ROS_INFO("Orientation:");
	std::cout << goal_pose.pose.orientation << std::endl;
	
	move_group.setPlanningTime(10.0);
	ROS_INFO("Planning result:");
	if(move_group.plan(myplan)==moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
		ROS_INFO("OK, Planned");
		}
	else ROS_INFO("Oh no! Something got wrongs!");

	
	moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
	
	
	visual_tools.deleteAllMarkers();

	geometry_msgs::Pose text_position;
	text_position.position.z = 0.5;
	text_position.orientation.w = 1;
	Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  	text_pose.translation().z() = 1.75;



	visual_tools.publishAxisLabeled(current_pose.pose, "pose_start");
	visual_tools.publishAxisLabeled(goal_pose.pose, "pose_goal");
	visual_tools.publishText(text_position,"This is my 2nd program",rviz_visual_tools::WHITE,rviz_visual_tools::XLARGE);
	
	
	visual_tools.publishTrajectoryLine(myplan.trajectory_,joint_model_group,rviz_visual_tools::GREEN);
	
	std::ofstream joint_value;
	joint_value.open("joint_value.txt");
	joint_value << "Joint name:" << std::endl;
	for(int i = 0;i<myplan.trajectory_.joint_trajectory.joint_names.size();i++)
		joint_value << myplan.trajectory_.joint_trajectory.joint_names.at(i) << std::endl;
	joint_value << "Joint value with time:" << std::endl;
	for(int i = 0;i<myplan.trajectory_.joint_trajectory.points.size();i++)
		joint_value << myplan.trajectory_.joint_trajectory.points.at(i) << std::endl;
	joint_value.close();

	std::ofstream end_effector_pose;
	end_effector_pose.open("end_effector_pose.txt");
	end_effector_pose << "The pose of the end-effector:" << std::endl;
	for(int i = 0;i<myplan.trajectory_.multi_dof_joint_trajectory.points.size();i++)
		end_effector_pose << myplan.trajectory_.multi_dof_joint_trajectory.points.at(i) << std::endl;
	end_effector_pose.close();

	visual_tools.trigger();
	
}


