#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <string.h>
int main(int argc, char** argv)
{
  ros::init(argc, argv,
            "my_control_node"); // ROS node init with name <my_control_node>
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  move_group.setPoseReferenceFrame("base_frame");
  ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());

  geometry_msgs::PoseStamped current_pose, goal_pose;
  current_pose = move_group.getCurrentPose("tool0");

  ROS_INFO("This is the current pose:");
  ROS_INFO("Frame ID:");
  std::cout << current_pose.header.frame_id << std::endl;
  ROS_INFO("Position:");
  std::cout << current_pose.pose.position << std::endl;
  ROS_INFO("Orientation:");
  std::cout << current_pose.pose.orientation << std::endl;

  // 	moveit::planning_interface::MoveGroupInterface::Plan myplan;
  // 	goal_pose = move_group.getRandomPose();
  // //	move_group.setApproximateJointValueTarget(goal_pose,"tool0");
  // 	move_group.setJointValueTarget(goal_pose,"tool0");

  // 	ROS_INFO("This is the goal pose:");
  // 	ROS_INFO("Frame ID:");
  // 	std::cout << goal_pose.header.frame_id << std::endl;
  // 	ROS_INFO("Position:");
  // 	std::cout << goal_pose.pose.position << std::endl;
  // 	ROS_INFO("Orientation:");
  // 	std::cout << goal_pose.pose.orientation << std::endl;

  // 	ROS_INFO("Planning result:");
  // 	if(move_group.plan(myplan)==moveit::planning_interface::MoveItErrorCode::SUCCESS)
  // 		{
  // 		ROS_INFO("OK, You can plan now");
  // 		}
  // 	else ROS_INFO("Oh no! Something got wrongs!");

  move_group.setRandomTarget();
  move_group.move();
}
