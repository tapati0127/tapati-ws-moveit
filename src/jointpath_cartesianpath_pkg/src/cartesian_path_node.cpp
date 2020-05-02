#include<ros/ros.h>
#include<moveit/robot_model_loader/robot_model_loader.h>
#include<moveit_visual_tools/moveit_visual_tools.h>
#include<moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include<moveit/robot_state/conversions.h>
#include<moveit_msgs/GetCartesianPath.h>
#include<eigen_conversions/eigen_msg.h>

int main(int argc,char** argv)
{
  ros::init(argc,argv,"pose_goal_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  const std::string PLANNING_GROUP ("moto_mini");
  const std::string BASE_FRAME ("base_link");
  const std::string PATH_LINK ("link_6_t");
  //Use RobotModelLoader to look up the robot description
  robot_model_loader::RobotModelLoader motomini_model_loader("robot_description");
  //Construct RobotModel from RobotModelLoader
  robot_model::RobotModelPtr motomini_model_ptr = motomini_model_loader.getModel();
  //Construct RobotState class from RobotModel
  robot_state::RobotStatePtr motomini_state_ptr(new robot_state::RobotState (motomini_model_ptr));
  motomini_state_ptr->setToDefaultValues(); // Set all joints to their default positions.
  //Construct JointModelGroup from RobotModel
  robot_state::JointModelGroup* motomini_joint_group = motomini_model_ptr->getJointModelGroup(PLANNING_GROUP);
  
  //Construct PlanningScene from RobotModel
  planning_scene::PlanningScenePtr motomini_scene_ptr(new planning_scene::PlanningScene (motomini_model_ptr));
  motomini_scene_ptr->getCurrentStateNonConst().setToDefaultValues(motomini_joint_group, "ready"); //Get the state at which the robot is assumed to be.
  //init the moveit_visual_tools to display the marker
  moveit_visual_tools::MoveItVisualTools motomini_visual_tools(BASE_FRAME);
  motomini_visual_tools.loadRobotStatePub("/display_robot_state");
  motomini_visual_tools.enableBatchPublishing();
  motomini_visual_tools.deleteAllMarkers();
  motomini_visual_tools.trigger();
  motomini_visual_tools.loadRemoteControl();
//    
    // 
  // Publish the RobotState
  //motomini_visual_tools.publishRobotState(motomini_scene_ptr->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  //motomini_visual_tools.trigger();
  //motomini_visual_tools.prompt("Published RobotState!!");

  double maxstep = 0.01;
  double jump = 0;

  moveit_msgs::GetCartesianPath srv;

  robot_state::robotStateToRobotStateMsg(*motomini_state_ptr.get(),srv.request.start_state);

  srv.request.group_name = PLANNING_GROUP;
  srv.request.header.frame_id = BASE_FRAME;
  srv.request.jump_threshold = jump;
  srv.request.header.frame_id = BASE_FRAME;
  srv.request.link_name = PATH_LINK;
  srv.request.max_step = maxstep;
  
  Eigen::Isometry3d start_pose =  motomini_state_ptr->getGlobalLinkTransform(PATH_LINK);
  geometry_msgs::Pose start_pose_msg, pose_1, pose_2, pose_3;
  tf::poseEigenToMsg(start_pose,start_pose_msg);
  pose_1.position.x = start_pose_msg.position.x + 0.1;
  pose_1.position.y = start_pose_msg.position.y + 0.1;
  pose_1.position.z = start_pose_msg.position.z;
  pose_2 = pose_1;
  pose_2.position.z = pose_1.position.z - 0.1;
  pose_3 = pose_2;
  pose_3.position.y -= 0.2;
  srv.request.waypoints.push_back(pose_1);
  srv.request.waypoints.push_back(pose_2);
  srv.request.waypoints.push_back(pose_3);



  ros::ServiceClient cartesian_client = nh.serviceClient<moveit_msgs::GetCartesianPath>("compute_cartesian_path");
  if (cartesian_client.call(srv)!=true)
    ROS_ERROR("Cannot call the srv.");

  if(!srv.response.error_code.SUCCESS)
    ROS_ERROR("Cannot compute the cartesian path.");

  std::cout << "The fraction: " << srv.response.fraction << std::endl;
 

  moveit_msgs::DisplayTrajectory display_traj;
  display_traj.trajectory_start = srv.response.start_state;
  display_traj.model_id = motomini_model_ptr->getName();
  display_traj.trajectory.push_back(srv.response.solution);


  ros::Publisher display_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  display_pub.publish(display_traj);
  motomini_visual_tools.publishTrajectoryLine(srv.response.solution,motomini_model_ptr->getLinkModel("tool0"),motomini_joint_group);
  motomini_visual_tools.trigger();

  // std::vector <robot_state::RobotStatePtr> motomini_state_ptr_vector;
  // for(int i = 0; i <  srv.response.solution.joint_trajectory.points.size();i++)
  // {
    // motomini_state_ptr->setJointGroupPositions(motomini_joint_group,srv.response.solution.joint_trajectory.points.at(i).positions);
    // motomini_state_ptr_vector.push_back(motomini_state_ptr);
  // }
    // 
  // bool a = motomini_visual_tools.publishTrajectoryPoints(motomini_state_ptr_vector,motomini_model_ptr->getLinkModel("tool0"));
  // motomini_visual_tools.trigger();

    

  

  return 0;   
}