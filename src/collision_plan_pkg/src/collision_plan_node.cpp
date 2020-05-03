#include<ros/ros.h>
#include<moveit/robot_model_loader/robot_model_loader.h>
#include<moveit_visual_tools/moveit_visual_tools.h>
#include<moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<eigen_conversions/eigen_msg.h>
#include<moveit_msgs/GetMotionPlan.h>
#include<moveit/robot_state/conversions.h>

int main(int argc,char** argv)
{
  ros::init(argc,argv,"collision_plan_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  const std::string PLANNING_GROUP ("moto_mini");
  const std::string BASE_FRAME ("base_link");
  const std::string END_EFFECTOR ("tool0");
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
  motomini_visual_tools.prompt("Next");

  std::vector<double> gstate;
  motomini_state_ptr->copyJointGroupPositions(motomini_joint_group,gstate);
  Eigen::Isometry3d tranform = motomini_state_ptr->getGlobalLinkTransform(END_EFFECTOR);
  geometry_msgs::Pose start_pose, goal_pose;
  tf::poseEigenToMsg(tranform,start_pose);
  goal_pose=start_pose;
  start_pose.position.y-=0.1;
  //Set RobotState by computing inverse kinematics (start_state)
  motomini_state_ptr->setFromIK(motomini_joint_group,start_pose);
  moveit_msgs::RobotState start_state_msg;
  robot_state::robotStateToRobotStateMsg(*motomini_state_ptr.get(),start_state_msg);
  goal_pose.position.y+=0.1;
  //Difine a pose goal;
  geometry_msgs::PoseStamped stamped_start,stamped_goal;
  stamped_start.header.frame_id = BASE_FRAME;
  stamped_goal.header.frame_id = BASE_FRAME;
  stamped_start.pose=start_pose;
  stamped_goal.pose=goal_pose;
  // tf2::Quaternion qtn;
  // qtn.setRPY(1.02,3.14,-0.6);
  // tf2::convert(qtn,pose_stamped.pose.orientation);
   
  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);
  moveit_msgs::GetMotionPlan srv;
  //We need to pass the planning group and the goal condtraints to our request.
  srv.request.motion_plan_request.group_name = PLANNING_GROUP;
  srv.request.motion_plan_request.start_state = start_state_msg;
  srv.request.motion_plan_request.allowed_planning_time = 5.0;
  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(END_EFFECTOR,stamped_goal); 
  srv.request.motion_plan_request.goal_constraints.push_back(pose_goal);

  ros::ServiceClient client = nh.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");
  client.call(srv);

  //Construct a publisher to display
  ros::Publisher display_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_traj;

  //The pose of the text
  geometry_msgs::Pose text_pose;
  text_pose.position.z = 0.5;
  text_pose.orientation.w = 1;

  //Publish Text, Trajectory Line and the Planning Result
  display_traj.trajectory_start = srv.response.motion_plan_response.trajectory_start;
  display_traj.model_id = motomini_model_ptr->getName();
  display_traj.trajectory.push_back(srv.response.motion_plan_response.trajectory);
  motomini_visual_tools.publishText(text_pose,"Pose goal!",rviz_visual_tools::BLUE,rviz_visual_tools::LARGE);
  motomini_visual_tools.publishAxisLabeled(stamped_start.pose,"Start");
  motomini_visual_tools.publishAxisLabeled(stamped_goal.pose,"Goal");
  motomini_visual_tools.publishTrajectoryLine(display_traj.trajectory.back(),motomini_model_ptr->getLinkModel(END_EFFECTOR),motomini_joint_group);
  motomini_visual_tools.trigger();
  display_pub.publish(display_traj);

  /*Let's start with collision planning*/
  motomini_visual_tools.prompt("Next");
  motomini_visual_tools.deleteAllMarkers();
  motomini_visual_tools.trigger();
  // Advertise the required topic
  ros::Publisher planning_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }
  //First define object
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = END_EFFECTOR;
  attached_object.object.header.frame_id = END_EFFECTOR;
  attached_object.object.id="box"; 

  // A default pose
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;

  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.04;
  primitive.dimensions[1] = 0.04;
  primitive.dimensions[2] = 0.04;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  // Since we are attaching the object to the robot hand to simulate picking up the object,
  // we want the collision checker to ignore collisions between the object and the robot hand.
  attached_object.touch_links = std::vector<std::string>{"tool0"};
  // Note that attaching an object to the robot requires
  // the corresponding operation to be specified as an ADD operation.
  attached_object.object.operation = attached_object.object.ADD;

  // Add an object into the environment
  ROS_INFO("Adding the object into the world at the location of the hand.");
  moveit_msgs::PlanningScene planning_scene_msg;
  planning_scene_msg.world.collision_objects.push_back(attached_object.object);
  planning_scene_msg.is_diff = true;

  planning_scene_publisher.publish(planning_scene_msg);
  motomini_visual_tools.prompt("Next");

  //motomini_visual_tools.trigger();

  //Plan with new planning scene
  client.call(srv);
  //Publish Text, Trajectory Line and the Planning Result
  display_traj.trajectory_start = srv.response.motion_plan_response.trajectory_start;
  display_traj.model_id = motomini_model_ptr->getName();
  display_traj.trajectory.clear();
  display_traj.trajectory.push_back(srv.response.motion_plan_response.trajectory);
  motomini_visual_tools.publishText(text_pose,"Collision-free!",rviz_visual_tools::BLUE,rviz_visual_tools::LARGE);
  motomini_visual_tools.publishAxisLabeled(stamped_start.pose,"Start");
  motomini_visual_tools.publishAxisLabeled(stamped_goal.pose,"Goal");
  motomini_visual_tools.publishTrajectoryLine(display_traj.trajectory.back(),motomini_model_ptr->getLinkModel(END_EFFECTOR),motomini_joint_group);
  motomini_visual_tools.trigger();
  display_pub.publish(display_traj);

  motomini_visual_tools.prompt("Next");
  motomini_visual_tools.deleteAllMarkers();
  motomini_visual_tools.trigger();
  /*Now attach an object to the robot*/
  //  * Removing the original object from the environment
  /* First, define the REMOVE object message*/
  moveit_msgs::CollisionObject remove_object;
  remove_object.id = "box";
  remove_object.header.frame_id = BASE_FRAME;
  remove_object.operation = remove_object.REMOVE;

  // Note how we make sure that the diff message contains no other
  // attached objects or collisions objects by clearing those fields
  // first.
  /* Carry out the REMOVE + ATTACH operation */
  ROS_INFO("Attaching the object to the hand and removing it from the world.");
  planning_scene_msg.world.collision_objects.clear();
  planning_scene_msg.world.collision_objects.push_back(remove_object);
  planning_scene_msg.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene_publisher.publish(planning_scene_msg);

  motomini_visual_tools.prompt("Next");
  //OK the object was attached to the robot, now we plan the robot again
  client.call(srv);
  //Publish Text, Trajectory Line and the Planning Result
  display_traj.trajectory_start = srv.response.motion_plan_response.trajectory_start;
  display_traj.model_id = motomini_model_ptr->getName();
  display_traj.trajectory.clear();
  display_traj.trajectory.push_back(srv.response.motion_plan_response.trajectory);
  motomini_visual_tools.publishText(text_pose,"Collision-free!",rviz_visual_tools::BLUE,rviz_visual_tools::LARGE);
  motomini_visual_tools.publishAxisLabeled(stamped_start.pose,"Start");
  motomini_visual_tools.publishAxisLabeled(stamped_goal.pose,"Goal");
  motomini_visual_tools.publishTrajectoryLine(display_traj.trajectory.back(),motomini_model_ptr->getLinkModel(END_EFFECTOR),motomini_joint_group);
  motomini_visual_tools.trigger();
  display_pub.publish(display_traj);
  return 0;   
}