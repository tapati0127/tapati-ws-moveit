#include<ros/ros.h>
#include<moveit/robot_model_loader/robot_model_loader.h>
#include<moveit_visual_tools/moveit_visual_tools.h>
#include<moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"pose_goal_node");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string PLANNING_GROUP ("moto_mini");
    const std::string BASE_FRAME ("base_link");

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
    motomini_visual_tools.publishRobotState(motomini_scene_ptr->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
    motomini_visual_tools.trigger();
    
    std::vector<double> joint_state;
    motomini_state_ptr->copyJointGroupPositions(motomini_joint_group,joint_state);
    joint_state.front() += 1.57;
    joint_state.at(2) += 0.5;
    
    motomini_state_ptr->setJointGroupPositions(motomini_joint_group,joint_state);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(*motomini_state_ptr.get(), motomini_joint_group);
    
    // A tolerance of 0.01 m is specified in position
    // and 0.01 radians in orientation
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    // We will now construct a loader to load a planner, by name.
    // Note that we are using the ROS pluginlib library here.
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    // We will get the name of planning plugin we want to load
    // from the ROS parameter server, and then load the planner
    // making sure to catch all exceptions.
    if (!nh.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(motomini_model_ptr, nh.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
    }
    
    // Make a  MotionPlan request
    planning_interface::MotionPlanRequest req;
    //We need to pass the planning group and the goal condtraints to our request.
    req.group_name = PLANNING_GROUP;
    req.allowed_planning_time = 1.0;
    req.goal_constraints.push_back(joint_goal);

    //Make a Motion Plan response
    planning_interface::MotionPlanResponse res;
  
    //Construct a planning context and solve that
    planning_interface::PlanningContextPtr context_ptr = planner_instance->getPlanningContext(motomini_scene_ptr,req,res.error_code_);
    context_ptr->solve(res);

    //Now we get Message from the response we have just gotten
    moveit_msgs::MotionPlanResponse res_msg;
    res.getMessage(res_msg);

    //Construct a publisher to display
    ros::Publisher display_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_traj;

    //The pose of the text
    geometry_msgs::Pose text_pose;
    text_pose.position.z = 0.5;
    text_pose.orientation.w = 1;

    //Publish Text, Trajectory Line and the Planning Result
    display_traj.trajectory_start = res_msg.trajectory_start;
    display_traj.trajectory.push_back(res_msg.trajectory);
    motomini_visual_tools.publishText(text_pose,"Joint goal!",rviz_visual_tools::BLUE,rviz_visual_tools::LARGE);
    motomini_visual_tools.publishTrajectoryLine(display_traj.trajectory.back(),motomini_joint_group);
    motomini_visual_tools.trigger();
    display_pub.publish(display_traj);


    return 0;   
}