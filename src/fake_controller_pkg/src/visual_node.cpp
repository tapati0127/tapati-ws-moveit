#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <time.h>

auto display_msg = new moveit_msgs::DisplayRobotState;
auto pub = new ros::Publisher;

void display (const std_msgs::Float64MultiArray::ConstPtr &joint_msg)
{
    clock_t t0 = clock();
    display_msg->state.joint_state.position = joint_msg->data;
    pub->publish(*display_msg);
    clock_t t = clock();
    std::cout << "Process time: " << (double)(t-t0)/CLOCKS_PER_SEC*1000 << " ms" << std::endl;
}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"visual_node");
    ros::NodeHandle nh;
    display_msg->state.is_diff = 1;
    display_msg->state.joint_state.header.frame_id = "base_link";
    display_msg->state.joint_state.name = {"joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"};
    *pub = nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state",1);
    ros::Subscriber sub = nh.subscribe("joint_publisher",1,display);

    ros::spin();


}
