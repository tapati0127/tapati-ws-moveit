#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"fake_motoros_node");
    ros::NodeHandle nh;

    const float joint_goal[6] = {M_PI/3,M_PI/7,M_PI/5,M_PI/6,M_PI/8,M_PI/2};
    const float tf = 3;
    const float f = 50;
    float joint[6];
    float t = 0;
    std_msgs::Float64MultiArray joint_value;
    


    ros::Publisher publisher = nh.advertise<std_msgs::Float64MultiArray>("joint_publisher",1);
    while (publisher.getNumSubscribers() < 1)
    {
    }
    ros::Rate loop(50);
    while(ros::ok)
    {
        while(t<tf)
        {
            joint_value.data.clear();
            for(int i = 0; i < 6; i++)
            {
                joint[i] = 3*joint_goal[i]/pow(tf,2)*pow(t,2)-2*joint_goal[i]/pow(tf,3)*pow(t,3);
                joint_value.data.push_back(joint[i]);
            }
            publisher.publish(joint_value);
            loop.sleep();
            t += 0.02;
        }
        
        
        
        
        
        
        
        
        
    }
}
