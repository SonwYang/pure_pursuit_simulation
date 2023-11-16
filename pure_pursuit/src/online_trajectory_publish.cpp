#include "pure_pursuit/online_trajectory_publish.h"

OnlineTrajectoryPub::OnlineTrajectoryPub():nhPrivate("~")
{
    nhPrivate.param("topicOdom", topicOdom, std::string("/raw_odom"));
    pose_sub = nh.subscribe(topicOdom.c_str(), 10, &OnlineTrajectoryPub::poseCallback, this);        
    state_pub_ = nh.advertise<nav_msgs::Path>("/purepursuit/trajectory", 10);
}

/*When the node is killed, publishing topic*/
OnlineTrajectoryPub::~OnlineTrajectoryPub()
{
    state_pub_.publish(ros_path_);
    // ROS_INFO("Publishing topic successfully!!!");
    ros::spinOnce();
}

void OnlineTrajectoryPub::poseCallback(const nav_msgs::Odometry::ConstPtr &currentWaypoint) 
{
    auto currentPositionX = currentWaypoint->pose.pose.position.x;
    auto currentPositionY = currentWaypoint->pose.pose.position.y;
    auto currentPositionZ = 0.0;

    //发布轨迹
    ros_path_.header.frame_id = "odom";
    ros_path_.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;

    pose.pose.position.x = currentPositionX;
    pose.pose.position.y = currentPositionY;
    pose.pose.position.z = 0;

    ros_path_.poses.push_back(pose);
    // state_pub_.publish(ros_path_);

    ROS_INFO("( x:%0.15f ,y:%0.15f ,z:%0.15f)",currentPositionX, currentPositionY, currentPositionZ);
}

void OnlineTrajectoryPub::run(){
        ros::Rate rate(10);
        while(!g_request_shutdown){
            ros::spinOnce();
            rate.sleep();
        }

        ros_path_.header.stamp = ros::Time::now();
        ros_path_.header.frame_id = "odom";
        state_pub_.publish(ros_path_);
        // // ros::Duration(2.0);
        ROS_INFO("Publishing topic successfully!!!");
        ros::shutdown();
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"publish_trajectory", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySignIntHandler);
    OnlineTrajectoryPub otp;
    otp.run();
    return 0;
}
