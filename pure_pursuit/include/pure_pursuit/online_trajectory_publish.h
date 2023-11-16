#include <ros/ros.h>
#include "turtlesim/Pose.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <signal.h>
#include "nav_msgs/Odometry.h"

sig_atomic_t volatile g_request_shutdown = 0;

void mySignIntHandler(int sig){
    g_request_shutdown = 1;
}

class OnlineTrajectoryPub{
    public:
        OnlineTrajectoryPub();
        ~OnlineTrajectoryPub();
        void poseCallback(const nav_msgs::Odometry::ConstPtr &currentWaypoint);
        void run();
    private:
        ros::NodeHandle nh, nhPrivate;
        std::string topicOdom;
        ros::Publisher state_pub_;
        nav_msgs::Path ros_path_;
        ros::Subscriber pose_sub;
};


