#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <signal.h>
#include <numeric>

#include "pure_pursuit/cpprobotics_types.h"
#include "pure_pursuit/cubic_spline.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

struct point{
    double x;
    double y;
    double z;
    double yaw;
};

sig_atomic_t volatile g_request_shutdown = 0;

void mySignIntHandler(int sig){
    g_request_shutdown = 1;
}

class PurePursuitController
{
    public:
        PurePursuitController();
        void poseCallback(const nav_msgs::Odometry &currentWaypoint);
        void velocityCallback(const geometry_msgs::Twist &carWaypoint);
        void pointCallback(const nav_msgs::Path &msg);
        void getWayPoints(std::string fileName);
        void run();
        bool isWayPtAwayFromDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);

    private:
        ros::NodeHandle n, nPrivate;
        ros::Publisher purepersuit_;
        ros::Publisher path_pub_, referencePathPub;
        ros::Subscriber splinePath, carVel, carPose;
        nav_msgs::Path path, referPath;
        std::string topicTrajectory, topicCmdVel, topicCurrentPose, filePath;
        float Ld;   //轴距
        float PREVIEW_DIS;  //预瞄距离
        float carVelocity = 0;
        float preview_dis = 0;
        float k = 0.1;
        int pointNum;  //保存路径点的个数
        int startIdx; //开始循迹的下标
        int middleIdx; //距离中值的下标
        int maxIdx; //距离最大值的下标
        bool initTrajectory = false;
        bool directionMatch = false;
        float midDistance; // 中值距离大小
        float maxDis; //最大值距离
        float linearVel; //线速度
        cpprobotics::Vec_f r_x_;
        cpprobotics::Vec_f r_y_;
        bool isReverse;  //是否轨迹反转
};