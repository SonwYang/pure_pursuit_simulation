#include "pure_pursuit/purepursuit.h"

using namespace std;
using namespace cpprobotics;

PurePursuitController::PurePursuitController():nPrivate("~")
{ 
    /*params setting*/
    nPrivate.param("Ld", Ld, float(0.3));
    nPrivate.param("PREVIEW_DIS", PREVIEW_DIS, float(2.0));
    nPrivate.param("linearVel", linearVel, float(0.3));
    nPrivate.param("isReverse", isReverse, bool(false));
    nPrivate.param("topicTrajectory", topicTrajectory, std::string("/purepursuit/trajectory"));
    nPrivate.param("topicCmdVel", topicCmdVel, std::string("/cmd_vel"));
    nPrivate.param("topicCurrentPose", topicCurrentPose, std::string("/orb_slam3/body_odom"));
    nPrivate.param("filePath", filePath, std::string("/catkin_ws/src/trajectory_publisher/odom_points.txt"));

    this->initTrajectory = false;
    preview_dis = PREVIEW_DIS;

    /*参数初始化*/
    startIdx = 0;
    /*订阅话题*/
    carVel = n.subscribe(topicCmdVel.c_str(), 10, &PurePursuitController::velocityCallback, this);
    carPose = n.subscribe(topicCurrentPose.c_str(), 1000, &PurePursuitController::poseCallback, this);

    //创建Publisher，发送经过pure_pursuit计算后的转角及速度
    purepersuit_ = n.advertise<geometry_msgs::Twist>(topicCmdVel.c_str(), 10);
    path_pub_ = n.advertise<nav_msgs::Path>("/purePursuit/rvizpath", 10, true);
    referencePathPub = n.advertise<nav_msgs::Path>("/purePursuit/referencePath", 10, true);
    referPath.header.frame_id = "odom";
    referPath.header.stamp = ros::Time::now();

    path.header.frame_id = "odom";
    path.header.stamp = ros::Time::now();

    getWayPoints(filePath);
    this->initTrajectory = true;
    ROS_INFO("Pure pursuit init successfully!!!");
}

/*加载轨迹文件，获取轨迹列表*/
void PurePursuitController::getWayPoints(std::string fileName)
{
  std::vector<point> waypoint;
  ifstream infile(fileName);
  cout<<"infile"<<endl;
  string line;
  //string temp;
  int i=0;
  vector<double> arr;
  //stringstream temp_line;
  point temp_point;
  double data;
  istringstream iss;
  while(getline(infile, line))
  {
      iss.clear();
      iss.str(line);
      while(iss>>data)
      {
          // cout<<"data = "<<data<<endl;
          arr.push_back(data);
      }
  }

  for(int j=0;j<arr.size();j+=4)
  {
      temp_point.x = arr[j];
      temp_point.y = arr[j+1];
      temp_point.z = arr[j+2];
      temp_point.yaw = arr[j+3];
      waypoint.push_back(temp_point);
  }

  pointNum = waypoint.size();

  geometry_msgs::PoseStamped pose;
  /*轨迹是否取反*/
  if(!isReverse)
  {
    for (int i = 0; i < pointNum; i++) {
      r_x_.push_back(waypoint[i].x);
      r_y_.push_back(waypoint[i].y);
      // ROS_INFO("x: %f, y: %f", waypoint[i].x, waypoint[i].y);
      pose.pose.position.x = waypoint[i].x;
      pose.pose.position.y = waypoint[i].y;
      pose.pose.position.z = 0;

      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 0.0;
      referPath.poses.push_back(pose);
    }
  }
  else
  {
    ROS_INFO("The trajectory has been reversed!!!");
    for (int i = pointNum - 1; i >= 0; i--) {
      r_x_.push_back(waypoint[i].x);
      r_y_.push_back(waypoint[i].y);
      // ROS_INFO("x: %f, y: %f", waypoint[i].x, waypoint[i].y);
      pose.pose.position.x = waypoint[i].x;
      pose.pose.position.y = waypoint[i].y;
      pose.pose.position.z = 0;

      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 0.0;
      referPath.poses.push_back(pose);
    }
  }

  referencePathPub.publish(referPath);

  ROS_INFO("Get waypoints successfully!!! Total points is %d", pointNum);
}


//计算发送给模型车的转角
void PurePursuitController::poseCallback(const nav_msgs::Odometry &currentWaypoint) {
  if (!this->initTrajectory)
  {
    ROS_INFO("initTrajectory !!!");
    return;
  }
  // ROS_INFO("poseCallback");

  auto currentPositionX = currentWaypoint.pose.pose.position.x;
  auto currentPositionY = currentWaypoint.pose.pose.position.y;
  auto currentPositionZ = 0.0;

  auto currentQuaternionX = currentWaypoint.pose.pose.orientation.x;
  auto currentQuaternionY = currentWaypoint.pose.pose.orientation.y;
  auto currentQuaternionZ = currentWaypoint.pose.pose.orientation.z;
  auto currentQuaternionW = currentWaypoint.pose.pose.orientation.w;
  auto currentPositionYaw = tf::getYaw(currentWaypoint.pose.pose.orientation);

  /*判断是否到达终点*/
  float cur2endDistance = sqrt(pow(r_y_[pointNum-1] - currentPositionY, 2) + pow(r_x_[pointNum-1] - currentPositionX, 2));
  // ROS_INFO("****  cur2endDistance %f", cur2endDistance);
  if(cur2endDistance < 2*Ld && startIdx > pointNum / 2)
  {
    ROS_INFO("The car has reached the end!!! The end to current distance is %f. The end point is %f, %f", cur2endDistance, r_x_[pointNum-1], r_y_[pointNum-1]);
    ros::shutdown();
  }

  double delta_l = sqrt(pow(r_y_[startIdx] - currentPositionY, 2) + pow(r_x_[startIdx] - currentPositionX, 2));
  // ROS_INFO("****  delta_l %f,  preview_dis: %f", delta_l, preview_dis);
  while (preview_dis > delta_l && startIdx < pointNum - 1){
    delta_l = sqrt(pow(r_y_[startIdx+1] - currentPositionY, 2) + pow(r_x_[startIdx+1] - currentPositionX, 2));
    startIdx+=1;
  }
  // ROS_INFO("The rate of progress is %d / %d. ", startIdx, pointNum);
  float alpha = atan2(r_y_[startIdx] - currentPositionY, r_x_[startIdx] - currentPositionX) - currentPositionYaw;
  // 当前点和目标点的距离Id
  float dl = sqrt(pow(r_y_[startIdx] - currentPositionY, 2) + pow(r_x_[startIdx] - currentPositionX, 2));
  // 发布小车运动指令及运动轨迹
  // ROS_INFO("****  alpha %f", alpha / M_PI * 180.0);

  /*原地转向，使得车的航向角对齐轨迹方向*/
  float theta = atan(2 * Ld * sin(alpha) / dl);
  if(!directionMatch && abs(alpha) > M_PI / 3.0)
  {
    // if(isReverse)
    // {
    //   theta = -theta;
    // }
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = theta;
    purepersuit_.publish(vel_msg);
    return;
  }
  else
  {
    // ROS_INFO("Direction Matched!!!");
    directionMatch = true;
  }

  if (dl > 2*Ld && directionMatch) 
  {
    // ROS_INFO("The rate of progress is %d / %d. ", startIdx, pointNum);
    // double delta = atan2(2* Ld * sin(alpha), preview_dis);
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = linearVel;
    vel_msg.angular.z = theta;
    purepersuit_.publish(vel_msg);
    // 发布小车运动轨迹
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = currentPositionX;
    this_pose_stamped.pose.position.y = currentPositionY;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(theta);
    this_pose_stamped.pose.orientation.x = currentQuaternionX;
    this_pose_stamped.pose.orientation.y = currentQuaternionY;
    this_pose_stamped.pose.orientation.z = currentQuaternionZ;
    this_pose_stamped.pose.orientation.w = currentQuaternionW;

    this_pose_stamped.header.stamp = ros::Time::now();

    this_pose_stamped.header.frame_id = "odom";
    path.poses.push_back(this_pose_stamped);
  } else {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    purepersuit_.publish(vel_msg);
  }
  path_pub_.publish(path);
}


void PurePursuitController::velocityCallback(const geometry_msgs::Twist &carWaypoint) {
  carVelocity = carWaypoint.linear.x;
  preview_dis = k * carVelocity + PREVIEW_DIS;
  // preview_dis = PREVIEW_DIS;
}


void PurePursuitController::run(){
  ros::Rate rate(10);
  while(!g_request_shutdown){
      ros::spinOnce();
      rate.sleep();
  }
  ros::shutdown();
}

int main(int argc, char **argv) {
  //创建节点
  ros::init(argc, argv, "pure_pursuit", ros::init_options::NoSigintHandler);
  signal(SIGINT, mySignIntHandler);;

  PurePursuitController ppc;  
  ppc.run();
  return 0;
}