#include "pure_pursuit/purepursuit.h"

using namespace std;
using namespace cpprobotics;

PurePursuitController::PurePursuitController():nPrivate("~")
{ 
    /*params setting*/
    nPrivate.param("Ld", Ld, float(0.3));
    nPrivate.param("PREVIEW_DIS", PREVIEW_DIS, float(2.0));
    nPrivate.param("topicTrajectory", topicTrajectory, std::string("/purepursuit/trajectory"));
    nPrivate.param("topicCmdVel", topicCmdVel, std::string("/diffbot/mobile_base_controller/cmd_vel"));
    nPrivate.param("topicCurrentPose", topicCurrentPose, std::string("/diffbot/mobile_base_controller/odom"));
    nPrivate.param("filePath", filePath, std::string("/home/yp/ros1/catkin_ws/odom_points_mid.txt"));

    this->initTrajectory = false;
    preview_dis = PREVIEW_DIS;
    psi = 0.5; //弧度
    /*订阅话题*/
    // splinePath = n.subscribe(topicTrajectory.c_str(), 1000, &PurePursuitController::pointCallback, this);
    // carVel = n.subscribe(topicCmdVel.c_str(), 10, &PurePursuitController::velocityCallback, this);
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

  int pointNum = waypoint.size();

  for (int i = 0; i < pointNum; i++) {
    r_x_.push_back(waypoint[i].x);
    r_y_.push_back(waypoint[i].y);
    ROS_INFO("index: %d, x: %f, y: %f", i, waypoint[i].x, waypoint[i].y);
  }

  geometry_msgs::PoseStamped pose;
  for(int i=0;i<waypoint.size();i++)
  {
      pose.pose.position.x = waypoint[i].x;
      pose.pose.position.y = waypoint[i].y;
      pose.pose.position.z = 0;

      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 0.0;
      referPath.poses.push_back(pose);
  }

  referencePathPub.publish(referPath);

  ROS_INFO("Get waypoints successfully!!! pointNum: %d", pointNum);
}


//计算发送给模型车的转角
void PurePursuitController::poseCallback(const nav_msgs::Odometry &currentWaypoint) {
  if (!this->initTrajectory)
  {
    ROS_INFO("initTrajectory !!!");
    return;
  }
  ROS_INFO("poseCallback");

  auto currentPositionX = currentWaypoint.pose.pose.position.x;
  auto currentPositionY = currentWaypoint.pose.pose.position.y;
  auto currentPositionZ = 0.0;

  auto currentQuaternionX = currentWaypoint.pose.pose.orientation.x;
  auto currentQuaternionY = currentWaypoint.pose.pose.orientation.y;
  auto currentQuaternionZ = currentWaypoint.pose.pose.orientation.z;
  auto currentQuaternionW = currentWaypoint.pose.pose.orientation.w;
  auto currentPositionYaw = tf::getYaw(currentWaypoint.pose.pose.orientation);

  /*判断是否到达终点*/
  // float cur2endDistance = sqrt(pow(r_y_[pointNum-3] - currentPositionY, 2) + pow(r_x_[pointNum-3] - currentPositionX, 2));
  // ROS_INFO("****  cur2endDistance %f, current %f, %f, target %f, %f", cur2endDistance, currentPositionX, currentPositionY, r_x_[pointNum-1], r_y_[pointNum-1]);
  // if(cur2endDistance < Ld)
  // {
  //   ROS_INFO("The car has arrived target point successfully!!!");
  //   ros::shutdown();
  // }

  // 方案二:通过计算当前坐标和目标轨迹距离，找到一个最小距离的索引号
  int index;
  vector<float> bestPoints_;
  for (int i = 0; i < r_x_.size(); i++) {
    // float lad = 0.0;
    float path_x = r_x_[i];
    float path_y = r_y_[i];
    // 遍历所有路径点和当前位置的距离，保存到数组中
    float lad = sqrt(pow(path_x - currentPositionX, 2) +
                     pow(path_y - currentPositionY, 2));

    bestPoints_.push_back(lad);
  }
   // 找到数组中最小横向距离
  index = min_element(bestPoints_.begin(),bestPoints_.end()) - bestPoints_.begin();
  ROS_INFO("****  index0001 %d", index);

  int min_ind = index;
  double delta_l = sqrt(pow(r_y_[index] - currentPositionY, 2) + pow(r_x_[index] - currentPositionX, 2));
  ROS_INFO("****  delta_l %f,  preview_dis: %f", delta_l, preview_dis);
  while (preview_dis > delta_l && min_ind < r_y_.size()-1){
    delta_l = sqrt(pow(r_y_[min_ind+1] - currentPositionY, 2) + pow(r_x_[min_ind+1] - currentPositionX, 2));
    min_ind+=1;
  }
  index = min_ind;

  /**************************************************************************************************/
  ROS_INFO("****  index %d, min_index %d", index, min_ind);
  ROS_INFO("****  currentPositionX %f, currentPositionY %f", currentPositionX, currentPositionY);

  float alpha = atan2(r_y_[index] - currentPositionY, r_x_[index] - currentPositionX) - currentPositionYaw;
  // 当前点和目标点的距离Id
  float dl = sqrt(pow(r_y_[index] - currentPositionY, 2) + pow(r_x_[index] - currentPositionX, 2));
  // 发布小车运动指令及运动轨迹
  ROS_INFO("****  dl %f", dl);

  // float theta = atan(2 * Ld * sin(alpha) / dl);
  // // double delta = atan2(2* Ld * sin(alpha), preview_dis);
  // geometry_msgs::Twist vel_msg;
  // vel_msg.linear.x = 0.3;
  // vel_msg.angular.z = theta;
  // purepersuit_.publish(vel_msg);
  // ROS_INFO("****  theta %f", theta);

  // // 发布小车运动轨迹
  // geometry_msgs::PoseStamped this_pose_stamped;
  // this_pose_stamped.pose.position.x = currentPositionX;
  // this_pose_stamped.pose.position.y = currentPositionY;

  // geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(theta);
  // this_pose_stamped.pose.orientation.x = currentQuaternionX;
  // this_pose_stamped.pose.orientation.y = currentQuaternionY;
  // this_pose_stamped.pose.orientation.z = currentQuaternionZ;
  // this_pose_stamped.pose.orientation.w = currentQuaternionW;

  // this_pose_stamped.header.stamp = ros::Time::now();

  // this_pose_stamped.header.frame_id = "odom";
  // path.poses.push_back(this_pose_stamped);

  if (dl > 0.2) 
  {
    float theta = atan(2 * Ld * sin(alpha) / dl);
    // double delta = atan2(2* Ld * sin(alpha), preview_dis);
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.3;
    vel_msg.angular.z = theta;
    purepersuit_.publish(vel_msg);
    // 发布小车运动轨迹
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position = currentWaypoint.pose.pose.position;
    this_pose_stamped.pose.orientation = currentWaypoint.pose.pose.orientation;

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

void PurePursuitController::pointCallback(const nav_msgs::Path &msg) {
  // ROS_INFO("*********Receiving trajectory form topics!!!**********");
  if(!initTrajectory)
  {
    pointNum = msg.poses.size();
    ROS_INFO("pointNum is %d", pointNum);

    /*反转*/
    for (int i = pointNum - 1; i >= 0; i--) {
      r_x_.push_back(msg.poses[i].pose.position.x);
      r_y_.push_back(msg.poses[i].pose.position.y);
      // ROS_INFO("x: %f, y: %f", msg.poses[i].pose.position.x, msg.poses[i].pose.position.y);
    }

    // for (int i = 0; i < pointNum; i++) {
    //   r_x_.push_back(msg.poses[i].pose.position.x);
    //   r_y_.push_back(msg.poses[i].pose.position.y);
    // }

    initTrajectory = true;
    ROS_INFO("The end point is x: %f, y: %f", msg.poses[pointNum-1].pose.position.x, msg.poses[pointNum-1].pose.position.y);
  }
  else
  {
    return;
  }
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