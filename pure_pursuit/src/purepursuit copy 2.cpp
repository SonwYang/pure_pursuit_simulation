#include "pure_pursuit/purepursuit.h"

using namespace std;
using namespace cpprobotics;

/*按大小排序，并返回索引列表*/
 template <typename T>
vector<size_t> sort_indexes_e(vector<T> &v)
{
    vector<size_t> idx(v.size());
    iota(idx.begin(), idx.end(), 0);
    sort(idx.begin(), idx.end(),
        [&v](size_t i1, size_t i2) {return v[i1] < v[i2]; });
    return idx;
}

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
    /*参数初始化*/
    midDistance = 1.0;
    pointNum = 0;
    startIndex = 0;
    middleIdx = 1;
    maxIdx = 1;

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

/*判断目标轨迹点是否超出前瞻距离*/
bool PurePursuitController::isWayPtAwayFromDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < PREVIEW_DIS)
        return false;
    else if(dist >= PREVIEW_DIS)
        return true;
}


/*获取给定文件中的轨迹点*/
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

  for (int i = 0; i < pointNum; i++) {
    r_x_.push_back(waypoint[i].x);
    r_y_.push_back(waypoint[i].y);

    pose.pose.position.x = waypoint[i].x;
    pose.pose.position.y = waypoint[i].y;
    pose.pose.position.z = 0;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.0;
    referPath.poses.push_back(pose);
    // ROS_INFO("index: %d, x: %f, y: %f", i, waypoint[i].x, waypoint[i].y);
  }
  /*发布参考轨迹*/
  referencePathPub.publish(referPath);

  /*找到距离中值的下标*/
  vector<float> distanceList;
  for (int i = 0; i < r_x_.size(); i++) {
    // 遍历所有路径点和当前位置的距离，保存到数组中
    float lad = sqrt(pow( r_x_[i] - r_x_[0], 2) +
                     pow(r_y_[i] - r_x_[0], 2));

    distanceList.push_back(lad);
  }

   /*按大小排序，返回索引列表，取中位数作为理想值*/
  std::vector<size_t> idx;
  idx = sort_indexes_e(distanceList);
  middleIdx = idx[int(distanceList.size() / 2)];
  midDistance = distanceList[middleIdx];
  ROS_INFO("Find the middle distance!!! Middle index %d / %d, val : %f ", middleIdx, pointNum, midDistance);

  /*找到最大值下标*/
  maxIdx = max_element(distanceList.begin(),distanceList.end()) - distanceList.begin();
  maxDis = distanceList[maxIdx];
  ROS_INFO("Find the furthest distance!!! Max index %d / %d, val : %f ", maxIdx, pointNum, maxDis);
}


//计算发送给模型车的线速度和角速度
void PurePursuitController::poseCallback(const nav_msgs::Odometry &currentWaypoint) {
  if (!this->initTrajectory)
  {
    ROS_INFO("initTrajectory !!!");
    return;
  }
  // ROS_INFO("poseCallback");

  geometry_msgs::Point carPos = currentWaypoint.pose.pose.position;
  double carYaw = tf::getYaw(currentWaypoint.pose.pose.orientation);
  bool foundForwardPt = false;
  geometry_msgs::Point forwardPt;

   /*判断是否到达终点*/
  float cur2endDistance = sqrt(pow(r_y_[pointNum-1] - carPos.y, 2) + pow(r_x_[pointNum-1] - carPos.x, 2));
  if(cur2endDistance < Ld && startIndex >= maxIdx / 2)
  {
    ROS_INFO("The car has arrived target point successfully!!!");
    ros::shutdown();
  }

  //通过计算当前坐标和目标轨迹距离，找到一个最小距离的索引号
  int minIdx;
  int endIdx;
  vector<float> bestPoints_;
  /*当小车行驶一段距离后再进行全局判断*/
  if(startIndex > maxIdx / 2)
  {
    endIdx = pointNum;
  }
  else{
    endIdx = maxIdx;
  }

  for (int i = 0; i <endIdx; i++) {
    // float lad = 0.0;
    float path_x = r_x_[i];
    float path_y = r_y_[i];
    // 遍历所有路径点和当前位置的距离，保存到数组中
    float lad = sqrt(pow(path_x - carPos.x, 2) +
                     pow(path_y - carPos.y, 2));

    bestPoints_.push_back(lad);
  }

   // 找到数组中最小横向距离
  minIdx = min_element(bestPoints_.begin(),bestPoints_.end()) - bestPoints_.begin();
  // ROS_INFO("****  index0001 %d", minIdx);

  double delta_l = sqrt(pow(r_y_[minIdx] - carPos.y, 2) + pow(r_x_[minIdx] - carPos.x, 2));
  // ROS_INFO("****  delta_l %f,  preview_dis: %f", delta_l, preview_dis);
  while (preview_dis > delta_l && minIdx < r_y_.size()-1){
    delta_l = sqrt(pow(r_y_[minIdx+1] - carPos.y, 2) + pow(r_x_[minIdx+1] - carPos.x, 2));
    minIdx+=1;
  }

  startIndex = minIdx;
  // 当前点和目标点的距离Id
  float dl = sqrt(pow(r_y_[minIdx] - carPos.y, 2) + pow(r_x_[minIdx] - carPos.x, 2));

  
  if (dl > 0.2) 
  {
    ROS_INFO("The rate of progress is %d / %d", startIndex, pointNum);
    float alpha = atan2(r_y_[minIdx] - carPos.y, r_x_[minIdx] - carPos.x) - carYaw;
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