# 概要

基于odom轨迹点的纯跟踪实现。

# 编译

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://e.coding.net/yunshitu/navigation_and_planning/pure_pursuit_ws.git
git clone https://github.com/ros-mobile-robots/diffbot.git
#### 拷贝pure_pursuit和trajectory_publisher到src中
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```



## 启动纯跟踪

修改参数

```bash
<launch>
    <node pkg="pure_pursuit" type="purepursuit_node" name="purepursuit_node"  output="screen">
        <param name="Ld" value="0.13" /> <!--轴距-->
        <param name="PREVIEW_DIS" value="0.5" /> <!--前瞻距离-->
        <param name="linearVel" value="1.0" /> <!--底盘线速度-->
        <param name="isReverse" value="true" /> <!--轨迹是否反转-->
        <param name="topicTrajectory" value="/splinepoints" />  
        <param name="topicCmdVel" value="/diffbot/mobile_base_controller/cmd_vel" />  <!--底盘速度话题-->
        <param name="topicCurrentPose" value="/diffbot/mobile_base_controller/odom" /> <!--实时定位话题-->
        <param name="filePath" value="/home/yp/ros1/catkin_ws/odom_points.txt" /> <!--轨迹文件-->
    </node> 
</launch>
```

PS: topicTrajectory已舍弃

新开终端

```
roslaunch pure_pursuit pure_pursuit.launch 
```



