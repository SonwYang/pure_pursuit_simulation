#include "pure_pursuit/online_trajectory_publish.h"


class OnlineTrajectoryPub{
    public:
        ros::NodeHandle nh;
        ros::Publisher state_pub_;
        nav_msgs::Path ros_path_;
        ros::Subscriber pose_sub;
        bool init;
        double EARTH_RADIUS = 6378.137;//地球半径
        my_pose init_pose;

        OnlineTrajectoryPub(){
            pose_sub = nh.subscribe("/gps/main",10, &OnlineTrajectoryPub::gpsCallback, this);        
            state_pub_ = nh.advertise<nav_msgs::Path>("/trajectory/gps_main", 10);
            init = false;
        }

        void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr)
        {
            //初始化
            if(!init)
            {
                init_pose.latitude = gps_msg_ptr->latitude;
                init_pose.longitude = gps_msg_ptr->longitude;
                init_pose.altitude = gps_msg_ptr->altitude;
                init = true;
                ROS_INFO("( latitude:%0.15f ,longitude:%0.15f ,altitude:%0.15f)",init_pose.latitude ,init_pose.longitude ,init_pose.altitude);
            }
            else
            {
            //计算相对位置
            double radLat1 ,radLat2, radLong1,radLong2,delta_lat,delta_long,x,y;
            radLat1 = rad(init_pose.latitude);
            radLong1 = rad(init_pose.longitude);
            radLat2 = rad(gps_msg_ptr->latitude);
            radLong2 = rad(gps_msg_ptr->longitude);
        
            //计算x
            delta_long = 0;
            delta_lat = radLat2 - radLat1;  //(radLat1,radLong1)-(radLat2,radLong1)
            if(delta_lat>0)
                x = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat1 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ));
                else
            x=-2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat1 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ));

                x = x*EARTH_RADIUS*1000;

                //计算y
            delta_lat = 0;
                delta_long = radLong2  - radLong1;   //(radLat1,radLong1)-(radLat1,radLong2)
            if(delta_long>0)
            y = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat2 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ) );
            else
            y=-2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat2 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ) );
                //double y = 2*asin( sin( delta_lat/2 ) + cos( radLat2 )*cos( radLat2)* sin( delta_long/2 )   );
                y = y*EARTH_RADIUS*1000;

                //计算z
                double z = gps_msg_ptr->altitude - init_pose.altitude;

                //发布轨迹
                ros_path_.header.frame_id = "odom";
                ros_path_.header.stamp = ros::Time::now();  

                geometry_msgs::PoseStamped pose;
                pose.header = ros_path_.header;

                pose.pose.position.x = x;
                pose.pose.position.y = -y;
                pose.pose.position.z = 0;

                ros_path_.poses.push_back(pose);

                ROS_INFO("( x:%0.15f ,y:%0.15f ,z:%0.15f)",x ,y ,z);
            }
        }

        /*When the node is killed, publishing topic*/
        ~OnlineTrajectoryPub(){
            state_pub_.publish(ros_path_);
            ROS_INFO("Publishing topic successfully!!!");
            ros::spinOnce();
        }

        void run(){
            ros::Rate rate(10);
            while(!g_request_shutdown){
                ros::spinOnce();
                rate.sleep();
            }
            state_pub_.publish(ros_path_);
            ROS_INFO("Publishing topic successfully!!!");
            ros::shutdown();
        }
};

int main(int argc,char **argv)
{
    ros::init(argc,argv,"gps_main", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySignIntHandler);
    OnlineTrajectoryPub otp;
    otp.run();
    return 0;
}
