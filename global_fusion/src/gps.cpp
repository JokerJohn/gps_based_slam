//#include "utility.h"
#include "gpsTools.h"
#include <thread>
#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <queue>
#include <deque>
#include <mutex>



class GNSSOdom {

 public:
  ros::NodeHandle nh;
  gpsTools gtools;
  double origin_latitude = 0.0, origin_longitude = 0.0, origin_altitude = 0.0;
  Eigen::Vector3d init_pos_;
  double origin_east = 0.0, origin_north = 0.0, origin_height;
  // imu到lidar的转换
  Eigen::Vector3d pos;
  Eigen::Matrix3d rot;
  Eigen::Matrix4d imu2velo;
  ros::Publisher gps_odom_pub_;
  ros::Subscriber imu_sub_, gps_sub_;
  std::string imu_topic, gps_topic, world_frame_id_;
  std::string save_dir_;
  bool use_kitti_ = false;
  // Eigen::Matrix4d T_imu2velo = Eigen::Matrix4d::Identity();
  nav_msgs::Path ros_path_;

  // imu和gps队列
  std::mutex mutex_lock;
  std::deque<sensor_msgs::ImuConstPtr> imuBuf;
  std::deque<sensor_msgs::NavSatFixConstPtr> gpsBuf;

  ros::Publisher fused_path_pub_;

  bool init_xyz = false;
  bool use_localmap = false;
  bool orientation_ready_ = false;
  Eigen::Vector3d prev_pos_;
  double yaw = 0.0;
  geometry_msgs::Quaternion yaw_quat_;

 public:
  GNSSOdom() : nh("~") {
/*    nh_.param<std::string>("imu_topic", imu_topic, "/imu/data");
    nh_.param<std::string>("gps_topic", gps_topic, "/fix");
    nh_.param<std::string>("world_frame_id", world_frame_id_, "map");
    nh_.param("use_localmap", use_localmap, false);  // 使用点云地图原点, 否则使用车辆运动的起点作为地图原点
    nh_.param<std::string>("save_dir_", save_dir_, "/home/xchu/workspace/xchujwu_slam/src/xchu_mapping/pcd/");
    nh_.param<bool>("use_kitti", use_kitti_, false);*/

/*    if (use_kitti_) {
      // kitti的imu到雷达的标定结果
      imu2velo.setIdentity();
      imu2velo << 9.999976e-01, 7.553071e-04, -2.035826e-03, -8.086759e-01,
          -7.854027e-04, 9.998898e-01, -1.482298e-02, 3.195559e-01,
          2.024406e-03, 1.482454e-02, 9.998881e-01, -7.997231e-01,
          0, 0, 0, 1;
      pos = imu2velo.block<3, 1>(0, 3).matrix();
      rot = imu2velo.block<3, 3>(0, 0).matrix();
    }*/

//    ROS_INFO("Init gps node 1");
    imu_sub_ = nh.subscribe("/imu/data", 200, &GNSSOdom::ImuCB, this, ros::TransportHints().tcpNoDelay());
    gps_sub_ = nh.subscribe("/fix", 10, &GNSSOdom::GNSSCB, this, ros::TransportHints().tcpNoDelay());

    gps_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/gps_odom", 4, false);
    fused_path_pub_ = nh.advertise<nav_msgs::Path>("/fused_gps_path", 10);
  }

  void Run() {
    ros::Rate rate(1);
    while (ros::ok()) {
      rate.sleep();
      if (!init_xyz) {
        //ROS_WARN("Waiting to  init lla first!!!");
        continue;
      }

      while (!gpsBuf.empty() /*&& !imuBuf.empty()*/) {
        ros::Time gps_stamp = gpsBuf.front()->header.stamp;
        bool imu_type = false;
        auto imu_iter = imuBuf.begin();
        for (imu_iter; imu_iter != imuBuf.end(); imu_iter++) {
          if (gps_stamp < (*imu_iter)->header.stamp) {
            break;
          }
          //      imu_msg.linear_acceleration = (*imu_iter)->linear_acceleration;
          //      imu_msg.angular_velocity = (*imu_iter)->angular_velocity;
          //      imu_msg.orientation = (*imu_iter)->orientation;
          //      imu_msg.orientation_covariance = (*imu_iter)->orientation_covariance;
          //      imu_msg.linear_acceleration_covariance = (*imu_iter)->linear_acceleration_covariance;
          //      imu_msg.angular_velocity_covariance = (*imu_iter)->angular_velocity_covariance;
          //      imu_msg.header.stamp = (*imu_iter)->header.stamp;
          imu_type = true;
        }
        mutex_lock.lock();
        imuBuf.erase(imuBuf.begin(), imu_iter);
        sensor_msgs::NavSatFixConstPtr gps_msg = gpsBuf.front();
        sensor_msgs::Imu imu_msg = *(imuBuf.front());
        gpsBuf.pop_front();
        mutex_lock.unlock();

        // 检验时间戳是否一致
        double imu_time = imu_msg.header.stamp.toSec();
        double gps_time = gps_msg->header.stamp.toSec();
        double off_time = gps_time - imu_time;
        if (std::abs(off_time) < 0.5) {
          ROS_WARN("off set time: %f ", off_time);
        } else {
          ROS_ERROR("Time aligned failed....");
        }

        // std::cout << "1111111111111111" << std::endl;
        //  convert  LLA to XYZ
        Eigen::Vector3d lla = gtools.GpsMsg2Eigen(*gps_msg);
        Eigen::Vector3d ecef = gtools.LLA2ECEF(lla);
        Eigen::Vector3d enu = gtools.ECEF2ENU(ecef);
        ROS_INFO("GPS ENU XYZ : %f, %f, %f", enu(0), enu(1), enu(2));

        //  std::cout << "222222222222222222" << std::endl;

        Eigen::Vector3d calib_enu;
        if (use_kitti_)
          // gps坐标转换到lidar系下 kitti的gps和imu安装在一起，所以对imu和lidar进行标定即可
          calib_enu = rot * enu + pos;
        else
          calib_enu = enu;
        //      std::cout << "pose bef and aft: " << enu(0) << ", " << enu(1) << ", " << enu(2) << std::endl;
        //      std::cout << "pose bef and aft: " << calib_enu(0) << ", " << calib_enu(1) << ", " << calib_enu(2) << std::endl;

        // 根据运动计算GPS YAW
        double distance = sqrt(pow(enu(1) - prev_pos_(1), 2) +
            pow(enu(0) - prev_pos_(0), 2));
        if (distance > 0.2) {
          yaw = atan2(enu(1) - prev_pos_(1),
                      enu(0) - prev_pos_(0)); // 返回值是此点与远点连线与x轴正方向的夹角
          yaw_quat_ = tf::createQuaternionMsgFromYaw(yaw);
          prev_pos_ = enu;
          orientation_ready_ = true;
          ROS_INFO("YAW CACULATE: %f", yaw);
        } else
          orientation_ready_ = false;

        // pub odom
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = gps_msg->header.stamp;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";

        // ----------------- 1. use utm -----------------------
        //        odom_msg.pose.pose.position.x = utm_x - origin_utm_x;
        //        odom_msg.pose.pose.position.y = utm_y - origin_utm_y;
        //        odom_msg.pose.pose.position.z = msg->altitude - origin_al;

        // ----------------- 2. use enu -----------------------
        odom_msg.pose.pose.position.x = calib_enu(0);
        odom_msg.pose.pose.position.y = calib_enu(1);
        odom_msg.pose.pose.position.z = calib_enu(2);
        odom_msg.pose.covariance[0] = gps_msg->position_covariance[0];
        odom_msg.pose.covariance[7] = gps_msg->position_covariance[4];
        odom_msg.pose.covariance[14] = gps_msg->position_covariance[8];

        if (imu_type) {
          // geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(M_PI / 2);
          // geometry_msgs::Quaternion quaternion = imu_msg.orientation;
          if (orientation_ready_)
            odom_msg.pose.pose.orientation = yaw_quat_;
          else {
            geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(M_PI / 2);
            odom_msg.pose.pose.orientation = quaternion;
          }

          // odom_msg.pose.pose.orientation = imu_msg.orientation;
          odom_msg.pose.covariance[21] = imu_msg.orientation_covariance[0];
          odom_msg.pose.covariance[22] = imu_msg.orientation_covariance[1];
          odom_msg.pose.covariance[23] = imu_msg.orientation_covariance[2];
          odom_msg.pose.covariance[27] = imu_msg.orientation_covariance[3];
          odom_msg.pose.covariance[28] = imu_msg.orientation_covariance[4];
          odom_msg.pose.covariance[29] = imu_msg.orientation_covariance[5];
          odom_msg.pose.covariance[33] = imu_msg.orientation_covariance[6];
          odom_msg.pose.covariance[34] = imu_msg.orientation_covariance[7];
          odom_msg.pose.covariance[35] = imu_msg.orientation_covariance[8];

          odom_msg.twist.twist.linear = imu_msg.linear_acceleration;
          odom_msg.twist.covariance[0] = imu_msg.linear_acceleration_covariance[0];
          odom_msg.twist.covariance[7] = imu_msg.linear_acceleration_covariance[4];
          odom_msg.twist.covariance[14] = imu_msg.linear_acceleration_covariance[8];

          odom_msg.twist.twist.angular = imu_msg.angular_velocity;
          odom_msg.twist.covariance[21] = imu_msg.angular_velocity_covariance[0];
          odom_msg.twist.covariance[28] = imu_msg.angular_velocity_covariance[4];
          odom_msg.twist.covariance[35] = imu_msg.angular_velocity_covariance[8];
          imu_type = false;
        }

        gps_odom_pub_.publish(odom_msg);


        // path
        ros_path_.header.frame_id = "map";
        ros_path_.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped pose;
        pose.header = ros_path_.header;

        pose.pose.position.x = calib_enu(0);
        pose.pose.position.y = calib_enu(1);
        pose.pose.position.z = calib_enu(2);

        pose.pose.orientation.x = yaw_quat_.x;
        pose.pose.orientation.y = yaw_quat_.y;
        pose.pose.orientation.z = yaw_quat_.z;
        pose.pose.orientation.w = yaw_quat_.w;

        ros_path_.poses.push_back(pose);
        fused_path_pub_.publish(ros_path_);
      }
    }
  }

  void ImuCB(const sensor_msgs::ImuConstPtr &msg) {
    mutex_lock.lock();
    imuBuf.push_back(msg);
    mutex_lock.unlock();
  }

  void GNSSCB(const sensor_msgs::NavSatFixConstPtr &msg) {
    if (std::isnan(msg->latitude + msg->longitude + msg->altitude)) {
      ROS_ERROR("POS LLA NAN...");
      return;
    }
    // 设置世界坐标系原点为起始点
    if (!use_localmap && !init_xyz) {
      ROS_INFO("Init Orgin GPS LLA  %f, %f, %f", msg->latitude, msg->longitude, msg->altitude);
      gtools.lla_origin_ << msg->latitude, msg->longitude, msg->altitude;
      init_xyz = true;

      // 保存原点
      //      std::ofstream out;
      //      out.open(save_dir_ + "map_origin.txt", std::ios::out);
      //      out << msg->latitude << " " << msg->longitude << " " << msg->altitude;
      //      out.close();
    }
    mutex_lock.lock();
    gpsBuf.push_back(msg);
    mutex_lock.unlock();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "gps_node");

  GNSSOdom gps;
  std::thread gps_thread(&GNSSOdom::Run, &gps);

  ros::spin();

  gps_thread.join();

  return 0;
}