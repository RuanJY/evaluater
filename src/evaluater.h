#pragma once
#ifndef IGP_ODOM_H_
#define IGP_ODOM_H_
//std
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <ctime>
#include <chrono>
#include <ctime>

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
//ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <sensor_msgs/FluidPressure.h>

//pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
//ours
#include "utility/common.h"
#include "utility/tool.h"


enum PathType{
    Slam, Odom, Grt
};
class Data_store{//global state
public:
    int mode;
    std::vector<geometry_msgs::PoseStamped> grt_pose_vector;
    std::vector<nav_msgs::Odometry> grt_odom_vector;
    nav_msgs::Odometry slam_odom_buff;
    std::vector<nav_msgs::Odometry> slam_odom_vector;
    int grt_pointer = 0;
    std::queue<sensor_msgs::PointCloud2> registered_pcl_queue;
    std::vector<sensor_msgs::FluidPressure> pressure_vector;
    std::vector<sensor_msgs::Imu> imu_vector;

    Transf T_world_grt0, T_world_grt1, T_lidar0_lidar1, T_grt_lidar, T_lidar0; //1 means final time, 0 means begin time
    nav_msgs::Path path_slam;//xyz and quat, corresponding to path_grt by timestamp
    nav_msgs::Path path_slam_aligned;//aligned with path_grt
    nav_msgs::Path path_slam_full;//all messages
    PointMatrix    position_slam;//only xyz

    nav_msgs::Path path_grt;
    nav_msgs::Path path_grt_aligned;//aligned with path_slam
    PointMatrix    pose_grt;
    ros::Publisher path_pub;


    ///record
    std::ofstream file_loc_report_wrt, file_loc_path_wrt, file_loc_path_cal_wrt, file_loc_path_grt_wrt, file_loc_path_full_wrt;
    std::string  file_loc_rawPcl;
    double trj_length = 0;
    double final_avg_error = 0;
    nav_msgs::Path path_odom;

    pcl::PointCloud<pcl::PointXYZI> pcl_raw_all;
    //for MSE
    std::queue<PointMatrix> ary_last_raw_points;
    PointMatrix last_raw_points;

    ///runtime
    //init
    int step = 0;
    Eigen::Vector3d g;

    Data_store();
    void logInit(const std::string & file_loc_report);
    void imuUpdatePose(Transf & now_slam_trans);
    void pushPath(enum PathType path_type, const Transf & trans){
        //pose
        geometry_msgs::PoseStamped_<std::allocator<void>> path_tmp;
        path_tmp.pose.position.x = trans(0, 3);
        path_tmp.pose.position.y = trans(1, 3);
        path_tmp.pose.position.z = trans(2, 3);

        //orientation
        //trans->tf::matrix->roll yaw pitch->tf::Q, there is no direct way from tf::matrix to tf::Q
        tf::Matrix3x3 tmp_m(trans(0,0), trans(0,1), trans(0,2),
                            trans(1,0), trans(1,1), trans(1,2),
                            trans(2,0), trans(2,1), trans(2,2));
        double roll, yaw, pitch;
        tmp_m.getEulerYPR(yaw, pitch, roll);
        tf::Quaternion tmp_q;
        tmp_q.setRPY(roll, pitch, yaw);
        path_tmp.pose.orientation.x = tmp_q.x();
        path_tmp.pose.orientation.y = tmp_q.y();
        path_tmp.pose.orientation.z = tmp_q.z();
        path_tmp.pose.orientation.w = tmp_q.w();

        //save
        switch (path_type){
            case Slam:{
                path_slam.poses.push_back(path_tmp); break;
            }
            case Odom:{
                path_odom.poses.push_back(path_tmp); break;
            }
            case Grt:{
                path_grt.poses.push_back(path_tmp); break;
            }
            default: ROS_INFO("wrong usage of pushPath");
        }
    }
    void savePath2TxtTum(std::ofstream & file_out, std::vector<geometry_msgs::PoseStamped> & pose_msg_vector){
        //tum format
        //std::ofstream file_out;
        //file_out.open(file_loc, std::ios::out);
        if(file_out){
            for(const auto& one_pose : pose_msg_vector){

                double roll, pitch, yaw;
                tf::Quaternion quat;
                tf::quaternionMsgToTF(one_pose.pose.orientation, quat);
                tf::Matrix3x3 matrix(quat);
                //tmp_M.getRPY(roll, pitch, yaw);
/*                file_out << setprecision(10) << matrix[0][0] << ' ' << matrix[0][1] << ' ' << matrix[0][2] << ' ' << one_pose.one_pose.position.x << ' ' <<
                         matrix[1][0] << ' ' << matrix[1][1] << ' ' << matrix[1][2] << ' ' << one_pose.one_pose.position.y << ' ' <<
                         matrix[2][0] << ' ' << matrix[2][1] << ' ' << matrix[2][2] << ' ' << one_pose.one_pose.position.z << "\n";*/
                file_out << std::setprecision(16) << one_pose.header.stamp.toSec() << " "
                         << one_pose.pose.position.x << " "
                         << one_pose.pose.position.y << " "
                         << one_pose.pose.position.z << " "
                         << one_pose.pose.orientation.w << " "
                         << one_pose.pose.orientation.x << " "
                         << one_pose.pose.orientation.y << " "
                         << one_pose.pose.orientation.z << "\n";
                         //<< roll << "," << pitch << "," << yaw << "\n";
            }
        }
        else{
            std::cout<<"Can not open file: " <<"\n";
        }
        file_out.close();
    }
    void savePressure2Txt(std::ofstream & file_out, std::vector<sensor_msgs::FluidPressure> & pressure_msg_vector){
        if(file_out){
            for(const auto& one_pressure : pressure_msg_vector){
                file_out << std::setprecision(16) << one_pressure.header.stamp.toSec() << " "
                         << one_pressure.fluid_pressure << " "
                         << one_pressure.variance <<
                         "\n";
            }
        }
        else{
            std::cout<<"Can not open file: " <<"\n";
        }
        file_out.close();
    }

    void saveImu2Txt(std::ofstream & file_out, std::vector<sensor_msgs::Imu> & imu_msg_vector){
        if(file_out){
            for(const auto& one_imu : imu_msg_vector){
                file_out << std::setprecision(16) << one_imu.header.stamp.toSec() << " "
                         << one_imu.orientation.w << " "
                         << one_imu.orientation.x << " "
                         << one_imu.orientation.y << " "
                         << one_imu.orientation.z << " "
                         << one_imu.angular_velocity.x << " "
                         << one_imu.angular_velocity.y << " "
                         << one_imu.angular_velocity.z << " "
                         << one_imu.linear_acceleration.x << " "
                         << one_imu.linear_acceleration.y << " "
                         << one_imu.linear_acceleration.z << " "
                         << one_imu.orientation_covariance[0] << " "
                         << one_imu.orientation_covariance[4] << " "
                         << one_imu.orientation_covariance[8] << " "
                         << one_imu.angular_velocity_covariance[0] << " "
                         << one_imu.angular_velocity_covariance[4] << " "
                         << one_imu.angular_velocity_covariance[8] << " "
                         << one_imu.linear_acceleration_covariance[0] << " "
                         << one_imu.linear_acceleration_covariance[4] << " "
                         << one_imu.linear_acceleration_covariance[8] <<
                         "\n";
            }
        }
        else{
            std::cout<<"Can not open file: " <<"\n";
        }
        file_out.close();

    }
    void saveOdom2Txt(std::ofstream & file_out, const std::vector<nav_msgs::Odometry> & odom_vector){
        //output odom msg
        //std::ofstream file_out;
        //file_out.open(file_loc, std::ios::out);
        if(file_out){
            for(const auto& odom : odom_vector){
                file_out << std::setprecision(16) << odom.header.stamp.toSec() << " "
                         << odom.pose.pose.position.x << " "
                         << odom.pose.pose.position.y << " "
                         << odom.pose.pose.position.z << " "
                         << odom.pose.pose.orientation.w << " "
                         << odom.pose.pose.orientation.x << " "
                         << odom.pose.pose.orientation.y << " "
                         << odom.pose.pose.orientation.z << " "
                         << odom.twist.twist.linear.x << " "
                         << odom.twist.twist.linear.y << " "
                         << odom.twist.twist.linear.z << " "
                         << odom.twist.twist.angular.x << " "
                         << odom.twist.twist.angular.y << " "
                         << odom.twist.twist.angular.z << " "
                         << odom.pose.covariance[0] << " "
                         << odom.pose.covariance[7] << " "
                         << odom.pose.covariance[14] << " "
                         << odom.pose.covariance[21] << " "
                         << odom.pose.covariance[28] << " "
                         << odom.pose.covariance[35] << " "
                         << odom.twist.covariance[0] << " "
                         << odom.twist.covariance[7] << " "
                         << odom.twist.covariance[14] << " "
                         << odom.twist.covariance[21] << " "
                         << odom.twist.covariance[28] << " "
                         << odom.twist.covariance[35] <<
                                                         "\n";
                //<< roll << "," << pitch << "," << yaw << "\n";
            }
        }
        else{
            std::cout<<"Can not open file: " <<"\n";
        }
        file_out.close();
    }
    void savePath2Txt(std::ofstream & file_out, nav_msgs::Path & path_msg){
        //kitti format
        //std::ofstream file_out;
        //file_out.open(file_loc, std::ios::out);
        if(file_out){
            for(const auto& path_pose : path_msg.poses){

                double roll, pitch, yaw;
                tf::Quaternion quat;
                tf::quaternionMsgToTF(path_pose.pose.orientation, quat);
                tf::Matrix3x3 matrix(quat);
                //tmp_M.getRPY(roll, pitch, yaw);
                file_out << setprecision(10) << matrix[0][0]<<' '<< matrix[0][1]<<' '<< matrix[0][2]<<' '<< path_pose.pose.position.x<<' '<<
                         matrix[1][0]<<' '<< matrix[1][1]<<' '<< matrix[1][2]<<' '<< path_pose.pose.position.y<<' '<<
                         matrix[2][0]<<' '<< matrix[2][1]<<' '<< matrix[2][2]<<' '<< path_pose.pose.position.z<<"\n";
                /*file_out << path_pose.pose.position.x << ","
                         << path_pose.pose.position.y << ","
                         << path_pose.pose.position.z << ","
                         << path_pose.pose.orientation.w << ","
                         << path_pose.pose.orientation.x << ","
                         << path_pose.pose.orientation.y << ","
                         << path_pose.pose.orientation.z << "\n";
                         //<< roll << "," << pitch << "," << yaw << "\n";*/
            }
        }
        else{
            std::cout<<"Can not open file: " <<"\n";
        }
        file_out.close();
    }
    void savePathEveryStep2Txt(std::ofstream & file_wrt, nav_msgs::Path & path_msg){

        if(file_wrt){

            auto path_pose = path_msg.poses.rbegin();
            double roll, pitch, yaw;
            tf::Quaternion quat;
            tf::quaternionMsgToTF((*path_pose).pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            file_wrt << (*path_pose).pose.position.x << ","
                     << (*path_pose).pose.position.y << ","
                     << (*path_pose).pose.position.z << ","
                     << roll << "," << pitch << "," << yaw << "\n";

        }
    }
    void saveResult();
    void pose_print(ros::Publisher& cloud_pub);

};
class Param{
public:
    int mode;//1 grt 2 mse 3
    std::string file_loc, file_loc_rawpcl, file_loc_grt_path;
    bool read_grt_txt;
    double timestamp_valid_thr = 0.1;
    void init(ros::NodeHandle& nh);
};
//}
#endif