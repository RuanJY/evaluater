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
//ceres
#include<ceres/ceres.h>
//ours
#include "utility/common.h"
#include "utility/tool.h"

enum Direction{
    X=0, Y, Z, Unknown
};
enum PathType{
    Slam, Odom, Grt
};
class Region{
public:
    double x_min{0}, y_min{0}, z_min{0}, x_max{0}, y_max{0}, z_max{0};
    Region(){
        x_min=0, y_min=0, x_max=0, y_max=0, z_max=0, z_min=0;
    }
    Region(double _x_min, double _y_min, double _z_min, double _x_max, double _y_max, double _z_max){
        x_min=_x_min, y_min=_y_min, x_max=_x_max, y_max=_y_max, z_max=_z_max, z_min=_z_min;
    }
};
class Data_store{//global state
public:
    //new
    int mode;
    //std::vector<nav_msgs::Odometry> grt_msg_buff;
    std::vector<geometry_msgs::PoseStamped> grt_msg_buff;
    nav_msgs::Odometry tf_slam_buff;
    int grt_pointer = 0;
    std::queue<sensor_msgs::PointCloud2> registed_pcl_buff;
    bool tf_slam_new = false;
    Transf T_grt_body0, T_grt_body1, T_lidar0_lidar1, T_body_lidar;;
    nav_msgs::Path path;
    nav_msgs::Path path_cal;//calibrated
    nav_msgs::Path path_full;
    PointMatrix    pose;
    nav_msgs::Path path_grt;
    PointMatrix    pose_grt;
    Transf T_grt_lidar;
    //Eigen::Matrix<double, 3, Eigen::Dynamic> pose;

    //old
    ///record
    std::ofstream file_loc_report_wrt, file_loc_path_wrt, file_loc_path_cal_wrt, file_loc_path_grt_wrt, file_loc_path_full_wrt;
    std::string file_loc_GPmap, file_loc_rawPcl;
    Eigen::Matrix<double, 1, Eigen::Dynamic> num_mc_now, num_mc_glb, num_mc_new ,
                                             time_cost, time_cost_draw_map, time_find_overlap,
                                             isline_num, overlap_point_num, gp_times_num, rg_times;
    double trj_length = 0;

    size_t gp_times_sum = 0;
    //size_t un_bounded_count = 0; //check if gp point is unbounded
    //size_t isline_count = 0;


    nav_msgs::Path path_odom;


    pcl::PointCloud<pcl::PointXYZ> pcl_raw_all;
    //for MSE
    std::queue<PointMatrix> ary_last_raw_points;
    PointMatrix last_raw_points;

    ///runtime
    //init
    Transf first_trans, lidar_trans, grt_first_trans;
    bool if_first_imu = false, if_first_alt = false, imu_init = false, if_first_trans_init = false;
    int step = 0;
    //trans
    std::vector<Transf> Tseq;

    Transf trf_odom_last = Eigen::MatrixXd::Identity(4, 4),
          trf_odom_now   = Eigen::MatrixXd::Identity(4, 4),
          trf_slam       = Eigen::MatrixXd::Identity(4, 4);
    std::vector<State> odom_offline;//not used
    //imu
    std::queue<sensor_msgs::ImuConstPtr> imu_buf;
    Eigen::Vector3d imu_pos, imu_vel, imu_Ba, imu_Bg, acc_0, gyr_0;
    Eigen::Matrix3d imu_rot;
    Eigen::Vector3d g;
    //lidar
    std::queue<sensor_msgs::PointCloud2> lidar_buf;
    sensor_msgs::PointCloud2 pcl_msg_buff;
    double laser_seq_new = -1;
    double laser_seq_old = -1;
    bool laser_new = false;
    Eigen::Matrix<double, 1, Eigen::Dynamic> range_buff;

    Data_store();
    void logInit(const std::string & file_loc_report);
    void imuUpdatePose(Transf & now_slam_trans);
    inline void savePath(enum PathType path_type, const Transf & trans){
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
                path.poses.push_back(path_tmp); break;
            }
            case Odom:{
                path_odom.poses.push_back(path_tmp); break;
            }
            case Grt:{
                path_grt.poses.push_back(path_tmp); break;
            }
            default: ROS_INFO("wrong usage of savePath");
        }
    }
    static void savePath2Txt(std::ofstream & file_out, nav_msgs::Path & path_msg){

        //std::ofstream file_out;
        //file_out.open(file_loc, std::ios::out);
        if(file_out){
            for(const auto& path_pose : path_msg.poses){

                double roll, pitch, yaw;
                tf::Quaternion quat;
                tf::quaternionMsgToTF(path_pose.pose.orientation, quat);
                tf::Matrix3x3 matrix(quat);
                //tmp_M.getRPY(roll, pitch, yaw);
                file_out << matrix[0][0]<<' '<< matrix[0][1]<<' '<< matrix[0][2]<<' '<< path_pose.pose.position.x<<' '<<
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
    static void savePathEveryStep2Txt(std::ofstream & file_wrt, nav_msgs::Path & path_msg){

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
    void saveResult(double final_avg_error);
    void pose_print(ros::Publisher& cloud_pub);
    void savePcl(pcl::PointCloud<pcl::PointXYZ> pcl_used, Transf& trans_result){

        //std::cout<<"num_point_all_raw_point last: \n"<<pcl_raw_all.width * pcl_raw_all.height<<"\n";//std::endl;
        static int skip_i = 0;
        int skip_step = 1;
        skip_i ++;
        if(skip_i % skip_step == 0){
            pcl::transformPointCloud(pcl_used, pcl_used, trans_result.cast<float>());
            pcl_raw_all = pcl_raw_all + pcl_used;
            std::cout<<"num_point_all_raw_point: "<<pcl_raw_all.width * pcl_raw_all.height<<"\n";//std::endl;
        }
    }
};
class Param{
public:
    int mode;//1 grt 2 mse 3
    int    max_steps, rg_times, num_test, min_num_to_gp;
    double range_max, range_min, range_unit;
    double delta_regist, delta_map_update, delta_map_show;
    double grid, voxel_size, residual;

    double correction_x, correction_y, correction_z, correction_roll_degree, correction_pitch_degree, correction_yaw_degree;
    double lidar_install_x, lidar_install_y, lidar_install_z, lidar_install_roll_degree, lidar_install_pitch_degree, lidar_install_yaw_degree;

    double keyframe_delta_t, keyframe_delta_r;
    int look_back, window_size;

    std::string file_loc, file_loc_report, file_loc_final_map, file_loc_rawpcl;

    bool odom_available, graph_optimize, isline_mode, offline, kdtree_sample, grt_available, imu_feedback, imu_optimize;

    double bias_acc_x, bias_acc_y;

    Param(){
        max_steps = 10000;
    };
    void init(ros::NodeHandle& nh);
};
//}
#endif