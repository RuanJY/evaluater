#pragma once
#ifndef TOOL_H_
#define TOOL_H_
#include "common.h"
#include <iostream>
#include <chrono>
//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility>
//ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
//pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <nav_msgs/Odometry.h>

//format transform
Transf state2trans3(State state);
Transf state2quat2trans3(State state);
Transf state2quat2trans3_down(State state);
State trans32state(Transf & _trans);
sensor_msgs::PointCloud matrix2D2pclMsg(Eigen::MatrixXd _mx, int _npoint);
sensor_msgs::PointCloud matrix3D2pclMsg(Eigen::MatrixXd _mx, int _npoint);
Eigen::Matrix<double, 1, 6> txt2matrixSeattleOdom(int _line, std::ifstream &file_odom);
bool txt2PoseMsg(std::ifstream &file_odom, geometry_msgs::PoseStamped & pose_msg);
pcl::PointCloud<pcl::PointXYZ> matrix3D2pcl(const PointMatrix & _pmatrix);
void eigen2pcl();
geometry_msgs::PoseWithCovariance transf2PoseWithCovariance(Transf trans);
Transf PoseWithCovariance2transf(geometry_msgs::PoseWithCovariance _pose);
Transf PoseStamp2transf(geometry_msgs::PoseStamped _pose);
Transf Odometry2transf(const nav_msgs::Odometry& _odom);
pcl::PointCloud<pcl::PointXYZ> readKitti(int line_num);
State trans32quat2state(Transf & _trans);
//transformation
Point trans3D(int _x, int _y, int _z, const Transf& _trans);
Eigen::MatrixXd trans3D(const Eigen::MatrixXd &bef_rotate, Eigen::Isometry3d &T);
Transf createTrans(double x, double y, double z, double roll, double pitch, double yaw);

//filter
sensor_msgs::PointCloud2 pclMsg2VotexFilter(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg, double voxel_size) ;
pcl::PointCloud<pcl::PointXYZ> pclVotexFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcl, double voxel_size) ;

//visualize
void scan_print(ros::Publisher& cloud_pub, PointMatrix& scan);
void scan_print3D(ros::Publisher& cloud_pub, PointMatrix& scan);
void pose_print(ros::Publisher& cloud_pub, Eigen::Matrix<double, 3, Eigen::Dynamic> & pose, int pose_num);

//others
inline double exp_quick(double x){
    //1
    x = 1.0 + x/64;
    x *= x;       x *= x;       x *= x;       x *= x;
    x *= x;       x *= x;
    return x;
    //2
/*    double result;
    *((int *)(&result) + 0) = 0;
    *((int *)(&result) + 1) = 1512775 * x + 1072632447;
    return result;*/
    //3
/*    return x < 1.5 ? (1 + x/1.5) : 0;*/
}
Point tran_error(Transf& t_last, Transf& t_new);
double getPosi(double x_min, double y_min, double z_min, double grid);
class TicToc
{
public:
    TicToc()
    {
        tic();
    }
    explicit TicToc(std::string  _name):name(std::move(_name))
    {
        tic();
    }
    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }
    void toc_print_ms(){
        std::cout<<name<<":"<< toc() << "ms"<< std::endl;
    }
    void toc_print_us(){
        std::cout<<name<<":"<< toc()*1000 << "us"<< std::endl;
    }

private:
    std::string name{"not named timer"};
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
float computeMSE(const PointMatrix & pM_source, const PointMatrix & pM_target);
inline void evenSetLinSpaced(Eigen::Matrix<double, 1, Eigen::Dynamic> & test, int num_test, double min, double max){

    test = Eigen::MatrixXd::Zero(1, num_test);
    double interval = (max - min) / (num_test*1.0);
    test(0,0) = min + 0.5 * interval;
    for(int i=1; i< num_test-1; i++){
        test(0,i) = min + (i+0.5)*interval;
    }
    test(0,num_test-1) = max - 0.5 * interval;
    //std::cout<<"evenSetLinSpaced:" << test<<std::endl;
}
#endif

