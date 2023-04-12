/**
 * FileName: evaluater.cpp
 * Author: Jianyuan Ruan ruanjy@zju.edu.cn
 * Date: 2023_02-10
 * Situation:
 * Description: online evaluation of slam"
 * History:
      <author>  <time>     <desc>
      ruanjy   20/02/10    create
      ruanjy   23/02/10    TO DO: save path mode, publish registered grt path, enable map evaluation,
 */
#include "evaluater.h"
Param param;//initialize ros parameters
Data_store g_data;//initialize global variables
Transf computeExtrinsic(PointMatrix &_scan_glb, PointMatrix &_scan_now){

    Transf R = Eigen::MatrixXd::Identity(4, 4);
    if((_scan_glb.num_point == 0)||(_scan_now.num_point == 0)){
        ROS_ERROR("0 point in computeRT");
        return R;
    }
    if((_scan_glb.num_point != _scan_now.num_point)){
        ROS_ERROR("Different num of points in computeRT");
        std::cout<<"num1 "<<_scan_glb.num_point<<" num2 "<<_scan_now.num_point<<std::endl;
        return R;
    }
    _scan_glb.mse_eig();
    _scan_now.mse_eig();

    Eigen::MatrixXd M = ((_scan_now.point.leftCols(_scan_now.num_point).colwise()-_scan_now.gravity)*
                         ((_scan_glb.point.leftCols(_scan_glb.num_point).colwise()-_scan_glb.gravity).transpose()))/ _scan_now.num_point;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV );
    Eigen::MatrixXd V = svd.matrixV(), U = svd.matrixU();
    double detVU =  (V*(U.adjoint())).determinant();
    //std::cout<<detVU<<std::endl;
    Eigen::Matrix<double, 3, 3> I;
    I << 1,0,0, 0,1,0, 0,0,detVU;//?
    Eigen::MatrixXd rotate = V*I *(U.adjoint());
    Eigen::Matrix<double, 3, 1 > transf = _scan_glb.gravity - (rotate * _scan_now.gravity);
    R << rotate, transf, 0,0,0,1;
    //std::cout << "Rnew:" << std::endl << R << std::endl;
    return R;
}
double rotationError(){
    double trans_error = 0, rotation_error = 0, ori_trans_error = 0;
    Transf trans_bef_cal, trans_aft_cal; int i;
    for(i=0; i<g_data.path.poses.size(); i++){
        geometry_msgs::PoseStamped path_pose = g_data.path.poses[i];
        geometry_msgs::PoseStamped pose_grt  = g_data.path_grt.poses[i];

        //pose->Transf
        trans_bef_cal = PoseStamp2transf(path_pose);
        //std::cout<<"trans_bef_cal\n"<<trans_bef_cal<<std::endl;
        //std::cout<<"g_data.T_body_lidar\n"<<g_data.T_body_lidar<<std::endl;
        //calibrated
        //trans_aft_cal = g_data.T_body_lidar * trans_bef_cal;
        trans_aft_cal = trans_bef_cal * g_data.T_body_lidar;

        //std::cout<<"trans_aft_cal\n"<<trans_aft_cal<<std::endl;
        //eror
        trans_error += sqrt(pow(trans_aft_cal(0, 3) - g_data.pose_grt.point(0, i), 2) +
                            pow(trans_aft_cal(1, 3) - g_data.pose_grt.point(1, i), 2) +
                            pow(trans_aft_cal(2, 3) - g_data.pose_grt.point(2, i), 2));
/*        ori_trans_error += sqrt(pow(g_data.pose.point(0, i) - g_data.pose_grt.point(0, i), 2) +
                                pow(g_data.pose.point(1, i) - g_data.pose_grt.point(1, i), 2) +
                                pow(g_data.pose.point(2, i) - g_data.pose_grt.point(2, i), 2));*/
/*        ori_trans_error += sqrt(pow(path_pose.pose.position.x - pose_grt.pose.position.x, 2) +
                                pow(path_pose.pose.position.y - pose_grt.pose.position.y, 2) +
                                pow(path_pose.pose.position.z - pose_grt.pose.position.z, 2));*/
        ori_trans_error += sqrt(pow(trans_bef_cal(0, 3) - g_data.pose_grt.point(0, i), 2) +
                                pow(trans_bef_cal(1, 3) - g_data.pose_grt.point(1, i), 2) +
                                pow(trans_bef_cal(2, 3) - g_data.pose_grt.point(2, i), 2));
        tf::Matrix3x3 tmp_m(trans_aft_cal(0,0), trans_aft_cal(0,1), trans_aft_cal(0,2),
                            trans_aft_cal(1,0), trans_aft_cal(1,1), trans_aft_cal(1,2),
                            trans_aft_cal(2,0), trans_aft_cal(2,1), trans_aft_cal(2,2));
        double roll, yaw, pitch;
        //tmp_m.getEulerYPR(yaw, pitch, roll);
        tmp_m.getRPY(roll, pitch, yaw);
        std::cout<<"path rpy"<<roll<<" "<< pitch<<" "<<yaw<<std::endl;
        double roll2, pitch2, yaw2;
        tf::Quaternion quat2;
        tf::quaternionMsgToTF(pose_grt.pose.orientation, quat2);
        tf::Matrix3x3 grt_M (quat2);
        grt_M.getRPY(roll2, pitch2, yaw2);
        std::cout<<"grt rpy"<<roll2<<" "<< pitch2<<" "<<yaw2<<std::endl;
        //grt_M.
        rotation_error += std::abs(roll-roll2) + std::abs(pitch-pitch2) + std::abs(yaw-yaw2);
    }
    trans_error /= double(i);
    rotation_error /= double(i);
    ori_trans_error /= double(i);
    std::cout<<"ori trans error "<<ori_trans_error<<std::endl;
    std::cout<<"new trans error "<<trans_error<<std::endl;
    std::cout<<"rotation_error "<<rotation_error<<std::endl;
}
double computeError(PointMatrix & _scan_glb, PointMatrix & _scan_now, double & _past_50error){

    double error = 0; int i;
    for(i=0; i<_scan_glb.num_point; i++){
        error += sqrt(pow(_scan_glb.point(0, i) - _scan_now.point(0, i), 2) +
                      pow(_scan_glb.point(1, i) - _scan_now.point(1, i), 2) +
                      pow(_scan_glb.point(2, i) - _scan_now.point(2, i), 2));
    }
    error /= double(i);
    int j=0, start = _scan_glb.num_point - 50; _past_50error = 0;
    if(start < 0) start = 0;
    for(j=start; j<_scan_glb.num_point; j++){
        _past_50error += sqrt(pow(_scan_glb.point(0, j) - _scan_now.point(0, j), 2) +
                              pow(_scan_glb.point(1, j) - _scan_now.point(1, j), 2) +
                              pow(_scan_glb.point(2, j) - _scan_now.point(2, j), 2));
    }
    _past_50error /= double(j-start);
    return error;
}

Data_store::Data_store(){
    //pose       = Eigen::MatrixXd::Zero(3, max_size);
    //first_trans = lidar_trans = grt_first_trans = trf_odom_now = trf_odom_last = Eigen::MatrixXd::Identity(4,4);
    path.header.frame_id      = "velodyne";
    path_odom.header.frame_id = "velodyne";
    path_grt.header.frame_id  = "velodyne";
    path_full.header.frame_id = "velodyne";
    pcl_raw_all.height = 1;
    pcl_raw_all.width = 0;
}
void Data_store::logInit(const std::string & file_loc){
    //path
    //size_t length = file_loc.size() - 4;
    //std::string tmp_file_loc = file_loc.substr(0, length);
    std::cout<<"tend to save report in: "<<file_loc<<std::endl;
    std::string file_loc_report    = file_loc + "_report.txt";
    std::string file_loc_path      = file_loc + "_path.txt";
    std::string file_loc_path_cal  = file_loc + "_path_cal.txt";
    std::string file_loc_path_full = file_loc + "_path_full.txt";
    std::string file_loc_path_grt  = file_loc + "_path_grt.txt";
    file_loc_rawPcl = file_loc + "_raw.pcd";
    //open
    file_loc_report_wrt.open (file_loc_report, std::ios::out);
    if(!file_loc_report_wrt){
        ROS_WARN("Can not open Report file");
    }
    file_loc_path_wrt.open(file_loc_path, std::ios::out);
    if(!file_loc_path_wrt){
        ROS_WARN("Can not open Path file");
    }
    file_loc_path_cal_wrt.open(file_loc_path_cal, std::ios::out);
    if(!file_loc_path_cal_wrt){
        ROS_WARN("Can not open Path file");
    }
    file_loc_path_full_wrt.open(file_loc_path_full, std::ios::out);
    if(!file_loc_path_full_wrt){
        ROS_WARN("Can not open Path Odom file");
    }
    file_loc_path_grt_wrt.open(file_loc_path_grt, std::ios::out);
    if(!file_loc_path_grt_wrt){
        ROS_WARN("Can not open Path Odom file");
    }
}
void   Data_store::saveResult()
{
    std::cout<<std::setprecision(10)<<setiosflags(std::ios::fixed);
    Transf trans_error = (T_grt_body0 * T_body_lidar * T_lidar0_lidar1).inverse() * T_grt_body1 *  T_body_lidar;
    double translation_error = std::abs(trans_error(0,3)) + std::abs(trans_error(1,3)) + std::abs(trans_error(2,3));
    State state_error = trans32quat2state(trans_error);
    double rotation_error = std::abs(state_error(3,0)) + std::abs(state_error(4,0)) + std::abs(state_error(5,0));
    //print
    /*std::cout << "trans_error: \n" << trans_error << std::endl
              << "tra_error: " << translation_error << std::endl
              << "rel_tra_error: " << translation_error/trj_length << std::endl
              << "rotation_error: " << rotation_error << std::endl
              << "TRJ LENGTH: " << trj_length << std::endl
              << "avg error: " << trj_length << std::endl;*/
    //std::cout<<"avg_error "<<final_avg_error<<std::endl;

    //std::cout << "\nERROR : \n" <<sqrt(pow(pose(0,step)-254.513,2) + pow(pose(1,step)-658.088,2) + pow(pose(2,step)-68.2958,2))<< std::endl;
    //save report
    if(file_loc_report_wrt){
        file_loc_report_wrt << "step: " << step << "\n"
                  //<<"avg_error "<<final_avg_error<<std::endl
                  << "trans_error: \n" << trans_error << std::endl
                  << "tra_error: " << translation_error << std::endl
                  << "rel_tra_error: " << translation_error/trj_length << std::endl
                  << "rotation_error: " << rotation_error << std::endl
                  << "TRJ LENGTH: " << trj_length << std::endl
                  << "T_body_lidar: \n"    << T_body_lidar << std::endl
                  << "T_grt_body0: \n"     << T_grt_body0 << std::endl
                  << "T_lidar0_lidar1: \n" << T_lidar0_lidar1 << std::endl
                  << "T_grt_body1: \n"     << T_grt_body1 << std::endl;
        std::cout << "Result saved in: " << param.file_loc_report << std::endl;
        file_loc_report_wrt.close();
    }
    else{
        std::cout << param.file_loc_report <<" Result not saved" << std::endl;
    }
    //save path
    savePath2Txt(file_loc_path_wrt,     path);
    savePath2Txt(file_loc_path_cal_wrt, path_cal);
    savePath2Txt(file_loc_path_full_wrt,path_full);
    savePath2Txt(file_loc_path_grt_wrt, path_grt);
    //savePath2TxtTum(file_loc_path_full_wrt, slam_tf_msg_vector);

    if(mode == 2){
        pcl::io::savePCDFileASCII(file_loc_rawPcl, pcl_raw_all);
    }
}
void   Data_store::imuUpdatePose(Transf & now_slam_trans){
    ROS_DEBUG("imuUpdatePose");
    Point _pose;
    _pose = trans3D(0, 0, 0, now_slam_trans);
    //pose.addPoint(_pose);
    if(step>0) trj_length += sqrt(pow(_pose(0,0) - pose.point(0,step-1), 2) +
                                  pow(_pose(1,0) - pose.point(1,step-1), 2) +
                                  pow(_pose(2,0) - pose.point(2,step-1), 2));

    pushPath(Slam, now_slam_trans);
    //save path and grt_path to txt
    //savePathEveryStep2Txt(file_loc_path_gdt_wrt, path_grt);
    //savePathEveryStep2Txt(file_loc_path_wrt, path);
}
void Param::init(ros::NodeHandle& nh){
    nh.param("evaluater/file_loc", file_loc, std::string("~"));
    nh.param("evaluater/file_loc_grt_path", file_loc_grt_path, std::string("~"));
    nh.param("evaluater/read_grt_online", read_grt_online, false);
}
bool saveOnePoseAndPath(geometry_msgs::PoseStamped & pose_msg, PointMatrix & pose, nav_msgs::Path & path){
    Point tmp_point;
    tmp_point << pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z;
    pose.addPoint(tmp_point);
    path.poses.push_back(pose_msg);
}
bool computeOnlineError(){
    double time_tf_slam, time_grt, final_avg_error;
    if(g_data.grt_msg_vector.size() < 2){
        return false;
    }
    time_tf_slam = g_data.tf_slam_buff.header.stamp.toSec();
    time_grt = g_data.grt_msg_vector[g_data.grt_pointer].header.stamp.toSec();
    //std::cout<<"time_tf_slam:"<<time_tf_slam<<std::endl;
    //std::cout<<"time_grt:"<<time_grt<<' ';
    //std::cout<<"num grt:"<<g_data.grt_msg_vector.size()<<std::endl;

    //move pointer to closest one
    while( time_grt < time_tf_slam && g_data.grt_pointer+1 < g_data.grt_msg_vector.size()){
        g_data.grt_pointer ++;
        time_grt = g_data.grt_msg_vector[g_data.grt_pointer].header.stamp.toSec();
        //std::cout<<time_grt<<' ';
    }
    //std::cout<<std::endl<<"final time_grt:"<<time_grt<<' ';
    if(std::abs(time_tf_slam - time_grt) > 0.1){
        std::cout<<"too far"<<std::endl;
        //only save path
        nav_msgs::Odometry & tf_slam_msg = g_data.tf_slam_buff;
        geometry_msgs::PoseStamped pose_stamped_msg;
        pose_stamped_msg.pose   = tf_slam_msg.pose.pose;
        pose_stamped_msg.header = tf_slam_msg.header;
        g_data.path_full.poses.push_back(pose_stamped_msg);
        return false;
    }
    //check grt jump
    geometry_msgs::PoseStamped & grt_tf_msg = g_data.grt_msg_vector[g_data.grt_pointer];
    geometry_msgs::PoseStamped & last_tf_msg = g_data.grt_msg_vector[g_data.grt_pointer - 1];
    /*if(grt_tf_msg.pose.position.z - last_tf_msg.pose.position.z > 10 ||
       grt_tf_msg.pose.position.z < -20){
        //grt_tf_msg.pose.pose.position.z = last_tf_msg.pose.pose.position.z;
        continue;
    }*/ //not used in
    //valid
    if(g_data.step == 0){
        g_data.T_grt_body0 = PoseStamp2transf(grt_tf_msg);
    }
    g_data.T_grt_body1 = PoseStamp2transf(grt_tf_msg);
    g_data.T_lidar0_lidar1 = Odometry2transf(g_data.tf_slam_buff);
    g_data.step++;
    std::cout<<"valid count:"<<g_data.step<<std::endl;

    //save grt pose and path
    saveOnePoseAndPath(grt_tf_msg, g_data.pose_grt, g_data.path_grt);
    //save tf slam
    geometry_msgs::PoseStamped pose_stamped_msg;
    pose_stamped_msg.pose   =  g_data.tf_slam_buff.pose.pose;
    pose_stamped_msg.header =  g_data.tf_slam_buff.header;
    saveOnePoseAndPath(pose_stamped_msg, g_data.pose, g_data.path);
    g_data.path_full.poses.push_back(pose_stamped_msg);
    //calculate path legth
    if(g_data.step > 1) g_data.trj_length += sqrt(
                pow(pose_stamped_msg.pose.position.x - g_data.pose.point(0,g_data.pose.num_point-2), 2) +
                pow(pose_stamped_msg.pose.position.y - g_data.pose.point(1,g_data.pose.num_point-2), 2) +
                pow(pose_stamped_msg.pose.position.z - g_data.pose.point(2,g_data.pose.num_point-2), 2));

    //find the transform between slam path and grt path, or named calibrate
    //if(g_data.trj_length < 2000 * 0.3){
    g_data.T_body_lidar = computeExtrinsic( g_data.pose, g_data.pose_grt);
    //}
    PointMatrix tmp_pose = g_data.pose;
    double past_50error;
    //std::cout<<"bef_error "<<computeError(tmp_pose, g_data.pose_grt, past_50error)<<std::endl;
    g_data.T_body_lidar = g_data.T_body_lidar.inverse();
    tmp_pose.trans(g_data.T_body_lidar);
    final_avg_error = computeError(tmp_pose, g_data.pose_grt, past_50error);
    std::cout<<"avg_error "<<final_avg_error<<std::endl;
    std::cout<<"past50_error "<<past_50error<<std::endl;

    //rotation error
    //path->path_cal
    //rotationError();

    std::cout<<"---"<<std::endl;
    return true;
}

void groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr & ground_truth_msg){
    //used in optitrack system
    ROS_DEBUG("GroundTruth seq: [%d]", ground_truth_msg->header.seq);
    //save grt path
    geometry_msgs::PoseStamped tmp_msg = *ground_truth_msg;
    g_data.grt_msg_vector.push_back(tmp_msg);
    //std::cout<<"size of buff"<<g_data.grt_msg_vector.size()<<std::endl;
}
void groundTruthUavCallback(const nav_msgs::Odometry::ConstPtr & odom_msg)
{
    //used in uav system
    ROS_DEBUG("GroundTruthUAV time: [%f]", odom_msg->header.stamp.toSec());
    geometry_msgs::PoseStamped pose_tmp;
    pose_tmp.header = odom_msg->header;
    pose_tmp.pose = odom_msg->pose.pose;
    g_data.grt_msg_vector.push_back(pose_tmp);
    //std::cout<<"size of buff"<<g_data.grt_msg_vector.size()<<std::endl;
}
void transSlamCallback(const nav_msgs::Odometry::ConstPtr & odom_msg)
{
    //receive tf
    ROS_DEBUG("transSlamCallback time: [%f]", odom_msg->header.stamp.toSec());
    //save tf slam path
    g_data.tf_slam_buff = *odom_msg;
    g_data.slam_tf_msg_vector.push_back(*odom_msg);
    computeOnlineError();
}
void pclCallback(const sensor_msgs::PointCloud2::ConstPtr & pcl_msg)
{
    ROS_DEBUG("PointCloud seq: [%d]", pcl_msg->header.seq);
    g_data.registered_pcl_queue.push(*pcl_msg);
}

int main(int argc, char **argv){
    // grt: ground truth
    // 0 compare online path with grt (online estimate extrinsic), grt can be read from txt file or ros topic
    // 1 receive path and store in txt
    // 2 accumulate registered raw pcl (for map comparison)

    //ros initialize
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    //std::cout<<std::setprecision(5)<<setiosflags(std::ios::fixed);
    ros::init(argc, argv, "evaluater");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);//Debug Info
    ros::Rate r(20);//Hz
    nh.param("evaluater/mode", g_data.mode,  0);

    param.init(nh);
    g_data.logInit(param.file_loc);

    //process
    std::cout<<"====START===="<<std::endl;
    while(nh.ok()){
        if(g_data.mode  == 0){
            ros::Subscriber trans_slam_sub       = nh.subscribe("/slam_odom", 500, transSlamCallback);//from slam
            ros::Subscriber grt_uav_sub, grt_sub;
            if(!param.read_grt_online){
                //grt_sub     = nh.subscribe("/pose", 500, groundTruthCallback);//vicon
                grt_uav_sub = nh.subscribe("/pose", 500, groundTruthUavCallback);//gps+imu
            }else{
                std::ifstream file_grtpath_read;
                file_grtpath_read.open (param.file_loc_grt_path, std::ios::in);
                if(!file_grtpath_read){
                    ROS_WARN("Can not read grt_path file");
                }
                geometry_msgs::PoseStamped tmp_pose_msg;
                while(nh.ok() && txt2PoseMsg(file_grtpath_read, tmp_pose_msg)){
                    g_data.grt_msg_vector.push_back(tmp_pose_msg);
                }
                std::cout<< "number pose read: " << g_data.grt_msg_vector.size() <<std::endl;
            }
            bool first = true;
            while(nh.ok() && (  g_data.grt_msg_vector.empty())){
                ros::spinOnce();
                if(first){
                    first = false;
                    if(g_data.grt_msg_vector.size() < 2){
                        std::cout<<"waiting for grt"<<std::endl;
                    }
                }
                r.sleep();
            }
            while(nh.ok()){
                ros::spinOnce();
                r.sleep();
            }
            g_data.saveResult();
        }

        else if(g_data.mode  == 1){
            //ros::Subscriber grt_sub     = nh.subscribe("/pose", 500, groundTruthCallback);//vicon
            ros::Subscriber grt_uav_sub = nh.subscribe("/pose", 500, groundTruthUavCallback);//gps+imu
            //save path
            while(nh.ok()){
                ros::spinOnce();
                r.sleep();
            }
            g_data.savePath2TxtTum(g_data.file_loc_path_grt_wrt, g_data.grt_msg_vector);
        }

        else if(g_data.mode  == 2){
            ros::Subscriber pointcloud_sub       = nh.subscribe("/velodyne_points", 10, pclCallback);
            while (nh.ok() && g_data.registered_pcl_queue.empty()) {
                ros::spinOnce();
            }
            while(!g_data.registered_pcl_queue.empty()){
                static int skip_i = 0;
                int skip_step = 5;
                skip_i ++;
                if(skip_i % skip_step == 0){
                    pcl::PointCloud<pcl::PointXYZ> rg_pcl;
                    pcl::fromROSMsg(g_data.registered_pcl_queue.front(), rg_pcl);
                    g_data.pcl_raw_all = g_data.pcl_raw_all + rg_pcl;
                    std::cout<<"step: "<< skip_i <<"num_point_all_raw_point: "<<g_data.pcl_raw_all.points.size()<<"\n";//std::endl;
                }
                g_data.registered_pcl_queue.pop();
            }
        }
        else if(g_data.mode  ==3){
        }
        else if(g_data.mode  ==4) {
        }
    }
    //std::cout<<"T_body_lidar\n"<<g_data.T_body_lidar<<std::endl;

    std::cout<<"done"<<std::endl;
    return 0;
}
