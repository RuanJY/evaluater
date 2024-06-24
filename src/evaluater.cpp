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

Transf computeAlignTransformation(PointMatrix & scan_glb, PointMatrix & scan_now){

    Transf T = Eigen::MatrixXd::Identity(4, 4);
    if((scan_glb.num_point == 0) || (scan_now.num_point == 0)){
        ROS_ERROR("0 point in computeRT");
        return T;
    }
    if((scan_glb.num_point != scan_now.num_point)){
        ROS_ERROR("Different num of points in computeRT");
        std::cout << "num1 " << scan_glb.num_point << " num2 " << scan_now.num_point << std::endl;
        return T;
    }
    scan_glb.mse_eig();
    scan_now.mse_eig();

    Eigen::MatrixXd M = ((scan_now.point.leftCols(scan_now.num_point).colwise() - scan_now.gravity) *
                        ((scan_glb.point.leftCols(scan_glb.num_point).colwise() - scan_glb.gravity).transpose())) / scan_now.num_point;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV );
    double detVU =  (svd.matrixV()*(svd.matrixU().adjoint())).determinant();
    Eigen::Matrix<double, 3, 3> I;
    I << 1,0,0, 0,1,0, 0,0,detVU;
    Eigen::MatrixXd rotate = svd.matrixV()*I *(svd.matrixU().adjoint());
    Eigen::Matrix<double, 3, 1 > transf = scan_glb.gravity - (rotate * scan_now.gravity);
    T << rotate, transf, 0,0,0,1;
    //T = T.inverse();
    //std::cout << "Rnew:" << std::endl << T << std::endl;
    return T;
}
double rotationError(){
    double trans_error = 0, rotation_error = 0, ori_trans_error = 0;
    Transf trans_bef_cal, trans_aft_cal; int i;
    for(i=0; i<g_data.path_slam.poses.size(); i++){
        geometry_msgs::PoseStamped path_pose = g_data.path_slam.poses[i];
        geometry_msgs::PoseStamped pose_grt  = g_data.path_grt.poses[i];

        //pose->Transf
        trans_bef_cal = PoseStamp2transf(path_pose);
        //std::cout<<"trans_bef_cal\n"<<trans_bef_cal<<std::endl;
        //std::cout<<"g_data.T_grt_lidar\n"<<g_data.T_grt_lidar<<std::endl;
        //calibrated
        //trans_aft_cal = g_data.T_grt_lidar * trans_bef_cal;
        trans_aft_cal = trans_bef_cal * g_data.T_grt_lidar;

        //std::cout<<"trans_aft_cal\n"<<trans_aft_cal<<std::endl;
        //eror
        trans_error += sqrt(pow(trans_aft_cal(0, 3) - g_data.pose_grt.point(0, i), 2) +
                            pow(trans_aft_cal(1, 3) - g_data.pose_grt.point(1, i), 2) +
                            pow(trans_aft_cal(2, 3) - g_data.pose_grt.point(2, i), 2));
/*        ori_trans_error += sqrt(pow(g_data.position_slam.point(0, i) - g_data.pose_grt.point(0, i), 2) +
                                pow(g_data.position_slam.point(1, i) - g_data.pose_grt.point(1, i), 2) +
                                pow(g_data.position_slam.point(2, i) - g_data.pose_grt.point(2, i), 2));*/
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
double computeTranslationError(PointMatrix & _scan_glb, PointMatrix & _scan_now, double & _past_50error){

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
void transformPath(const nav_msgs::Path& ori_path, nav_msgs::Path& after_path, const Eigen::Matrix4d& transformationMatrix)
{// Apply transformation to position and orientation

    after_path.poses.clear();
    for (const auto& pose : ori_path.poses)
    {
        // Apply transformation to position
        Eigen::Vector4d position(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 1.0);
        Eigen::Vector4d transformedPosition = transformationMatrix * position;
        geometry_msgs::PoseStamped tmp_pose;
        tmp_pose.pose.position.x = transformedPosition(0);
        tmp_pose.pose.position.y = transformedPosition(1);
        tmp_pose.pose.position.z = transformedPosition(2);

        // Apply transformation to orientation
        Eigen::Quaterniond orientation(
                pose.pose.orientation.w,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z
        );
        Eigen::Matrix3d ori_rotation_matrix = orientation.toRotationMatrix();

        Eigen::Matrix3d after_rotationMatrix = transformationMatrix.block(0, 0, 3, 3) * ori_rotation_matrix;
        Eigen::Quaterniond transformedOrientation(after_rotationMatrix);
        tmp_pose.pose.orientation.w = transformedOrientation.w();
        tmp_pose.pose.orientation.x = transformedOrientation.x();
        tmp_pose.pose.orientation.y = transformedOrientation.y();
        tmp_pose.pose.orientation.z = transformedOrientation.z();

        after_path.poses.push_back(tmp_pose);
    }
}
Data_store::Data_store(){
    //pose       = Eigen::MatrixXd::Zero(3, max_size);
    //first_trans = lidar_trans = grt_first_trans = trf_odom_now = trf_odom_last = Eigen::MatrixXd::Identity(4,4);

    pcl_raw_all.height = 1;
    pcl_raw_all.width = 0;
}
void Data_store::logInit(const std::string & file_loc){
    //path
    //size_t length = file_loc.size() - 4;
    //std::string tmp_file_loc = file_loc.substr(0, length);
    std::cout << "tend to save report in: " << file_loc << std::endl;
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
    std::cout << "Saving result..." << std::endl;
    std::cout<<std::setprecision(10)<<setiosflags(std::ios::fixed);
    //kitti like RTE compute
    Transf transformation_error = (T_world_grt0 * T_grt_lidar * T_lidar0_lidar1).inverse() * T_world_grt1 * T_grt_lidar;
    double translation_error = sqrt(transformation_error(0, 3) * transformation_error(0, 3) +
                                    transformation_error(1, 3) * transformation_error(1, 3) +
                                    transformation_error(2, 3) * transformation_error(2, 3));
    double start_to_end_displacement = sqrt(T_lidar0_lidar1(0, 3) * T_lidar0_lidar1(0, 3)
                                          + T_lidar0_lidar1(1, 3) * T_lidar0_lidar1(1, 3)
                                          + T_lidar0_lidar1(2, 3) * T_lidar0_lidar1(2, 3));

    State state_error = trans32quat2state(transformation_error);
    double rotation_error = std::abs(state_error(3,0)) + std::abs(state_error(4,0)) + std::abs(state_error(5,0));
    //print
    std::cout
            << "step: " << step << "\n"
            << "trajectory_length: " << trj_length << std::endl

            << "RMSE translation " << final_avg_error << std::endl
            << "start_to_end_displacement " << start_to_end_displacement << std::endl
            << "translation_error: " << translation_error << std::endl
            << "rel_translation_error: " << 100 * translation_error/trj_length << "%"<<std::endl
            << "transformation_error: \n" << transformation_error << std::endl

            //<< "rotation_error: " << rotation_error << std::endl
            << "T_grt_lidar: \n" << T_grt_lidar << std::endl
            //<< "T_world_grt0: \n"     << T_world_grt0 << std::endl
            << "T_lidar0_lidar1: \n" << T_lidar0_lidar1 << std::endl
            //<< "T_world_grt1: \n"     << T_world_grt1
            << std::endl;
    //std::cout<<"avg_error "<<final_avg_error<<std::endl;

    //std::cout << "\nERROR : \n" <<sqrt(pow(pose(0,step)-254.513,2) + pow(pose(1,step)-658.088,2) + pow(pose(2,step)-68.2958,2))<< std::endl;
    //save report
    if(file_loc_report_wrt){
        file_loc_report_wrt
                << "step: " << step << "\n"
                << "trajectory_length: " << trj_length << std::endl

                << "RMSE translation " << final_avg_error << std::endl
                << "start_to_end_displacement " << start_to_end_displacement << std::endl
                << "translation_error: " << translation_error << std::endl
                << "rel_translation_error: " << 100 * translation_error/trj_length << "%"<<std::endl
                << "transformation_error: \n" << transformation_error << std::endl

                //<< "rotation_error: " << rotation_error << std::endl
            << "T_grt_lidar: \n" << T_grt_lidar << std::endl
            //<< "T_world_grt0: \n"     << T_world_grt0 << std::endl
            << "T_lidar0_lidar1: \n" << T_lidar0_lidar1 << std::endl
            //<< "T_world_grt1: \n"     << T_world_grt1
            << std::endl;
        std::cout << "Result saved in: " << param.file_loc << std::endl;
        file_loc_report_wrt.close();
    }
    else{
        std::cout << param.file_loc <<" Result not saved" << std::endl;
    }
    //save path
    savePath2Txt(file_loc_path_wrt,     path_slam);
    savePath2Txt(file_loc_path_cal_wrt, path_slam_aligned);
    savePath2Txt(file_loc_path_full_wrt, path_slam_full);
    savePath2Txt(file_loc_path_grt_wrt, path_grt);
    //savePath2TxtTum(file_loc_path_full_wrt, slam_tf_msg_vector);

    if(mode == 2){
//        std:: cout << file_loc_rawPcl << std::endl;
//        std:: cout << pcl_raw_all.width * pcl_raw_all.height << std::endl;
        pcl::io::savePCDFileASCII(file_loc_rawPcl, pcl_raw_all);
    }
}
void   Data_store::imuUpdatePose(Transf & now_slam_trans){
    ROS_DEBUG("imuUpdatePose");
    Point _pose;
    _pose = trans3D(0, 0, 0, now_slam_trans);
    //pose.addPoint(_pose);
    if(step>0) trj_length += sqrt(pow(_pose(0,0) - position_slam.point(0, step - 1), 2) +
                                  pow(_pose(1,0) - position_slam.point(1, step - 1), 2) +
                                  pow(_pose(2,0) - position_slam.point(2, step - 1), 2));

    pushPath(Slam, now_slam_trans);
    //save path and grt_path to txt
    //savePathEveryStep2Txt(file_loc_path_gdt_wrt, path_grt);
    //savePathEveryStep2Txt(file_loc_path_wrt, path);
}
void Param::init(ros::NodeHandle& nh){
    nh.param("evaluater/file_loc", file_loc, std::string("~"));
    nh.param("evaluater/file_loc_grt_path", file_loc_grt_path, std::string("~"));
    nh.param("evaluater/read_grt_txt", read_grt_txt, false);
    nh.param("evaluater/timestamp_valid_thr", timestamp_valid_thr, 0.15);
}
bool saveOnePoseAndPath(geometry_msgs::PoseStamped & pose_msg, PointMatrix & pose, nav_msgs::Path & path){

    Point tmp_point;
    tmp_point << pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z;
    pose.addPoint(tmp_point);
    path.poses.push_back(pose_msg);
}
void findGrtAndSlamPoseCorrespondenceByTimeStamp(geometry_msgs::PoseStamped & grt_pose_msg){

    double time_slam, time_grt;
    if(g_data.grt_msg_vector.size() < 2){
        return;
    }
    //for this slam pose
    time_slam = g_data.tf_slam_buff.header.stamp.toSec();
    //find a grt pose with the closest time
    time_grt = g_data.grt_msg_vector[g_data.grt_pointer].header.stamp.toSec();
    //std::cout<<"time_slam:"<<time_slam<<std::endl;
    //std::cout<<"time_grt:"<<time_grt<<' ';
    //std::cout<<"num grt:"<<g_data.grt_msg_vector.size()<<std::endl;

    //move the pointer, grt is older than the slam pose
    while(time_grt < time_slam && g_data.grt_pointer + 1 < g_data.grt_msg_vector.size()){
        g_data.grt_pointer ++;
        time_grt = g_data.grt_msg_vector[g_data.grt_pointer].header.stamp.toSec();
        //std::cout<<time_grt<<' ';
    }
    if(g_data.grt_pointer + 1 < g_data.grt_msg_vector.size()){
        //if found the older grt, then find the newer grt
        double a_step_forward_time_grt = g_data.grt_msg_vector[g_data.grt_pointer + 1].header.stamp.toSec();
        //chose the closer one
        if(abs(a_step_forward_time_grt - time_slam) < abs(time_grt - time_slam)){
            g_data.grt_pointer ++;
            time_grt = a_step_forward_time_grt;
        }
    }
    //std::cout<<std::endl<<"final time_grt:"<<time_grt<<' ';
    //check timestamp not too far
    if(std::abs(time_slam - time_grt) > param.timestamp_valid_thr){
        std::cout << "no grt timestamp close to this frame pose: " << std::abs(time_slam - time_grt) << std::endl;
        //only save path
        nav_msgs::Odometry & tf_slam_msg = g_data.tf_slam_buff;
        geometry_msgs::PoseStamped pose_stamped_msg;
        pose_stamped_msg.pose   = tf_slam_msg.pose.pose;
        pose_stamped_msg.header = tf_slam_msg.header;
        g_data.path_slam_full.poses.push_back(pose_stamped_msg);
        return;
    }
    grt_pose_msg = g_data.grt_msg_vector[g_data.grt_pointer];
    geometry_msgs::PoseStamped & last_tf_msg = g_data.grt_msg_vector[g_data.grt_pointer - 1];
    /*if(grt_pose_msg.pose.position.z - last_tf_msg.pose.position.z > 10 ||
       grt_pose_msg.pose.position.z < -20){
        //grt_pose_msg.pose.pose.position.z = last_tf_msg.pose.pose.position.z;
        continue;
    }*/ //not used in
}
void updateTransformation(geometry_msgs::PoseStamped & grt_pose_msg){

    if(g_data.step == 0){
        g_data.T_world_grt0 = PoseStamp2transf(grt_pose_msg);
        g_data.T_lidar0 = Odometry2transf(g_data.tf_slam_buff);
    }
    g_data.T_world_grt1 = PoseStamp2transf(grt_pose_msg);
    g_data.T_lidar0_lidar1 = g_data.T_lidar0.inverse() * Odometry2transf(g_data.tf_slam_buff);

    //push grt pose and path
    saveOnePoseAndPath(grt_pose_msg, g_data.pose_grt, g_data.path_grt);
    //push tf slam
    geometry_msgs::PoseStamped pose_stamped_msg;
    pose_stamped_msg.pose   =  g_data.tf_slam_buff.pose.pose;
    pose_stamped_msg.header =  g_data.tf_slam_buff.header;
    saveOnePoseAndPath(pose_stamped_msg, g_data.position_slam, g_data.path_slam);
    g_data.path_slam_full.poses.push_back(pose_stamped_msg);

    //calculate path legth
    if(g_data.step > 1){
        g_data.trj_length += sqrt(
                pow(pose_stamped_msg.pose.position.x - g_data.position_slam.point(0, g_data.position_slam.num_point - 2), 2) +
                pow(pose_stamped_msg.pose.position.y - g_data.position_slam.point(1, g_data.position_slam.num_point - 2), 2) +
                pow(pose_stamped_msg.pose.position.z - g_data.position_slam.point(2, g_data.position_slam.num_point - 2), 2));
    }
}
bool computeOnlineError(){
    //for one slam pose, find the closest (in time) grt pose, align two path, and calculate RMSE error
    geometry_msgs::PoseStamped grt_tf_msg;
    findGrtAndSlamPoseCorrespondenceByTimeStamp(grt_tf_msg);
    //valid
    updateTransformation(grt_tf_msg);
    g_data.step ++;
    std::cout<<"valid count:"<<g_data.step<<std::endl;

    //find the transform between slam path and grt path, or named calibrate
    //if(g_data.trj_length < 2000 * 0.3){//partially align
    g_data.T_grt_lidar = computeAlignTransformation(g_data.position_slam, g_data.pose_grt);
    //}
    PointMatrix grt_pose_aligned = g_data.pose_grt;
    double past_50error;
    //std::cout << "bef_error " << computeTranslationError(grt_pose_aligned, g_data.position_slam, past_50error) << std::endl;
    //g_data.T_grt_lidar = g_data.T_grt_lidar.inverse();
    grt_pose_aligned.trans(g_data.T_grt_lidar);
    g_data.final_avg_error = computeTranslationError(grt_pose_aligned, g_data.position_slam, past_50error);
    std::cout<<"avg_error "<<g_data.final_avg_error<<std::endl;
    std::cout<<"past50_error "<<past_50error<<std::endl;

    //transformPath(g_data.path_slam, g_data.path_slam_aligned, g_data.T_grt_lidar);
    transformPath(g_data.path_grt, g_data.path_grt_aligned, g_data.T_grt_lidar);
    //publish a nav_msgs::Path, g_data.path_grt_aligned

    g_data.path_pub.publish(g_data.path_grt_aligned);

    //rotation error
    //path->path_slam_aligned
    //rotationError();

    std::cout<<"---"<<std::endl;
    return true;
}

void groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr & ground_truth_msg){
    //used in optitrack system
    ROS_DEBUG("GroundTruth seq: [%d]", ground_truth_msg->header.seq);
    //save grt path
    g_data.grt_msg_vector.push_back(*ground_truth_msg);
    //std::cout<<"size of buff"<<g_data.grt_msg_vector.size()<<std::endl;
    static bool first = true;
    if(first){
        g_data.path_grt.header.frame_id  = ground_truth_msg->header.frame_id;
        g_data.path_grt_aligned.header.frame_id =  ground_truth_msg->header.frame_id;
        first = false;
    }
}
void groundTruthUavCallback(const nav_msgs::Odometry::ConstPtr & odom_msg)
{
    //used in uav system
    ROS_DEBUG("GroundTruthUAV time: [%f]", odom_msg->header.stamp.toSec());
    //convert to PoseStamped
    geometry_msgs::PoseStamped pose_tmp;
    pose_tmp.header = odom_msg->header;
    pose_tmp.pose = odom_msg->pose.pose;
    g_data.grt_msg_vector.push_back(pose_tmp);
    //std::cout<<"size of buff"<<g_data.grt_msg_vector.size()<<std::endl;
    static bool first = true;
    if(first){
        g_data.path_grt.header.frame_id  = odom_msg->header.frame_id;
        g_data.path_grt_aligned.header.frame_id =  odom_msg->header.frame_id;
        first = false;
    }
}
void transSlamCallback(const nav_msgs::Odometry::ConstPtr & odom_msg)
{
    //receive tf
    ROS_DEBUG("transSlamCallback time: [%f]", odom_msg->header.stamp.toSec());
    //save tf slam path
    g_data.tf_slam_buff = *odom_msg;
    g_data.slam_tf_msg_vector.push_back(*odom_msg);
    if(g_data.mode  == 0){
        computeOnlineError();
        g_data.path_grt_aligned.header.frame_id =  odom_msg->header.frame_id;
    }
    static bool first = true;
    if(first){
        g_data.path_slam.header.frame_id = odom_msg->header.frame_id;
        g_data.path_slam_full.header.frame_id = odom_msg->header.frame_id;
        first = false;
    }

}
void pclCallback(const sensor_msgs::PointCloud2::ConstPtr & pcl_msg)
{
    ROS_DEBUG("PointCloud seq: [%d]", pcl_msg->header.seq);
    g_data.registered_pcl_queue.push(*pcl_msg);
}
void readGrtFromTxt(){

    std::ifstream file_grtpath_read;
    file_grtpath_read.open (param.file_loc_grt_path, std::ios::in);
    if(!file_grtpath_read){
        ROS_WARN("Can not read grt_path file");
    }
    geometry_msgs::PoseStamped tmp_pose_msg;
    while(txt2PoseMsg(file_grtpath_read, tmp_pose_msg)){
        g_data.grt_msg_vector.push_back(tmp_pose_msg);
    }
    std::cout<< "number pose read: " << g_data.grt_msg_vector.size() <<std::endl;
}

int main(int argc, char **argv){
    // grt: ground truth
    // 0 compare online path with grt (online estimate extrinsic), grt can be read from txt file or ros topic
    // 1 receive path and store in txt
    // 2 accumulate registered raw pcl (for map comparison)

    //ros initialize
    ros::init(argc, argv, "evaluater");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);//Debug Info
    ros::Rate r(20);//Hz
    nh.param("evaluater/mode", g_data.mode,  0);
    ros::Subscriber grt_uav_sub, grt_sub, trans_slam_sub;
    g_data.path_pub = nh.advertise<nav_msgs::Path>("/path_grt_aligned", 1);

    param.init(nh);
    g_data.logInit(param.file_loc);

    //process
    std::cout<<"====START===="<<std::endl;
    while(nh.ok()){
        if(g_data.mode  == 0){//compare online path with grt
            trans_slam_sub = nh.subscribe("/slam_odom", 500, transSlamCallback);//from slam
            if(param.read_grt_txt){
                readGrtFromTxt();//read 6dof grt path
            }else{
                //grt_sub = nh.subscribe("/pose", 500, groundTruthCallback);//vicon, msg type: geometry_msgs::PoseStamped
                grt_uav_sub = nh.subscribe("/pose", 500, groundTruthUavCallback);//gps+imu, msg type: nav_msgs::Odometry
            }
            //wait for the first message
            bool first = true;
            while(nh.ok() && (g_data.grt_msg_vector.empty())){
                ros::spinOnce();
                if(first){
                    first = false;
                    if(g_data.grt_msg_vector.size() < 2){
                        std::cout<<"waiting for grt"<<std::endl;
                    }
                }
                r.sleep();
            }
            //process intrigued by slam pose
            while(nh.ok()){
                ros::spinOnce();
                r.sleep();
            }
            //save result
            g_data.saveResult();
        }

        else if(g_data.mode  == 1){//just receive path and store in txt

            //grt_sub = nh.subscribe("/pose", 500, groundTruthCallback);//vicon, msg type: geometry_msgs::PoseStamped
            grt_uav_sub = nh.subscribe("/pose", 500, groundTruthUavCallback);//gps+imu, msg type: nav_msgs::Odometry
            //save path
            while(nh.ok()){
                ros::spinOnce();
                r.sleep();
                std::cout << "size of pose: " << g_data.grt_msg_vector.size() << std::endl;
            }
            g_data.savePath2TxtTum(g_data.file_loc_path_grt_wrt, g_data.grt_msg_vector);
        }

        else if(g_data.mode == 2){//accumulate registered raw pcl
            ros::Subscriber pointcloud_sub  = nh.subscribe("/velodyne_points", 10, pclCallback);
            while (nh.ok()){
                while(nh.ok() && g_data.registered_pcl_queue.empty()) {
                    ros::spinOnce();
                }
                while(nh.ok() && !g_data.registered_pcl_queue.empty()){
                    static int step_i = 0;
                    int skip_step = 5;//skip some frames to limit the size of final pcd
                    step_i ++;
                    if(step_i % skip_step == 0){
                        pcl::PointCloud<pcl::PointXYZI> registered_pcl;
                        pcl::fromROSMsg(g_data.registered_pcl_queue.front(), registered_pcl);
                        //accumulate point cloud, your memory may be run out
                        g_data.pcl_raw_all = g_data.pcl_raw_all + registered_pcl;
                        std::cout << "step: " << step_i << " num_point_accumulated_point: " << g_data.pcl_raw_all.points.size() / 1000 << " k\n";
                    }
                    g_data.registered_pcl_queue.pop();
                }
            }
            g_data.saveResult();
        }
        else if(g_data.mode  ==3){
        }
        else if(g_data.mode  ==4) {
        }
    }
    //std::cout<<"T_grt_lidar\n"<<g_data.T_grt_lidar<<std::endl;

    std::cout<<"done"<<std::endl;
    return 0;
}
