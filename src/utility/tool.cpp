#include "tool.h"

//format transform
Transf state2trans3(State state){

    Transf result_exd_R;
    Eigen::Matrix3d  rotate = Eigen::Matrix3d::Identity();
    //Eigen::AngleAxisd rotationVector(M_PI/360,Eigen::Vector3d(90,0,0));//roll pintch yaw
    //rotationVector << state(3,0),state(4,0),state(5,0);not a matrix, can not initialized in this way
    //rotate = rotationVector.toRotationMatrix();
    Eigen::Matrix<double, 3, 1 > _transf;
    _transf << state(0,0), state(1,0), state(2,0);
    double roll=state(3,0), pintch=state(4,0), yaw=state(5,0);
    rotate << cos(yaw)*cos(pintch)-sin(roll)*sin(yaw)*sin(pintch), -1*cos(roll)*sin(yaw), cos(yaw)*sin(pintch)+cos(pintch)*sin(roll)*sin(yaw),
            cos(pintch)*sin(yaw)+cos(yaw)*sin(roll)*sin(pintch), cos(roll)*cos(yaw), sin(yaw)*sin(pintch)-cos(yaw)*cos(pintch)*sin(roll),
            -1*cos(roll)*sin(pintch), sin(roll), cos(roll)*cos(pintch);
    result_exd_R << rotate, _transf, 0,0,0,1;
    return result_exd_R;
}
Transf state2quat2trans3(State state){

    Transf result_exd_R;
    tf::Quaternion q_tmp;
    Eigen::Matrix3d  rotate;
    Eigen::Matrix<double, 3, 1 > _transf;

    double _roll, _pitch, _yaw;
    _roll = state(3,0) ;
    _pitch = state(4,0);
    _yaw = state(5,0);
    q_tmp.setRPY(_roll, _pitch, _yaw);
    tf::Matrix3x3 matrix(q_tmp);
    rotate << matrix[0][0], matrix[0][1], matrix[0][2],
            matrix[1][0], matrix[1][1], matrix[1][2],
            matrix[2][0], matrix[2][1], matrix[2][2];
    _transf << state(0,0), state(1,0), state(2,0);

    result_exd_R << rotate, _transf, 0,0,0,1;
    //std::cout<<"state2quat2trans3 input: \n"<<state<<"\n";
    //std::cout<<"state2quat2trans3 result: \n"<<result_exd_R<<"\n";
    return result_exd_R;
}
Transf state2quat2trans3_down(State state){

    Transf result_exd_R;
    tf::Quaternion q_tmp;
    Eigen::Matrix3d  rotate;
    Eigen::Matrix<double, 3, 1 > _transf;

    double _roll, _pitch, _yaw;
    _pitch = state(3,0);
    _roll = state(4,0);
    _yaw =  state(5,0);
    q_tmp.setRPY(_roll, _pitch, _yaw);
    tf::Matrix3x3 matrix(q_tmp);

    rotate << matrix[0][0], matrix[0][1], matrix[0][2],
            matrix[1][0], matrix[1][1], matrix[1][2],
            matrix[2][0], matrix[2][1], matrix[2][2];
    _transf << state(0,0), state(1,0), state(2,0);

    result_exd_R << rotate, _transf, 0,0,0,1;
    return result_exd_R;
}
State trans32state(Transf & _trans){

    State result_state;
    result_state(0, 0) = _trans(0, 3);
    result_state(1, 0) = _trans(1, 3);
    result_state(2, 0) = _trans(2, 3);
    double roll, pitch, yaw;
/*    roll = atan2(_trans(2,1), _trans(2,2));
    pitch = atan2(-1 * _trans(2, 0), sqrt(pow(_trans(2, 1), 2) + pow(_trans(2, 2), 2)));
    yaw = atan2(_trans(1,0), _trans(0,0));*/
/*    yaw = atan2(_trans(2,1), _trans(2,2));
    pitch = atan2(-1 * _trans(2, 0), sqrt(pow(_trans(2, 1), 2) + pow(_trans(2, 2), 2)));
    roll = atan2(_trans(1,0), _trans(0,0));*/
    roll = atan2(_trans(2,1), sqrt(pow(_trans(0, 1), 2) + pow(_trans(1, 1), 2)));
    pitch = atan2(-1*_trans(2,0), _trans(2,2));
    yaw = atan2(-1*_trans(0,1), _trans(1,1));
    result_state(3, 0) = roll;
    result_state(4, 0) = pitch;
    result_state(5, 0) = yaw;
    return result_state;
}
State trans32quat2state(Transf & _trans){

    State result_state;
    result_state(0, 0) = _trans(0, 3);
    result_state(1, 0) = _trans(1, 3);
    result_state(2, 0) = _trans(2, 3);
    double roll, pitch, yaw;
    tf::Matrix3x3 tmp_m(_trans(0, 0), _trans(0, 1), _trans(0, 2),
                        _trans(1, 0), _trans(1, 1), _trans(1, 2),
                        _trans(2, 0), _trans(2, 1), _trans(2, 2));
    tmp_m.getEulerYPR(yaw, pitch, roll);
    result_state(3, 0) = roll;
    result_state(4, 0) = pitch;
    result_state(5, 0) = yaw;
    return result_state;
}
sensor_msgs::PointCloud matrix2D2pclMsg(Eigen::MatrixXd _mx, int _npoint){

    if((_mx.cols()<_npoint)||(_mx.rows()<2)){
        ROS_INFO("Error! _mx has not correct point");
    }
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "sensor_frame";//"sensor_frame";

    unsigned int num_points = _npoint;
    cloud.points.resize(num_points);
    for (unsigned int i = 0; i < num_points; ++i) {
        cloud.points[i].x = _mx(0,i);
        cloud.points[i].y = _mx(1,i);
        cloud.points[i].z = 0;
    }
    return cloud;
}
sensor_msgs::PointCloud matrix3D2pclMsg(Eigen::MatrixXd _mx, int _npoint){

    if((_mx.cols()<_npoint)||(_mx.rows()<3)){
        ROS_INFO("Error! _mx has not correct point");
    }
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "velodyne";//"sensor_frame";

    unsigned int num_points = _npoint;
    cloud.points.resize(num_points);
    for (unsigned int i = 0; i < num_points; ++i) {
        cloud.points[i].x = _mx(0,i);
        cloud.points[i].y = _mx(1,i);
        cloud.points[i].z = _mx(2,i);
    }
    return cloud;
}
Eigen::Matrix<double, 1, 6> txt2matrixSeattleOdom(int _line, std::ifstream &file_odom){

    Eigen::Matrix<double, 1, 6> _odom;
    _odom.fill(0);
    char _buffer[30];
    double _irng;
    int _i = 0;
    while((_i<5) && (!file_odom.eof())){
        file_odom.getline(_buffer,30,','); //
        sscanf( _buffer, "%lf, ", &_irng );//%d mean int, %lf mean double, %f mean float
        //std::cout<<"buff: "<<_buffer<<std::endl;
        _odom(0,_i) = _irng;
        _i++;
    }
    file_odom.getline(_buffer,30,'\n'); //
    sscanf( _buffer, "%lf, ", &_irng );
    //std::cout<<"buff: "<<_buffer<<std::endl;
    _odom(0,5) = _irng;
    //std::cout<<"txt2matrixSeattleOdom result: "<<_line<<"  "<<_odom<<std::endl;
    return _odom;
}
bool txt2PoseMsg(std::ifstream &file_odom, geometry_msgs::PoseStamped & pose_msg){
    //read tum format trajectory, return a pose msg
    if(file_odom.eof()){
        std::cout << "end of file"<<std::endl;
        return false;
    }
    Eigen::Matrix<double, 1, 8> one_line_pose;
    one_line_pose.setZero();
    char read_buffer[30];
    double double_buff;
    //read one line
    int i;
    for(i=0; i<7; i++){
        if(file_odom.eof()){
            std::cout << "unexpected end of file"<<std::endl;
            return false;
        }
        file_odom.getline(read_buffer, 30, ' '); //
        //one_line_pose(i) = std::stod(read_buffer);
        sscanf(read_buffer, "%lf, ",  &double_buff);//%d mean int, %lf mean double, %f mean float
        one_line_pose(i) = double_buff;
        //std::cout << "buff: " << read_buffer <<  " double: "<<  one_line_pose(i) << std::endl;

    }
    if(file_odom.eof()){
        std::cout << "unexpected end of file"<<std::endl;
        return false;
    }
    file_odom.getline(read_buffer, 30, '\n'); //
    one_line_pose(i) = std::stod(read_buffer);
    sscanf(read_buffer, "%lf, ", &double_buff );
    one_line_pose(i) = double_buff;
    //std::cout << "buff: " << read_buffer <<  " double: "<<  one_line_pose(i) << std::endl;
    //convert to pose msg
    pose_msg.header.stamp.fromSec(one_line_pose(0));
    pose_msg.header.frame_id = "velodyne";
    pose_msg.pose.position.x = one_line_pose(1);
    pose_msg.pose.position.y = one_line_pose(2);
    pose_msg.pose.position.z = one_line_pose(3);
    pose_msg.pose.orientation.x = one_line_pose(4);
    pose_msg.pose.orientation.y = one_line_pose(5);
    pose_msg.pose.orientation.z = one_line_pose(6);
    pose_msg.pose.orientation.w = one_line_pose(7);
    return  true;
}
pcl::PointCloud<pcl::PointXYZ> matrix3D2pcl(const PointMatrix & _pmatrix){

    pcl::PointCloud<pcl::PointXYZ> pcl_result;
    pcl_result.header.frame_id = "velodyne";//"sensor_frame";

    unsigned int num_points = _pmatrix.num_point;
    pcl_result.width = _pmatrix.num_point;
    pcl_result.height = 1;
    pcl_result.points.resize(num_points);
    pcl_result.getMatrixXfMap() = _pmatrix.point.leftCols(_pmatrix.num_point).cast<float>();
    //std::cout<<pcl_result.height*pcl_result.width<<"=="<<_pmatrix.num_point<<"?"<<std::endl;
    return pcl_result;
}
void eigen2pcl(){

}
geometry_msgs::PoseWithCovariance transf2PoseWithCovariance(Transf trans) {

    geometry_msgs::PoseWithCovariance _pose;
    //pose
    _pose.pose.position.x = trans(0, 3);
    _pose.pose.position.y = trans(1, 3);
    _pose.pose.position.z = trans(2, 3);
    //orientation
    //trans->tf::matrix->roll yaw pitch->tf::Q, there is no direct way from tf::matrix to tf::Q
    tf::Matrix3x3 tmp_m(trans(0, 0), trans(0, 1), trans(0, 2),
                        trans(1, 0), trans(1, 1), trans(1, 2),
                        trans(2, 0), trans(2, 1), trans(2, 2));
    double roll, yaw, pitch;
    tmp_m.getEulerYPR(yaw, pitch, roll);
    tf::Quaternion tmp_q;
    tmp_q.setRPY(roll, pitch, yaw);
    _pose.pose.orientation.x = tmp_q.x();
    _pose.pose.orientation.y = tmp_q.y();
    _pose.pose.orientation.z = tmp_q.z();
    _pose.pose.orientation.w = tmp_q.w();

    return _pose;
}
Transf PoseWithCovariance2transf(geometry_msgs::PoseWithCovariance _pose) {

    Transf _trans;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(_pose.pose.orientation, quat);
    //tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    tf::Matrix3x3 matrix(quat);
    _trans << matrix[0][0], matrix[0][1], matrix[0][2], _pose.pose.position.x,
              matrix[1][0], matrix[1][1], matrix[1][2], _pose.pose.position.y,
              matrix[2][0], matrix[2][1], matrix[2][2], _pose.pose.position.z,
              0, 0, 0, 1;

    return _trans;
}
Transf PoseStamp2transf(geometry_msgs::PoseStamped _pose){
    Transf _trans;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(_pose.pose.orientation, quat);
    //tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    tf::Matrix3x3 matrix(quat);
    _trans << matrix[0][0], matrix[0][1], matrix[0][2], _pose.pose.position.x,
            matrix[1][0], matrix[1][1], matrix[1][2], _pose.pose.position.y,
            matrix[2][0], matrix[2][1], matrix[2][2], _pose.pose.position.z,
            0, 0, 0, 1;

    return _trans;
}
Transf Odometry2transf(const nav_msgs::Odometry& _odom){
    Transf result;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(_odom.pose.pose.orientation, quat);
    tf::Matrix3x3 matrix(quat);
    result << matrix[0][0], matrix[0][1], matrix[0][2], _odom.pose.pose.position.x,
              matrix[1][0], matrix[1][1], matrix[1][2], _odom.pose.pose.position.y,
              matrix[2][0], matrix[2][1], matrix[2][2], _odom.pose.pose.position.z,
              0, 0, 0, 1;
    return result;
}
pcl::PointCloud<pcl::PointXYZ> readKitti(int line_num){

    std::stringstream lidar_data_path;
    lidar_data_path << "/media/nuc/摄影/slam_data_set/kitti_odometry/data_odometry_velodyne/dataset/sequences/00/velodyne/"
                    << std::setfill('0') << std::setw(6) << line_num << ".bin";

    std::ifstream lidar_data_file(lidar_data_path.str(), std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);
    std::vector<float> lidar_data(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data[0]), num_elements*sizeof(float));
    std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";

    std::vector<Eigen::Vector3d> lidar_points;
    std::vector<float> lidar_intensities;
    pcl::PointCloud<pcl::PointXYZ> laser_cloud;
    for (std::size_t i = 0; i < lidar_data.size(); i += 4)
    {
        lidar_points.emplace_back(lidar_data[i], lidar_data[i+1], lidar_data[i+2]);
        lidar_intensities.push_back(lidar_data[i+3]);

        pcl::PointXYZ point;
        point.x = lidar_data[i];
        point.y = lidar_data[i + 1];
        point.z = lidar_data[i + 2];
        //point.intensity = lidar_data[i + 3];
        laser_cloud.push_back(point);
    }
    return laser_cloud;
}

//transformation
Point trans3D(int _x, int _y, int _z, const Transf& _trans){

    Eigen::Matrix<double, 4, 1> _point_expend;
    _point_expend << _x, _y, _z, 1;
    _point_expend = _trans*_point_expend;
    Point _point_turned = _point_expend.topRows(3);
    return _point_turned;
}
Eigen::MatrixXd trans3D(const Eigen::MatrixXd &bef_rotate, Eigen::Isometry3d &T){

    Eigen::MatrixXd aft_rotate;
    aft_rotate = Eigen::MatrixXd::Zero(4, 1).replicate(1, bef_rotate.cols());
    //std::cout<<"bef_rotate"<<bef_rotate<<std::endl;
    aft_rotate.topRightCorner(bef_rotate.rows(), bef_rotate.cols()) = bef_rotate;
    aft_rotate = T.matrix() * aft_rotate;
    //std::cout<<"aft_rotate"<<aft_rotate<<std::endl;
    return aft_rotate.topRightCorner(bef_rotate.rows(), bef_rotate.cols());
}
Transf createTrans(double x, double y, double z, double roll, double pitch, double yaw){

    //theta unit: degree, just for convenience
    State state;
    //state << x, y, z, pitch / 180.0 * M_PI, roll / 180.0 * M_PI, yaw / 180.0 * M_PI;
    state << x, y, z, roll / 180.0 * M_PI, pitch / 180.0 * M_PI, yaw / 180.0 * M_PI;
    Transf _result = state2quat2trans3(state);
    //Transf _result = state2trans3(state);
    return _result;
}

//filter
sensor_msgs::PointCloud2 pclMsg2VotexFilter(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg, double voxel_size) {
    //msg to pcl2
    auto *pcl2Ptr = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(pcl2Ptr);
    pcl_conversions::toPCL(*pcl_msg, *pcl2Ptr);
    //pcl2 filter
    pcl::PCLPointCloud2 msg_filtered;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(msg_filtered);
    //pcl2 to msg
    sensor_msgs::PointCloud2 point_output;
    pcl_conversions::fromPCL(msg_filtered, point_output);
    return point_output;
    //view it
/*    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudT(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pcl_msg, *cloudT);*/
    //pcl::visualization::CloudViewer viewer("Cloud Viewer");
    //viewer.showCloud(cloudT);
    //you can publish point_output
}
pcl::PointCloud<pcl::PointXYZ> pclVotexFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcl, double voxel_size) {

    auto voxel_size_f = float(voxel_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(pcl);//ptr
    sor.setLeafSize(voxel_size_f, voxel_size_f, voxel_size_f);
    sor.filter(*output);//object
    return *output;
    //view it
/*    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudT(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pcl_msg, *cloudT);*/
    //pcl::visualization::CloudViewer viewer("Cloud Viewer");
    //viewer.showCloud(cloudT);
    //you can publish point_output
}

//visualize
void scan_print(ros::Publisher& cloud_pub, PointMatrix& scan){//ok

    sensor_msgs::PointCloud cloud1 = matrix2D2pclMsg(scan.point, scan.num_point);
    cloud_pub.publish(cloud1);
}
void scan_print3D(ros::Publisher& cloud_pub, PointMatrix& scan){//ok

    sensor_msgs::PointCloud cloud1 = matrix3D2pclMsg(scan.point, scan.num_point);
    cloud_pub.publish(cloud1);
}
void pose_print(ros::Publisher& cloud_pub, Eigen::Matrix<double, 3, Eigen::Dynamic> & pose, int pose_num){//ok

    sensor_msgs::PointCloud cloud1 = matrix3D2pclMsg(pose, pose_num);
    cloud_pub.publish(cloud1);
}

//others
Point tran_error(Transf& t_last, Transf& t_new){

    Eigen::Matrix<double, 3, 1> v_last, v_new;
    v_last.topRows(2) = t_last.topRightCorner(2,1);
    v_last.row(3) = t_last.bottomRightCorner(1,1);
}
double getPosi(double x_min, double y_min, double z_min, double grid){
    double posi = x_min/double(grid)*10000 +
                  y_min/double(grid)*1 +
                  z_min/double(grid)*0.0001;// +  _direction*0.1;
    return posi;
}
float computeMSE(const PointMatrix & pM_source, const PointMatrix & pM_target){
    //target = map_glb
    pcl::PointCloud<pcl::PointXYZ> turned_pcl_source = matrix3D2pcl(pM_source);
    pcl::PointCloud<pcl::PointXYZ> turned_pcl_target = matrix3D2pcl(pM_target);
    pcl::PCLPointCloud2::Ptr cloud_target (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_source (new pcl::PCLPointCloud2 ());
    pcl::toPCLPointCloud2(turned_pcl_source, *cloud_source);
    pcl::toPCLPointCloud2(turned_pcl_target, *cloud_target);


    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_source (new pcl::PointCloud<pcl::PointXYZ> ());
    fromPCLPointCloud2 (*cloud_source, *xyz_source);
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_target (new pcl::PointCloud<pcl::PointXYZ> ());
    fromPCLPointCloud2 (*cloud_target, *xyz_target);

    pcl::PCLPointCloud2 output;
    const std::string &correspondence_type = "nn";

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_xyzi (new pcl::PointCloud<pcl::PointXYZI> ());
    output_xyzi->points.resize (xyz_source->points.size ());
    output_xyzi->height = xyz_source->height;
    output_xyzi->width = xyz_source->width;

    float rmse = 0.0f;

    if (correspondence_type == "index")
    {
//    print_highlight (stderr, "Computing using the equal indices correspondence heuristic.\n");

        if (xyz_source->points.size () != xyz_target->points.size ())
        {
            pcl::console::print_error ("Source and target clouds do not have the same number of points.\n");
            return -1;
        }

        for (std::size_t point_i = 0; point_i < xyz_source->points.size (); ++point_i)
        {
            if (!std::isfinite (xyz_source->points[point_i].x) || !std::isfinite (xyz_source->points[point_i].y) || !std::isfinite (xyz_source->points[point_i].z))
                continue;
            if (!std::isfinite (xyz_target->points[point_i].x) || !std::isfinite (xyz_target->points[point_i].y) || !std::isfinite (xyz_target->points[point_i].z))
                continue;


            float dist = squaredEuclideanDistance (xyz_source->points[point_i], xyz_target->points[point_i]);
            rmse += dist;

            output_xyzi->points[point_i].x = xyz_source->points[point_i].x;
            output_xyzi->points[point_i].y = xyz_source->points[point_i].y;
            output_xyzi->points[point_i].z = xyz_source->points[point_i].z;
            output_xyzi->points[point_i].intensity = dist;
        }
        rmse = std::sqrt (rmse / static_cast<float> (xyz_source->points.size ()));
    }
    else if (correspondence_type == "nn")
    {
//    print_highlight (stderr, "Computing using the nearest neighbor correspondence heuristic.\n");

        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
        tree->setInputCloud (xyz_target);

        for (std::size_t point_i = 0; point_i < xyz_source->points.size (); ++ point_i)
        {
            if (!std::isfinite (xyz_source->points[point_i].x) || !std::isfinite (xyz_source->points[point_i].y) || !std::isfinite (xyz_source->points[point_i].z))
                continue;

            std::vector<int> nn_indices (1);
            std::vector<float> nn_distances (1);
            if (!tree->nearestKSearch (xyz_source->points[point_i], 1, nn_indices, nn_distances))
                continue;
            std::size_t point_nn_i = nn_indices.front();

            float dist = squaredEuclideanDistance (xyz_source->points[point_i], xyz_target->points[point_nn_i]);
            rmse += dist;

            output_xyzi->points[point_i].x = xyz_source->points[point_i].x;
            output_xyzi->points[point_i].y = xyz_source->points[point_i].y;
            output_xyzi->points[point_i].z = xyz_source->points[point_i].z;
            output_xyzi->points[point_i].intensity = dist;
        }
        rmse = std::sqrt (rmse / static_cast<float> (xyz_source->points.size ()));

    }
    else if (correspondence_type == "nnplane")
    {
//    print_highlight (stderr, "Computing using the nearest neighbor plane projection correspondence heuristic.\n");

        pcl::PointCloud<pcl::Normal>::Ptr normals_target (new pcl::PointCloud<pcl::Normal> ());
        //fromPCLPointCloud2 (*cloud_target, *normals_target);

        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
        tree->setInputCloud (xyz_target);

        for (std::size_t point_i = 0; point_i < xyz_source->points.size (); ++ point_i)
        {
            if (!std::isfinite (xyz_source->points[point_i].x) || !std::isfinite (xyz_source->points[point_i].y) || !std::isfinite (xyz_source->points[point_i].z))
                continue;

            std::vector<int> nn_indices (1);
            std::vector<float> nn_distances (1);
            if (!tree->nearestKSearch (xyz_source->points[point_i], 1, nn_indices, nn_distances))
                continue;
            std::size_t point_nn_i = nn_indices.front();

            Eigen::Vector3f normal_target = normals_target->points[point_nn_i].getNormalVector3fMap (),
                    point_source = xyz_source->points[point_i].getVector3fMap (),
                    point_target = xyz_target->points[point_nn_i].getVector3fMap ();

            float dist = normal_target.dot (point_source - point_target);
            rmse += dist * dist;

            output_xyzi->points[point_i].x = xyz_source->points[point_i].x;
            output_xyzi->points[point_i].y = xyz_source->points[point_i].y;
            output_xyzi->points[point_i].z = xyz_source->points[point_i].z;
            output_xyzi->points[point_i].intensity = dist * dist;
        }
        rmse = std::sqrt (rmse / static_cast<float> (xyz_source->points.size ()));
    }
    else
    {
//    print_error ("Unrecognized correspondence type. Check legal arguments by using the -h option\n");
        return rmse;
    }

    //toPCLPointCloud2 (*output_xyzi, output);

    //pcl::console::print_highlight ("RMSE Error: %f\n", rmse);
    return rmse;
}



