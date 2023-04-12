# evaluater
My evaluator for online testing the accuracy of odometry compared with the groundtruth.
##mode 
- 0: compare online path with grt (online estimate extrinsic), grt can be read from txt file or ros topic
- 1: receive path and store it to txt file (kitti or tum format)
- 2: accumulate registered raw pcl (for map comparison)
## usage 
roslaunch evaluater run.launch
## parameters
- mode: mode
- file_loc: where to write path
- file_loc_grt_path: read grt file
- read_grt_online: read grt from ros or from txt file
- remap from="/pose" to="/mavros/local_position/odom"  this is the topic of ground truth, typically provided by GPS or motion capture system
- remap from="/slam_odom" to="/lio_sam/mapping/odometry" this is the pose published by slam
- remap from="/velodyne_points" to="/turned_velodyned"this is the registered point cloud to accumulate.
