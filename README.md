# evaluater

My evaluator for online testing the accuracy of odometry compared with the groundtruth.

## parameters
- `mode`: mode
- `file_loc`: where to write path file
- `file_loc_grt_path`: path to read grt file
- `read_grt_txt`: read grt from txt file (true) or from ros topic (false)
- `remap from="/pose" to="/mavros/local_position/odom"`  this is the topic of ground truth, typically provided by GPS or motion capture system
- `remap from="/slam_odom" to="/lio_sam/mapping/odometry"` this is the pose published by slam
- `remap from="/velodyne_points" to="/turned_velodyned"` this is the registered point cloud to accumulate.


## mode 

### 0: compare online path with ground truth, estimate extrinsic online, ground truth can be read from txt file or ros topic

set "/slam_odom" topic

if  `read_grt_txt`: set the `file_loc_grt_path`

else: set the "/pose" topic as the ground truth msg

translation error will be print in terminal like:

```
---
valid count:1886
avg_error 0.572287
past50_error 0.615518

```
Error and extrinsic between ground truth path and slam path will be sore in txt file `***_report.txt` located in `file_loc` like:

```step: 1891
avg_translation_error 0.572639
TRJ LENGTH: 783.372
T_grt_lidar: 
   0.999772  0.00942929  -0.0191394   0.0900965
-0.00951913    0.999944 -0.00460857   -0.349278
  0.0190949  0.00478971    0.999806   -0.286419
         -0           0          -0           1
T_lidar0_lidar1: 
  0.999634  0.0246512 -0.0111369 -0.0345238
-0.0242729   0.999163  0.0329132  0.0110479
  0.011939 -0.0326308   0.999396   0.716268
         0          0          0          1
```

Other report file meaning:

Those poses are corresponded one-to-one with ground truth pose according to  timestamp.

`***_path.txt` : slam path

`***_path_cal.txt` : slam path transformed to the ground truth frame

`***_path_grt.txt` : ground truth path

This is the full slam path

`***_path_full.txt` : slam path

kitti format, the file will be like:

```
0.9998635927 0.0003255862971 0.01651332668 0 -8.131516294e-20 0.9998056849 -0.01971274699 0 -0.01651653609 0.01971005803 0.9996693041 0
```

### 1: receive pose msg (slam or ground truth) and store it to txt file (kitti or tum format)

set "/pose" topic

set `file_loc`

ground truth is saved in tum format:

```
1673593778.390609 10.3305025100708 -16.45950126647949 14.21239376068115 0.9998688640400115 0.0113609339884524 0.01147543235411046 0.001263267490416942
```

### 2:accumulate registered raw pcl (for map comparison)

you can skip scans.

## usage 
```
roslaunch evaluater run.launch
```

