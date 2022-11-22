# Localization

## Dependencies
- geometry_msgs
- pcl_ros
- sensor_msgs
- tf2
- tf2_ros
- tf_conversions

## Nodes
- pub_map
  - parameters: map_path
  - subscribe: N/A
  - publish: /map (sensor_msgs::PointCloud2)
  
- localizer
  - parameters: baselink2lidar_trans (float array), baselink2lidar_rot (float array), result_save_path (string)
  - subscribe: /map (sensor_msgs::PointCloud2), /lidar_points (sensor_msgs::PointCloud2), /gps (geometry_msgs::PointStamped)
  - publish: /lidar_pose (geometry_msgs::PoseStamped), /transformed_points (sensor_msgs::PointCloud2)
  - output: result poses as csv file saved in `result_save_path`

## How to Use

- [prepare your data](#prepare-data)
- [start launch file](#launch-file)
- [play rosbag with slow speed](#play-rosbag)
- [check your result](#check-result)

### Prepare Code & Data
First, you should download your pcd map to your computer and place in src/data/
```
data/
  ├── sdc_localization_1.bag 
  ├── sdc_localization_2.bag 
  ├── sdc_localization_3.bag 
  ├── itri_map.pcd
  └── nuscenes_maps/
src/
  ├── localization/ 
  │   ├── ...
  │   └── src/
  │   	├── pub_map_node.pcd
  │   	└── localizer_node.cpp <-- Students are asked to complete this part !!
  └── map_tile_loader/
```

### Launch File
Start launch file by,  
```bash
> roslaunch localization itri.launch
# or
> roslaunch localization nuscenes.launch save_path:="/root/catkin_ws/src/localiztion/results/result_2.csv"
```

### Play Rosbag

Play rosbag with the following command.
```bash
rosbag play --pause -r 0.1 <your_bag_file> --clock
```
You can pause/resume the playing process by pressing white space on the rosbag play terminal.

It is recommanded to pause a while when the localizer node is initializing. 

### Check Result

The results will be saved in `results/` folder by default. Check if there are correct number of lines and upload your result to our competition servers.
