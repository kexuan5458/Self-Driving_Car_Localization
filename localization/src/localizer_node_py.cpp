// docker exec -it ros bash
// roscore
// roslaunch localization nuscenes.launch save_path:="/root/catkin_ws/src/localization/results/result_2.csv"
// roslaunch localization nuscenes.launch save_path:="/root/catkin_ws/src/localization/results/result_3.csv"
// rosbag play -r 0.01 --pause data/sdc_localization_2_lite.bag --clock --topic /wheel_odometry /lidar_points /gps /imu/data
// rosbag play -r 0.01 --pause data/sdc_localization_3_lite.bag --clock --topic /wheel_odometry /lidar_points /gps /imu/data

#include<iostream>
#include<fstream>
#include<limits>
#include<vector>

#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<tf/transform_broadcaster.h>
#include<tf2_eigen/tf2_eigen.h>
#include<nav_msgs/Odometry.h>


#include<Eigen/Dense>

#include<pcl/registration/icp.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl_ros/transforms.h>
#include<pcl/surface/mls.h>

#include <pybind11/pybind11.h>

class Localizer{
private:

  float mapLeafSize = 1., scanLeafSize = 1.;
  std::vector<float> d_max_list, n_iter_list;

  ros::NodeHandle _nh;
  ros::Subscriber sub_map, sub_points, sub_gps, sub_ekf;////////////////////////////////////
  ros::Publisher pub_points, pub_pose, pub_car_pose;
  tf::TransformBroadcaster br;

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_points;
  pcl::PointXYZ gps_point;
  bool gps_ready = false, map_ready = false, initialized = false;
  Eigen::Matrix4f init_guess;
  int cnt = 0;
  
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Matrix4 icp;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter; // create filter object

  std::string result_save_path;
  std::ofstream outfile;
  geometry_msgs::Transform car2Lidar;
  std::string mapFrame, lidarFrame, carFrame;/////////////////////////////////////////////////

public:
  Localizer(ros::NodeHandle nh): map_points(new pcl::PointCloud<pcl::PointXYZI>) {
    std::vector<float> trans, rot;

    _nh = nh;

    _nh.param<std::vector<float>>("baselink2lidar_trans", trans, std::vector<float>());
    _nh.param<std::vector<float>>("baselink2lidar_rot", rot, std::vector<float>());
    _nh.param<std::string>("result_save_path", result_save_path, "result.csv");
    _nh.param<float>("scanLeafSize", scanLeafSize, 1.0);
    _nh.param<float>("mapLeafSize", mapLeafSize, 1.0);
    _nh.param<std::string>("mapFrame", mapFrame, "world");
    _nh.param<std::string>("lidarFrame", lidarFrame, "nuscenes_lidar");
    // _nh.param<std::string>("carFrame", carFrame, "car");//

    ROS_INFO("saving results to %s", result_save_path.c_str());
    outfile.open(result_save_path);
    outfile << "id,x,y,z,yaw,pitch,roll" << std::endl;

    if(trans.size() != 3 | rot.size() != 4){
      ROS_ERROR("transform not set properly");
    }

    car2Lidar.translation.x = trans.at(0);
    car2Lidar.translation.y = trans.at(1);
    car2Lidar.translation.z = trans.at(2);
    car2Lidar.rotation.x = rot.at(0);
    car2Lidar.rotation.y = rot.at(1);
    car2Lidar.rotation.z = rot.at(2);
    car2Lidar.rotation.w = rot.at(3);

    sub_map = _nh.subscribe("/map", 1, &Localizer::map_callback, this);
    sub_points = _nh.subscribe("/lidar_points", 400, &Localizer::pc_callback, this);
    sub_gps = _nh.subscribe("/gps", 1, &Localizer::gps_callback, this);
    sub_ekf = _nh.subscribe("/odometry/filtered", 10, &Localizer::ekf_callback, this); /////////////////// I add this subscriber

    pub_points = _nh.advertise<sensor_msgs::PointCloud2>("/transformed_points", 1);
    pub_pose = _nh.advertise<geometry_msgs::PoseStamped>("/lidar_pose", 1);
    pub_car_pose = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/car_pose", 1);
    init_guess.setIdentity();
    ROS_INFO("%s initialized", ros::this_node::getName().c_str());

  }

  // Gentaly end the node
  ~Localizer(){
    if(outfile.is_open()) outfile.close();
  }

  void down_sampling(pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr dest_cloud){
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    // set the pointClouds which are needed to be filtered
    voxel_filter.setInputCloud(source_cloud);
    voxel_filter.setLeafSize(1.0f, 1.0f, 1.0f);
    voxel_filter.filter(*dest_cloud);
  }
  void up_sampling(pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr dest_cloud){
    pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI> filter;
    filter.setInputCloud(source_cloud);
    // create searching object
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree;
    filter.setSearchMethod(kdtree);
    filter.setSearchRadius(0.03);

    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI>::SAMPLE_LOCAL_PLANE);
    filter.setUpsamplingRadius(0.03);
    filter.setUpsamplingStepSize(0.02);
    filter.process(*dest_cloud);
  }

  void map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    ROS_INFO("Got map message");
    pcl::fromROSMsg(*msg, *map_points);
    map_ready = true;
  }
  
  void pc_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    ROS_INFO("Got lidar message");
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Matrix4f result;

    while(!(gps_ready & map_ready)){
      ROS_WARN("waiting for map and gps data ...");
      ros::Duration(0.05).sleep();
      ros::spinOnce();
    }

    pcl::fromROSMsg(*msg, *scan_ptr);
    ROS_INFO("point size: %d", scan_ptr->width);
    result = align_map(scan_ptr); // after ICP result

    // publish transformed points
    sensor_msgs::PointCloud2::Ptr out_msg(new sensor_msgs::PointCloud2);
    pcl_ros::transformPointCloud(result, *msg, *out_msg);
    out_msg->header = msg->header;
    out_msg->header.frame_id = mapFrame;
    pub_points.publish(out_msg);

    // broadcast transforms
    tf::Matrix3x3 rot;
    rot.setValue(
      static_cast<double>(result(0, 0)), static_cast<double>(result(0, 1)), static_cast<double>(result(0, 2)), 
      static_cast<double>(result(1, 0)), static_cast<double>(result(1, 1)), static_cast<double>(result(1, 2)),
      static_cast<double>(result(2, 0)), static_cast<double>(result(2, 1)), static_cast<double>(result(2, 2))
    );
    tf::Vector3 trans(result(0, 3), result(1, 3), result(2, 3));
    tf::Transform transform(rot, trans);
    // br.sendTransform(tf::StampedTransform(transform.inverse(), msg->header.stamp, lidarFrame, mapFrame));


    // publish lidar pose
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.header.frame_id = mapFrame;
    pose.pose.position.x = trans.getX();
    pose.pose.position.y = trans.getY();
    pose.pose.position.z = trans.getZ();
    pose.pose.orientation.x = transform.getRotation().getX();
    pose.pose.orientation.y = transform.getRotation().getY();
    pose.pose.orientation.z = transform.getRotation().getZ();
    pose.pose.orientation.w = transform.getRotation().getW();
    pub_pose.publish(pose);  // publish lidar pose

    Eigen::Affine3d transform_c2l, transform_m2l;
    transform_m2l.matrix() = result.cast<double>();
    transform_c2l = (tf2::transformToEigen(car2Lidar));
    Eigen::Affine3d tf_p = transform_m2l * transform_c2l.inverse();
    geometry_msgs::TransformStamped transform_m2c = tf2::eigenToTransform(tf_p);
    

    tf::Quaternion q(transform_m2c.transform.rotation.x, transform_m2c.transform.rotation.y, transform_m2c.transform.rotation.z, transform_m2c.transform.rotation.w);
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);
    outfile << ++cnt << "," << tf_p.translation().x() << "," << tf_p.translation().y() << "," << 0 << "," << yaw << "," << pitch << "," << roll << std::endl;
    // This expresses an estimated (car)pose with a reference coordinate frame and timestamp

    // publish car pose
    geometry_msgs::PoseWithCovarianceStamped pose_car;
    pose_car.header = msg->header;
    pose_car.header.frame_id = mapFrame;  // this map is world frame
    pose_car.pose.pose.position.x = tf_p.translation().x();
    pose_car.pose.pose.position.y = tf_p.translation().y();
    pose_car.pose.pose.position.z = tf_p.translation().z();
    pose_car.pose.pose.orientation.x = transform_m2c.transform.rotation.x;  // orientation ~ rotation
    pose_car.pose.pose.orientation.y = transform_m2c.transform.rotation.y;
    pose_car.pose.pose.orientation.z = transform_m2c.transform.rotation.z;
    pose_car.pose.pose.orientation.w = transform_m2c.transform.rotation.w;
    pose_car.pose.covariance = {1, 0, 0, 0, 0, 0,
                                0, 1, 0, 0, 0, 0,
                                0, 0, 1, 0, 0, 0,
                                0, 0, 0, 0.05, 0, 0,
                                0, 0, 0, 0, 0.05, 0,
                                0, 0, 0, 0, 0, 0.05};
    pub_car_pose.publish(pose_car);  // publish car pose
    std::cout << "Car_Pose" << std::endl;
    std::cout << pose_car.pose.pose.position << std::endl;

  }

  void gps_callback(const geometry_msgs::PointStamped::ConstPtr& msg){
    ROS_INFO("Got GPS message");
    gps_point.x = msg->point.x;
    gps_point.y = msg->point.y;
    gps_point.z = msg->point.z;

    if(!initialized){
    // if(true){
      geometry_msgs::PoseStamped pose;
      pose.header = msg->header;
      pose.pose.position = msg->point;
      pub_pose.publish(pose);
      // ROS_INFO("pub pose");

      tf::Matrix3x3 rot;
      rot.setIdentity();
      tf::Vector3 trans(msg->point.x, msg->point.y, msg->point.z);
      tf::Transform transform(rot, trans);
      // Is this world->lidar ?
      // br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "world", "nuscenes_lidar"));
    }

    gps_ready = true;
    return;
  }
// navsat_transform_node - Allows users to easily transform geographic coordinates into the robot's world frame(typically map or odometry)
  void ekf_callback(const nav_msgs::Odometry::ConstPtr& msg){
    // nav_msgs::Odometry to Eigen::Matrix4f
    Eigen::Matrix4f EKFeigen;

    Eigen::Quaterniond quat;
    quat.w() = msg->pose.pose.orientation.w;
    quat.x() = msg->pose.pose.orientation.x;
    quat.y() = msg->pose.pose.orientation.y;
    quat.z() = msg->pose.pose.orientation.z;

    Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
    isometry.linear() = quat.toRotationMatrix();
    isometry.translation() = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    EKFeigen = isometry.matrix().cast<float>();
    std::cout << "EKFeigen" << std::endl;
    std::cout << msg->pose.pose.position << std::endl;

    Eigen::Affine3f transform_c2l;
    transform_c2l = (tf2::transformToEigen(car2Lidar)).cast<float>();

    Eigen::Matrix4f transform_c2l_4f = transform_c2l.matrix(); // Affine3f to Matrix4f

    Eigen::Matrix4f EKFmatrix4f = EKFeigen * transform_c2l_4f; // (Matrix4f) * (Matrix4f)
  
    init_guess = EKFmatrix4f;
    return;
  }

  Eigen::Matrix4f align_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr scan_points){
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Matrix4f result;

  /* [Part 1] Perform pointcloud preprocessing here e.g. downsampling use setLeafSize(...) ... */
    down_sampling(scan_points, filtered_scan_ptr);
    down_sampling(map_points, filtered_map_ptr);
    
    // find the initial orientation
    if (!initialized) {
    /* [Part 3] you can perform ICP several times to find a good initial guess */
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> first_icp;
        float yaw, min_yaw, min_score = std::numeric_limits<float>::max();
        Eigen::Matrix4f min_pose(Eigen::Matrix4f::Identity());

        for (yaw = 0; yaw < (M_PI * 2); yaw += 0.6) {
          Eigen::Translation3f init_translation(gps_point.x, gps_point.y, gps_point.z);
          Eigen::AngleAxisf init_rotation_z(yaw, Eigen::Vector3f::UnitZ());
          init_guess = (init_translation * init_rotation_z).matrix();

          //Do ICP here
          first_icp.setInputSource (filtered_scan_ptr);
          first_icp.setInputTarget (filtered_map_ptr);
          
          first_icp.setMaxCorrespondenceDistance (3);
          first_icp.setMaximumIterations (100);
          first_icp.setTransformationEpsilon (1e-8);
          first_icp.setEuclideanFitnessEpsilon (1e-8);

          first_icp.align (*transformed_scan_ptr, init_guess);
          
          result = first_icp.getFinalTransformation ();
          double score = first_icp.getFitnessScore(0.5);
          if (score < min_score) {
              min_score = score;
              min_pose = first_icp.getFinalTransformation();
              ROS_INFO("Update best pose");
          }

        // The output of the ekf node can be used as a good initial guess for ICP.
        // set initial guess
        }
        init_guess = min_pose;
        initialized = true;
    }
      
    /* [Part 2] Perform ICP here or any other scan-matching algorithm */
    /* Refer to https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point.html#details */
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    // Set the input source and target
    icp.setInputSource (filtered_scan_ptr);
    icp.setInputTarget (filtered_map_ptr);
    
    // Set the max correspondence distance to 50cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (3);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (80);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1e-7);
    
    // Perform the alignment
    icp.align (*transformed_scan_ptr, init_guess);
    
    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    result = icp.getFinalTransformation ();

    
	/* Use result as next initial guess */
    //init_guess = result;
    std::cout << "Result" << std::endl;
    std::cout << result << std::endl;
    return result;
  }
};


int main(int argc, char* argv[]){
  ros::init(argc, argv, "localizer");
  ros::NodeHandle n("~");
  Localizer localizer(n);
  ros::spin();
  return 0;
}

// first argument of PYBIND11_MODULE must be as the same as '.so' file you would compile later
PYBIND11_MODULE(_localization, l)
{
    // generate _localization.so
    l.doc() = "fusion IMU, wheel_odom, gps";      // module doc string
    l.def("multiply_naive", &multiply_naive);   //
    l.def("multiply_mkl", &multiply_mkl);       //
    l.def("multiply_tile", &multiply_tile);     //

    pybind11::class_<Localizer>(l, "Localizer")       // 
        .def(py::init<>())
        .def("down_sampling", static_cast<void (Localizer::*)(pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr dest_cloud)>(&Localizer::down_sampling))
        .def("map_callback", static_cast<void (Localizer::*)(const sensor_msgs::PointCloud2::ConstPtr& msg)>(&Localizer::map_callback))
        .def("pc_callback", static_cast<void (Localizer::*)(const sensor_msgs::PointCloud2::ConstPtr& msg)>(&Localizer::pc_callback))
        .def("gps_callback", static_cast<void (Localizer::*)(const geometry_msgs::PointStamped::ConstPtr& msg)>(&Localizer::gps_callback))
        .def("ekf_callback", static_cast<void (Localizer::*)(const nav_msgs::Odometry::ConstPtr& msg)>(&Localizer::ekf_callback))
        
        // .def("train", &Regression::train)
        
        // .def_property_readonly("weight", &Regression::get_weight);
}
