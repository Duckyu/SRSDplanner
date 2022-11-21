#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <algorithm>
#include <random>
#include <vector>
#include <string>

//transformation
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//ros msg
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//pcl 
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//octomap lib
#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>
#include <octomap/ScanGraph.h>
#include <octomap/math/Vector3.h>

//opencv
#include <opencv2/core.hpp>

using namespace std;

class octomap_only{
private:
	ros::Publisher octo_occu_pub;
    ros::Publisher octo_free_pub;
    ros::Publisher free_ray_pub;

	ros::Subscriber sub_state;
	ros::Subscriber sub_sensor;
	ros::Subscriber sub_pose;
	ros::Subscriber sub_odom;

	ros::Timer octo_input_timer;

	geometry_msgs::PoseStamped curr_pose;
	sensor_msgs::PointCloud2 sensor_data;

	Eigen::Matrix4f map_t_body = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f body_t_sensor = Eigen::Matrix4f::Identity();

	bool use_free_ray;
	int sensor_type; // 0: LiDAR, 1: RGBD, 2: 
	int robot_type; // 0: Pixhawk, 1: DJI, 
	string lidar_topic;
	string pose_topic;

	//LiDAR
	int sensor_ring;
	int sensor_samples;
	double sensor_vert_min;
	double sensor_vert_max;
	double sensor_hori_min;
	double sensor_hori_max;
	double min_sensor_range;
	double max_sensor_range;
    vector<double> ring_angle_map;
	vector<int> free_ray_ring;

	bool ring_angle_map_exist;
	bool free_ray_ring_exist;
	// Eigen::MatrixXd check_reference;	

	double update_sensor_range;
	double octomap_resolution;
	double octomap_hit;
	double octomap_miss;
	double octomap_thresh;
	double octomap_hz;
	double update_tolerance;
	bool input_log;

	double map_min_x;
	double map_min_y;
	double map_min_z;
	double map_max_x;
	double map_max_y;
	double map_max_z;

	double no_input_cnt = 0;

	sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud, string frame_id = "camera_link");
	pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg);
	octomap::Pointcloud cloudmsg2octocloud(sensor_msgs::PointCloud2 cloudmsg);
	octomap::Pointcloud cloud2octocloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn);
	octomap::Pointcloud cloud2transOctoCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, Eigen::Matrix4f map_t_sensor);

	// octomath::Pose6D cvt2octomath(nav_msgs::Odometry curr_odom);
	// octomath::Pose6D cvt2octomath(geometry_msgs::PoseStamped curr_pose);

	bool data_input(sensor_msgs::PointCloud2 input);
	// bool check_free(geometry_msgs::PoseStamped point, geometry_msgs::PoseStamped end, octomap::point3d& hit_point);
    // bool check_free(geometry_msgs::PoseStamped point, int depth=0);
    bool opti_check_free(octomap::point3d check, int depth=0);
    bool pessi_check_free(octomap::point3d check, int depth=0);
	
	void subs_sensor(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void subs_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void subs_odom(const nav_msgs::Odometry::ConstPtr& msg);
	void cvt_octo2pubpc();

	// float normalize(float a, float b);
	// float normalize(float a, float b, float c);
	// float normalize(float a, float b, float c, float d);
	// double normalize(double a, double b);
	// double normalize(double a, double b, double c);
	// double normalize(double a, double b, double c, double d);

public:
	ros::NodeHandle nh;

    shared_ptr<octomap::OcTree> m_octree = nullptr;

	void input_timer(const ros::TimerEvent& event);

	octomap_only(ros::NodeHandle& n) : nh(n)
	{
        body_t_sensor(0,3) = 0.08;
        body_t_sensor(1,3) = 0;
        body_t_sensor(2,3) = 0.1;


		//Usage options
        nh.param("/use_free_ray",        use_free_ray,        true); // For planner
        nh.param("/sensor_type",         sensor_type,         0); // 0: LiDAR, rest of them doesn;t implemented yet
        nh.param("/robot_type",          robot_type,          0); // 0: LiDAR, rest of them doesn;t implemented yet
        

		nh.param<string>("/lidar_topic",         lidar_topic,         "/velodyne_points"); // topic name 
		nh.param<string>("/pose_topic",           pose_topic,         "/mavros/local_position/pose"); // topic name 
		
		//Ouster OS0-32 configuration w/ beam spacing option: uniform
        if(sensor_type == 0) //LiDAR
		{
			nh.param("/sensor_ring",         sensor_ring,         32);
        	nh.param("/sensor_samples",      sensor_samples,      2048);
        	nh.param("/sensor_vert_min",     sensor_vert_min,     -0.785398);
        	nh.param("/sensor_vert_max",     sensor_vert_max,     0.785398);
        	nh.param("/sensor_hori_min",     sensor_hori_min,     -3.141592);
        	nh.param("/sensor_hori_max",     sensor_hori_max,     3.141592);
			nh.param("/min_sensor_range",    max_sensor_range,    0.5);
			nh.param("/max_sensor_range",    max_sensor_range,    100.0);

			// check_reference.resize(sensor_ring, sensor_samples);

			ring_angle_map_exist = nh.hasParam("/ring_angle_map");
			if(ring_angle_map_exist){
      			nh.getParam("/ring_angle_map", ring_angle_map);
				if(ring_angle_map.size() != sensor_ring){
					ring_angle_map_exist = false;
				}
			}
			free_ray_ring_exist = nh.hasParam("/free_ray_ring");
			if(free_ray_ring_exist){
      			nh.getParam("/free_ray_ring", free_ray_ring);
				if(free_ray_ring.size() != sensor_ring){
					free_ray_ring_exist = false;
				}
			}
			else{
				for(int i = 0; i< sensor_ring; i++){
					free_ray_ring.push_back(i);
				}				
			}
		}

		//Octomap
		nh.param("/update_sensor_range", update_sensor_range, 35.0);
		nh.param("/octomap_resolution",  octomap_resolution,  0.5);
		nh.param("/octomap_hit",         octomap_hit,         0.7);
		nh.param("/octomap_miss",        octomap_miss,        0.4);
		nh.param("/octomap_thresh",      octomap_thresh,      0.5);
		nh.param("/octomap_hz",          octomap_hz,          10.0);
        nh.param("/update_tolerance",    update_tolerance,    0.02);
        nh.param("/input_log",           input_log,           true);

		//Octomap BBX
		nh.param("/map_min_x",           map_min_x, -58.0);
		nh.param("/map_min_y",           map_min_y, -17.0);
		nh.param("/map_min_z",           map_min_z, 0.0);
		nh.param("/map_max_x",           map_max_x, 32.0);
		nh.param("/map_max_y",           map_max_y, 53.0);
		nh.param("/map_max_z",           map_max_z, 20.0);
		
		octomap::point3d map_min;
		octomap::point3d map_max;

		map_min.x() = map_min_x;
		map_min.y() = map_min_y;
		map_min.z() = map_min_z;
		map_max.x() = map_max_x;
		map_max.y() = map_max_y;
		map_max.z() = map_max_z;

        m_octree = make_shared<octomap::OcTree>(octomap_resolution);
		m_octree -> setProbHit(octomap_hit);
	 	m_octree -> setProbMiss(octomap_miss);
        m_octree -> setClampingThresMax(0.97);
        m_octree -> setClampingThresMin(0.03);
        m_octree -> setOccupancyThres(0.5);
	 	// m_octree -> enableChangeDetection(true);
	 	m_octree -> useBBXLimit(true);
	 	m_octree -> setBBXMin(map_min);
	 	m_octree -> setBBXMax(map_max);
  		unsigned int full_depth = m_octree->getTreeDepth();
		cout << "full_depth: " << full_depth << endl;

		octo_occu_pub    = nh.advertise<sensor_msgs::PointCloud2>("/occu", 10);
        octo_free_pub    = nh.advertise<sensor_msgs::PointCloud2>("/free", 10);
		free_ray_pub     = nh.advertise<sensor_msgs::PointCloud2>("/free_ray", 10);
		
		sub_sensor = nh.subscribe<sensor_msgs::PointCloud2>((string)lidar_topic, 1, &octomap_only::subs_sensor,this, ros::TransportHints().tcpNoDelay());
		if(robot_type == 0){
			sub_pose  = nh.subscribe<geometry_msgs::PoseStamped>((string)pose_topic, 1, &octomap_only::subs_pose,this, ros::TransportHints().tcpNoDelay());
		}
		else if(robot_type == 1){
			sub_odom  = nh.subscribe<nav_msgs::Odometry>((string)pose_topic, 1, &octomap_only::subs_odom,this, ros::TransportHints().tcpNoDelay());
		}

		// sub_odom  = nh.subscribe<nav_msgs::Odometry>((string)pose_topic, 1, &octomap_only::subs_odom,this, ros::TransportHints().tcpNoDelay());

		octo_input_timer    = nh.createTimer(ros::Duration(1/octomap_hz), &octomap_only::input_timer, this); // every hz
	};
};


		