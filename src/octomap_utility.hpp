#define FRONT 0
#define UP 1
#define RIGHT 2
//sensor_direction

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <math.h>

//transformation
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//ros msg
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

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

//additional
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>

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

//custom service
#include "n_cpp/get_octomap.h"
#include "n_cpp/get_triple_points.h"
#include "n_cpp/collision_check.h"
#include "n_cpp/RayCast.h"
#include "n_cpp/EliminateTriple.h"
// #include "n_cpp/ThreeDAstar.h"
// #include "n_cpp/FindCollisionAvoidPath.h"

class srsd_octomap
{
private:
	bool check_free(geometry_msgs::PoseStamped point, geometry_msgs::PoseStamped end);

	ros::Publisher octo_occu_pub;

	ros::Subscriber subs_sensor;
	ros::Subscriber subs_pose;

	ros::Timer octo_input_timer;
	ros::Timer control_timer;

	geometry_msgs::PoseStamped curr_pose;
	sensor_msgs::PointCloud2 sensor_data;

	Eigen::Matrix4f map_t_body = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f body_t_sensor = Eigen::Matrix4f::Identity();
	body_t_sensor(0,3) = 0.08;
	body_t_sensor(1,3) = 0;
	body_t_sensor(2,3) = 0.1;
	
	double sensor_range;
	double octomap_resolution;
	double octomap_hit;
	double octomap_miss;
	double octomap_hz;

	octomap::OcTree *m_octree;

	sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud, std::string frame_id = "camera_link");
	pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg);
	octomap::Pointcloud cloudmsg2octocloud(sensor_msgs::PointCloud2 cloudmsg);
	octomap::Pointcloud cloud2octocloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn);
	octomap::Pointcloud cloud2transOctoCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, Eigen::Matrix4f map_t_sensor);

	octomath::Pose6D cvt2octomath(nav_msgs::Odometry curr_pose);
	octomath::Pose6D cvt2octomath(geometry_msgs::PoseStamped curr_pose);

	bool data_input(octomap::OcTree *octree, sensor_msgs::PointCloud2 input, int sensor_direction);
	
	void subs_sensor(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void subs_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
	// void cb_tf(const tf2_msgs::TFMessage::ConstPtr& msg);
	void cb_st_tf(const tf2_msgs::TFMessage::ConstPtr& msg);

	void cvt_octo2pubpc(octomap::OcTree *octree);

	float normalize(float a, float b);
	float normalize(float a, float b, float c);
	float normalize(float a, float b, float c, float d);
	double normalize(double a, double b);
	double normalize(double a, double b, double c);
	double normalize(double a, double b, double c, double d);

public:
	ros::NodeHandle nh;

	void input_timer(const ros::TimerEvent& event);
	
	srsd_octomap(ros::NodeHandle& n) : nh(n)
	{
		nh.param("/sensor_range", sensor_range, 18.0);
		nh.param("/octomap_resolution", octomap_resolution, 0.1);
		nh.param("/octomap_hit", octomap_hit, 0.65);
		nh.param("/octomap_miss", octomap_miss, 0.3);
		nh.param("/octomap_hz", octomap_hz, 6.0);

		m_octree = new octomap::OcTree(octomap_resolution);
		m_octree -> setProbHit(octomap_hit);
	 	m_octree -> setProbMiss(octomap_miss);
	 	m_octree -> enableChangeDetection(true);

		octo_occu_pub    = nh.advertise<sensor_msgs::PointCloud2>("/N_octo/occu", 10);
		octo_free_pub    = nh.advertise<sensor_msgs::PointCloud2>("/N_octo/free", 10);
		octo_change_pub  = nh.advertise<sensor_msgs::PointCloud2>("/N_octo/new", 10);
		tri_points_pub  = nh.advertise<sensor_msgs::PointCloud2>("/N_octo/tri_points", 10);

		sub_sensor = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &srsd_octomap::subs_sensor,this, ros::TransportHints().tcpNoDelay());
		sub_pose         = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &srsd_octomap::subs_pose,this, ros::TransportHints().tcpNoDelay());

    	srv_get_octomap                = nh.advertiseService("get_octomap", &srsd_octomap::get_changed_octomap, this);

		octo_input_timer    = nh.createTimer(ros::Duration(1/octomap_hz), &srsd_octomap::input_timer, this); // every hz
		control_timer    = nh.createTimer(ros::Duration(1/octomap_hz), &srsd_octomap::control_timer, this); // every hz
	};
};