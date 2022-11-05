#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <random>
#include <vector>

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

// #define distanceGraph vector<pair<octomap::OcTreeKey, double>> 
// #define distanceGraph vector<pair<int, double>> 

using namespace std;

class octomap_only{
private:
	ros::Publisher octo_occu_pub;

	ros::Subscriber sub_state;
	ros::Subscriber sub_sensor;
	ros::Subscriber sub_pose;

	ros::Timer octo_input_timer;

	geometry_msgs::PoseStamped curr_pose;
	sensor_msgs::PointCloud2 sensor_data;

	Eigen::Matrix4f map_t_body = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f body_t_sensor = Eigen::Matrix4f::Identity();

	double sensor_range;
	double octomap_resolution;
	double octomap_hit;
	double octomap_miss;
	double octomap_hz;

	sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud, string frame_id = "camera_link");
	pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg);
	octomap::Pointcloud cloudmsg2octocloud(sensor_msgs::PointCloud2 cloudmsg);
	octomap::Pointcloud cloud2octocloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn);
	octomap::Pointcloud cloud2transOctoCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, Eigen::Matrix4f map_t_sensor);

	octomath::Pose6D cvt2octomath(nav_msgs::Odometry curr_pose);
	octomath::Pose6D cvt2octomath(geometry_msgs::PoseStamped curr_pose);

	bool data_input(sensor_msgs::PointCloud2 input);
	// bool check_free(geometry_msgs::PoseStamped point, geometry_msgs::PoseStamped end, octomap::point3d& hit_point);
    // bool check_free(geometry_msgs::PoseStamped point, int depth=0);
    bool check_free(octomap::point3d check, int depth=0);
	
	void subs_sensor(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void subs_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void cvt_octo2pubpc();

	float normalize(float a, float b);
	float normalize(float a, float b, float c);
	float normalize(float a, float b, float c, float d);
	double normalize(double a, double b);
	double normalize(double a, double b, double c);
	double normalize(double a, double b, double c, double d);

public:
	ros::NodeHandle nh;

    shared_ptr<octomap::OcTree> m_octree = nullptr;

	void input_timer(const ros::TimerEvent& event);

	octomap_only(ros::NodeHandle& n) : nh(n)
	{
        body_t_sensor(0,3) = 0.08;
        body_t_sensor(1,3) = 0;
        body_t_sensor(2,3) = 0.1;

		nh.param("/sensor_range", sensor_range, 30.0);
		nh.param("/octomap_resolution", octomap_resolution, 0.5);
		nh.param("/octomap_hit", octomap_hit, 0.7);
		nh.param("/octomap_miss", octomap_miss, 0.1);
		nh.param("/octomap_hz", octomap_hz, 10.0);

        m_octree = make_shared<octomap::OcTree>(octomap_resolution);
		m_octree -> setProbHit(octomap_hit);
	 	m_octree -> setProbMiss(octomap_miss);
        m_octree -> setClampingThresMax(0.97);
        m_octree -> setClampingThresMin(0.12);
        m_octree -> setOccupancyThres(0.5);
	 	m_octree -> enableChangeDetection(true);

		octo_occu_pub    = nh.advertise<sensor_msgs::PointCloud2>("/srsd/occu", 10);
        
		sub_sensor = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &octomap_only::subs_sensor,this, ros::TransportHints().tcpNoDelay());
		sub_pose  = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &octomap_only::subs_pose,this, ros::TransportHints().tcpNoDelay());

		octo_input_timer    = nh.createTimer(ros::Duration(1/octomap_hz), &octomap_only::input_timer, this); // every hz
	};
};


		