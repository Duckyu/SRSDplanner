#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <math.h>
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
#include <std_msgs/Duration.h>
#include <std_msgs/Empty.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//ros srv
#include <mavros_msgs/SetMode.h>

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

class srsd_planner{
private:
    ///position target control type arranged
    int velocity_control_type = mavros_msgs::PositionTarget::IGNORE_AFX|
                                mavros_msgs::PositionTarget::IGNORE_AFY|
                                mavros_msgs::PositionTarget::IGNORE_AFZ|
                                mavros_msgs::PositionTarget::IGNORE_PX|
                                mavros_msgs::PositionTarget::IGNORE_PY|
                                mavros_msgs::PositionTarget::IGNORE_PZ|
                                mavros_msgs::PositionTarget::IGNORE_YAW;

    int position_control_type = mavros_msgs::PositionTarget::IGNORE_AFX|
                                mavros_msgs::PositionTarget::IGNORE_AFY|
                                mavros_msgs::PositionTarget::IGNORE_AFZ|
                                mavros_msgs::PositionTarget::IGNORE_VX|
                                mavros_msgs::PositionTarget::IGNORE_VY|
                                mavros_msgs::PositionTarget::IGNORE_VZ|
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
                                
    bool require_target;
    bool armed_flag;
    bool offb_flag;
    bool hover_flag;
    bool start_flag;
    bool end_flag;
    
    double sensor_hor_range;
    double sensor_ver_range;
    double sensor_reliable_range;
    double sensor_angle;
    double update_tolerance;
    double robot_size;
    double size_depth;

    double map_min_x;
	double map_min_y;
	double map_min_z;
	double map_max_x;
	double map_max_y;
	double map_max_z;
    geometry_msgs::Point map_min;
	geometry_msgs::Point map_max;

	ros::Publisher octo_occu_pub;
    ros::Publisher octo_free_pub;
    ros::Publisher setpoint_raw_pub;
    ros::Publisher astar_pub;
    ros::Publisher map_bound_pub;
    ros::Publisher abort_pub;
    ros::Publisher check_pub;

    ros::Subscriber sub_start;
	ros::Subscriber sub_state;
	ros::Subscriber sub_sensor;
	ros::Subscriber sub_pose;

    ros::ServiceClient set_mode_client;

	ros::Timer octo_input_timer;
	ros::Timer srsd_planner_timer;
	ros::Timer srsd_control_timer;

	geometry_msgs::PoseStamped curr_pose;
	sensor_msgs::PointCloud2 sensor_data;


	geometry_msgs::Point target;
    mavros_msgs::PositionTarget control_msg;

	Eigen::Matrix4f map_t_body = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f body_t_sensor = Eigen::Matrix4f::Identity();

	double sensor_range;
	double octomap_resolution;
	double octomap_hit;
	double octomap_miss;
	double octomap_hz;

    double unit;

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
    bool opti_check_free(octomap::point3d check, int depth=0);
	bool pessi_check_free(octomap::point3d check, int depth=0);
	
    void subs_start(const std_msgs::Empty::ConstPtr& msg);
	void subs_sensor(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void subs_state(const mavros_msgs::State::ConstPtr& msg);
	void subs_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void cvt_octo2pubpc();

	float normalize(float a, float b);
	float normalize(float a, float b, float c);
	float normalize(float a, float b, float c, float d);
	double normalize(double a, double b);
	double normalize(double a, double b, double c);
	double normalize(double a, double b, double c, double d);

    bool arrival_check();
    bool astar(octomap::point3d start, octomap::point3d end, nav_msgs::Path& path);

public:
	ros::NodeHandle nh;

    ros::Time mission_start;
    ros::Duration mission_period;

    shared_ptr<octomap::OcTree> m_octree = nullptr;

    geometry_msgs::Point hover_point;
    geometry_msgs::Point start_point;

    // bool x_priority;
    vector<octomap::point3d> pre_ban_list;
    vector<octomap::point3d> final_ban_list;
    int path_idx = 0;
    nav_msgs::Path calculated_path;
    int now[3] = {0,0,0};
    int prev[3] = {0,0,0};

    ros::Time offb_request;

    // bool check_x_priority();

	void input_timer(const ros::TimerEvent& event);
	void planner_timer(const ros::TimerEvent& event);
	void control_timer(const ros::TimerEvent& event);

    map<int, octomap::OcTreeKey> closed_key_list;
    map<int, octomap::OcTreeKey> opened_key_list;
    map<int, int> child_parent; 
	map<int, double> closed_g; // key, G distance
    map<int, double> closed_h; // key, H distance
    map<int, double> closed_f; // key, F distance
    map<int, double> opened_g; // key, distance
    map<int, double> opened_h; // key, distance
    map<int, double> opened_f; // key, distance

	srsd_planner(ros::NodeHandle& n) : nh(n)
	{
        offb_request = ros::Time::now();
        require_target = false;
        armed_flag = false;
        offb_flag = false;
        hover_flag = true;
        start_flag     = false;
        end_flag       = false;

        body_t_sensor(0,3) = 0.08;
        body_t_sensor(1,3) = 0;
        body_t_sensor(2,3) = 0.1;

		nh.param("/sensor_range", sensor_range, 30.0);
		nh.param("/octomap_resolution", octomap_resolution, 0.3);
		nh.param("/octomap_hit", octomap_hit, 0.9);
		nh.param("/octomap_miss", octomap_miss, 0.4);
		nh.param("/octomap_hz", octomap_hz, 10.0);

		nh.param("/robot_size", robot_size, 0.5);
		nh.param("/sensor_reliable_range", sensor_reliable_range, 12.0);
        nh.param("/sensor_angle", sensor_angle, 0.523599);
        nh.param("/update_tolerance", update_tolerance, 0.02);

        sensor_hor_range = sensor_reliable_range;
        sensor_ver_range = 2.0 * sensor_reliable_range * sin(sensor_angle/2.0);
        ROS_INFO("sensor_ver_range: %lf",sensor_ver_range);

        start_point.x = curr_pose.pose.position.x;
        start_point.y = curr_pose.pose.position.y;
        start_point.z = sensor_ver_range/2.0;

        size_depth = floor(log2(robot_size*2.0/octomap_resolution));
        unit = octomap_resolution*pow(2, (int)size_depth);
        // unit = octomap_resolution;

        hover_point = start_point;
        target = hover_point;

        calculated_path.header.frame_id = "map";

        control_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        control_msg.type_mask = position_control_type;
        control_msg.position = hover_point;

		nh.param("/map_min_x", map_min_x, -58.0);
		nh.param("/map_min_y", map_min_y, -17.0);
		nh.param("/map_min_z", map_min_z, 0.0);
		nh.param("/map_max_x", map_max_x, 32.0);
		nh.param("/map_max_y", map_max_y, 53.0);
		nh.param("/map_max_z", map_max_z, 20.0);
        //start point 10 -10 0


        map_min.x = map_min_x;
        map_min.y = map_min_y;
        map_min.z = map_min_z;
        map_max.x = map_max_x;
        map_max.y = map_max_y;
        map_max.z = map_max_z;

        m_octree = make_shared<octomap::OcTree>(octomap_resolution);
		m_octree -> setProbHit(octomap_hit);
	 	m_octree -> setProbMiss(octomap_miss);
        m_octree -> setClampingThresMax(0.97);
        m_octree -> setClampingThresMin(0.12);
        m_octree -> setOccupancyThres(0.5);
	 	m_octree -> enableChangeDetection(true);

		octo_occu_pub    = nh.advertise<sensor_msgs::PointCloud2>("/occu", 10);
		octo_free_pub    = nh.advertise<sensor_msgs::PointCloud2>("/free", 10);
        setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        astar_pub        = nh.advertise<nav_msgs::Path>("/astar_path", 10);
        map_bound_pub    = nh.advertise<visualization_msgs::Marker>("/map_bound", 10);
        abort_pub    = nh.advertise<visualization_msgs::Marker>("/abort", 10);
        check_pub        = nh.advertise<visualization_msgs::Marker>("/check", 10);
        
        sub_start = nh.subscribe<std_msgs::Empty>("/start", 1, &srsd_planner::subs_start, this);
		sub_sensor = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &srsd_planner::subs_sensor,this, ros::TransportHints().tcpNoDelay());
        sub_state = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, &srsd_planner::subs_state, this);
		sub_pose  = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &srsd_planner::subs_pose,this, ros::TransportHints().tcpNoDelay());

        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

		octo_input_timer    = nh.createTimer(ros::Duration(1/octomap_hz), &srsd_planner::input_timer, this); // every hz
		srsd_planner_timer    = nh.createTimer(ros::Duration(1/60.0), &srsd_planner::planner_timer, this); // every hz
	    srsd_control_timer    = nh.createTimer(ros::Duration(1/60.0), &srsd_planner::control_timer, this); // every hz
        
        // x_priority = check_x_priority();
        octomap::point3d start_ban(0,0,0);
        pre_ban_list.push_back(start_ban);
	};
};


		