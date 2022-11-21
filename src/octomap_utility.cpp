#include "exploration.hpp"

bool srsd_planner::data_input(sensor_msgs::PointCloud2 input)
{
	bool result = true;

	////////////If cb_tf exists, kill below code //////////////
	Eigen::Quaternionf q(curr_pose.pose.orientation.w,
						 curr_pose.pose.orientation.x,
						 curr_pose.pose.orientation.y,
						 curr_pose.pose.orientation.z);
	map_t_body.block(0,0,3,3) = q.normalized().toRotationMatrix();
	map_t_body(0,3) = curr_pose.pose.position.x;
	map_t_body(1,3) = curr_pose.pose.position.y;
	map_t_body(2,3) = curr_pose.pose.position.z;
	//////////////////////////////////////////////////////////
	Eigen::Matrix4f map_t_sensor;
	map_t_sensor = map_t_body * body_t_sensor;

	double timeoffset = fabs(curr_pose.header.stamp.toSec() - input.header.stamp.toSec());

	if (input.point_step>0 && timeoffset < update_tolerance){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>());
		*cloudIn = cloudmsg2cloud(input);
		octomap::Pointcloud transOcto = cloud2transOctoCloud(cloudIn, map_t_sensor);
		// std::cout << "data_input size: " << cloudIn->size()<< std::endl;
		m_octree->insertPointCloud(transOcto, octomap::point3d(map_t_sensor(0,3),map_t_sensor(1,3),map_t_sensor(2,3)), sensor_range);//
		cloudIn->clear();
		transOcto.clear();
		// cloudIn.reset(new pcl::PointCloud<pcl::PointXYZ>());
	}
	else{
		std::cout << "no input!!!" << std::endl;
		result = false;
	}
	
	return result;
}

void srsd_planner::input_timer(const ros::TimerEvent& event)
{
	data_input(sensor_data);
	cvt_octo2pubpc();

	// std::cout << "m_octree: " << m_octree->size() << std::endl;
}


	
sensor_msgs::PointCloud2 srsd_planner::cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud, std::string frame_id) // = "camera_link"
{
	sensor_msgs::PointCloud2 cloud_ROS;
	pcl::toROSMsg(cloud, cloud_ROS);
	cloud_ROS.header.frame_id = frame_id;
	return cloud_ROS;
}

pcl::PointCloud<pcl::PointXYZ> srsd_planner::cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudresult(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(cloudmsg, *cloudresult);

	// cloudresult.reset(new pcl::PointCloud<pcl::PointXYZ>());
	return *cloudresult;
}

octomap::Pointcloud srsd_planner::cloudmsg2octocloud(sensor_msgs::PointCloud2 cloudmsg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZ>());
	*pcl_pc = cloudmsg2cloud(cloudmsg);
	
	octomap::Pointcloud octopc_result;
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pcl_pc->begin(); it!=pcl_pc->end(); ++it){
	  octopc_result.push_back(it->x, it->y, it->z);
	}	
	pcl_pc->clear();
	// pcl_pc.reset(new pcl::PointCloud<pcl::PointXYZ>());
	return octopc_result;
}

octomap::Pointcloud srsd_planner::cloud2octocloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn)
{
	int cloudSize = cloudIn->size();
	octomap::Pointcloud octopc_result;

	#pragma omp parallel for num_threads(numberOfCores)
	for (int i = 0; i < cloudSize; ++i){
	  	octopc_result.push_back(cloudIn->points[i].x, cloudIn->points[i].y, cloudIn->points[i].z);
	}
	return octopc_result;
}

octomap::Pointcloud srsd_planner::cloud2transOctoCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, Eigen::Matrix4f map_t_sensor)
{
	octomap::Pointcloud octopc_result;
	int cloudSize = cloudIn->size();

	#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
    	const auto &pointFrom = cloudIn->points[i];
    	#pragma omp critical
    	octopc_result.push_back(map_t_sensor(0,0) * pointFrom.x + map_t_sensor(0,1) * pointFrom.y + map_t_sensor(0,2) * pointFrom.z + map_t_sensor(0,3),
    							map_t_sensor(1,0) * pointFrom.x + map_t_sensor(1,1) * pointFrom.y + map_t_sensor(1,2) * pointFrom.z + map_t_sensor(1,3),
    							map_t_sensor(2,0) * pointFrom.x + map_t_sensor(2,1) * pointFrom.y + map_t_sensor(2,2) * pointFrom.z + map_t_sensor(2,3));
    }

	return octopc_result;
}


octomath::Pose6D srsd_planner::cvt2octomath(nav_msgs::Odometry curr_odom){
	octomath::Vector3 octo_trans(curr_odom.pose.pose.position.x, curr_odom.pose.pose.position.y, curr_odom.pose.pose.position.z);
	octomath::Quaternion octo_quat(curr_odom.pose.pose.orientation.w, curr_odom.pose.pose.orientation.x, curr_odom.pose.pose.orientation.y, curr_odom.pose.pose.orientation.z);
	octomath::Pose6D cvt_result(octo_trans, octo_quat);
	return cvt_result;
}
octomath::Pose6D srsd_planner::cvt2octomath(geometry_msgs::PoseStamped curr_pose){
	octomath::Quaternion octo_quat(curr_pose.pose.orientation.w, curr_pose.pose.orientation.x, curr_pose.pose.orientation.y, curr_pose.pose.orientation.z);
	octomath::Vector3 octo_euler = octo_quat.toEuler();
	float x = curr_pose.pose.position.x;
	float y = curr_pose.pose.position.y;
	float z = curr_pose.pose.position.z;
	double roll = octo_euler.x();
	double pitch = octo_euler.y();
	double yaw = octo_euler.z();
	octomath::Pose6D cvt_result(x, y, z, roll, pitch, yaw);
	// octomath::Pose6D cvt_result;
	return cvt_result;
}

void srsd_planner::cvt_octo2pubpc()//octomap::OcTree *octree
{
	sensor_msgs::PointCloud2 pub_octo;
	sensor_msgs::PointCloud2 pub_octo_free;
	pcl::PointCloud<pcl::PointXYZ>::Ptr octo_pcl_pub(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr free_octo_pcl_pub(new pcl::PointCloud<pcl::PointXYZ>());
	// for (octomap::OcTree::iterator it=m_octree->begin(); it!=m_octree->end(); ++it){
	for (octomap::OcTree::leaf_iterator it=m_octree->begin_leafs(); it!=m_octree->end_leafs(); ++it){
		if(m_octree->isNodeOccupied(*it))
		{
			octo_pcl_pub->push_back(pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));//malloc
		}
		else
		{
			free_octo_pcl_pub->push_back(pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));
		}
	}
	pub_octo = cloud2msg(*octo_pcl_pub, "map");
	pub_octo_free = cloud2msg(*free_octo_pcl_pub, "map");
	octo_occu_pub.publish(pub_octo);
	octo_free_pub.publish(pub_octo_free);
	octo_pcl_pub->clear();
	free_octo_pcl_pub->clear();
	// octo_pcl_pub.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

float srsd_planner::normalize(float a, float b)						{float sum;  float result;  sum = pow(a,2) + pow(b,2); result = pow(sum,0.5); return result;}
float srsd_planner::normalize(float a, float b, float c)				{float sum;  float result;  sum = pow(a,2) + pow(b,2) + pow(c,2); result = pow(sum,0.5); return result;}
float srsd_planner::normalize(float a, float b, float c, float d)		{float sum;  float result;  sum = pow(a,2) + pow(b,2) + pow(c,2) + pow(d,2); result = pow(sum,0.5); return result;}
double srsd_planner::normalize(double a, double b)					{double sum; double result; sum = pow(a,2) + pow(b,2); result = pow(sum,0.5); return result;}
double srsd_planner::normalize(double a, double b, double c)			{double sum; double result; sum = pow(a,2) + pow(b,2) + pow(c,2); result = pow(sum,0.5); return result;}
double srsd_planner::normalize(double a, double b, double c, double d){double sum; double result; sum = pow(a,2) + pow(b,2) + pow(c,2) + pow(d,2); result = pow(sum,0.5); return result;}

// bool srsd_planner::check_free(geometry_msgs::PoseStamped point, geometry_msgs::PoseStamped next_point, octomap::point3d& hit_point){
//   octomap::point3d start(point.pose.position.x, point.pose.position.y, point.pose.position.z);
//   octomap::point3d direction(next_point.pose.position.x-point.pose.position.x,
//   							 next_point.pose.position.y-point.pose.position.y,
// 							 next_point.pose.position.z-point.pose.position.z);
//   double dist2first = normalize(next_point.pose.position.x-point.pose.position.x,
// 								next_point.pose.position.y-point.pose.position.y,
// 								next_point.pose.position.z-point.pose.position.z);
//   #pragma omp parallel
//   bool hit = m_octree->castRay(start, direction, hit_point, false, dist2first);
// //   auto *end_unknown = m_octree->search(end,0);
// //   if (!ray_result && end_unknown)
// //   {
// //     res.visible = true;
// //   }
// //   else{
// //     res.visible = false;
// //   }
//   return !hit;
// }

// bool srsd_planner::check_free(geometry_msgs::PoseStamped point, int depth=0){
//   octomap::point3d check(point.pose.position.x, point.pose.position.y, point.pose.position.z);
// //   OcTreeKey check_key = coordToKey(check);
// //   octomap::OcTreeNode* node = ;
//   auto *node = m_octree->search(check,depth);
//   if (node){
// 	#pragma omp parallel
// 	return !m_octree->isNodeOccupied(node);
//   }
//   else{
// 	return false; //unknown
//   }  
// }

bool srsd_planner::opti_check_free(octomap::point3d check, int depth){
//   octomap::point3d check(point.pose.position.x, point.pose.position.y, point.pose.position.z);
//   OcTreeKey check_key = coordToKey(check);
//   octomap::OcTreeNode* node = ;
  unsigned int full_depth = m_octree->getTreeDepth();
  auto *node = m_octree->search(check, full_depth-depth);
//   auto *node = m_octree->search(check);
  if (node){
	// #pragma omp parallel
	return !m_octree->isNodeOccupied(node);
  }
  else{
	// return false; //unknown //pessimistic
	return true; //unknown //optimistic
  }  
}
bool srsd_planner::pessi_check_free(octomap::point3d check, int depth){
//   octomap::point3d check(point.pose.position.x, point.pose.position.y, point.pose.position.z);
//   OcTreeKey check_key = coordToKey(check);
//   octomap::OcTreeNode* node = ;
  unsigned int full_depth = m_octree->getTreeDepth();
  auto *node = m_octree->search(check, full_depth-depth);
//   auto *node = m_octree->search(check);
  if (node){
	// #pragma omp parallel
	return !m_octree->isNodeOccupied(node);
  }
  else{
	return false; //unknown //pessimistic
	// return true; //unknown //optimistic
  }  
}


void srsd_planner::subs_start(const std_msgs::Empty::ConstPtr& msg){start_flag = true; require_target = true; hover_flag = false; ROS_INFO("start exploration");mission_start = ros::Time::now();}
void srsd_planner::subs_sensor(const sensor_msgs::PointCloud2::ConstPtr& msg){sensor_data = *msg;}
void srsd_planner::subs_state(const mavros_msgs::State::ConstPtr& msg){armed_flag = msg->armed; offb_flag = (msg->mode == mavros_msgs::State::MODE_PX4_OFFBOARD);}
void srsd_planner::subs_pose(const geometry_msgs::PoseStamped::ConstPtr& msg){
  curr_pose = *msg;

  if (start_flag && !hover_flag){hover_point = curr_pose.pose.position;}
}
