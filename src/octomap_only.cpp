#include "octomap_only.hpp"

bool octomap_only::data_input(sensor_msgs::PointCloud2 input)
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

	if (input.point_step>0 && timeoffset < 0.02){
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

void octomap_only::input_timer(const ros::TimerEvent& event)
{
	data_input(sensor_data);
	cvt_octo2pubpc();

	std::cout << "m_octree: " << m_octree->size() << std::endl;
}


	
sensor_msgs::PointCloud2 octomap_only::cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud, std::string frame_id) // = "camera_link"
{
	sensor_msgs::PointCloud2 cloud_ROS;
	pcl::toROSMsg(cloud, cloud_ROS);
	cloud_ROS.header.frame_id = frame_id;
	return cloud_ROS;
}

pcl::PointCloud<pcl::PointXYZ> octomap_only::cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudresult(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(cloudmsg, *cloudresult);

	// cloudresult.reset(new pcl::PointCloud<pcl::PointXYZ>());
	return *cloudresult;
}

octomap::Pointcloud octomap_only::cloudmsg2octocloud(sensor_msgs::PointCloud2 cloudmsg)
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

octomap::Pointcloud octomap_only::cloud2octocloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn)
{
	int cloudSize = cloudIn->size();
	octomap::Pointcloud octopc_result;

	#pragma omp parallel for num_threads(numberOfCores)
	for (int i = 0; i < cloudSize; ++i){
	  	octopc_result.push_back(cloudIn->points[i].x, cloudIn->points[i].y, cloudIn->points[i].z);
	}
	return octopc_result;
}

octomap::Pointcloud octomap_only::cloud2transOctoCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, Eigen::Matrix4f map_t_sensor)
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


octomath::Pose6D octomap_only::cvt2octomath(nav_msgs::Odometry curr_odom){
	octomath::Vector3 octo_trans(curr_odom.pose.pose.position.x, curr_odom.pose.pose.position.y, curr_odom.pose.pose.position.z);
	octomath::Quaternion octo_quat(curr_odom.pose.pose.orientation.w, curr_odom.pose.pose.orientation.x, curr_odom.pose.pose.orientation.y, curr_odom.pose.pose.orientation.z);
	octomath::Pose6D cvt_result(octo_trans, octo_quat);
	return cvt_result;
}
octomath::Pose6D octomap_only::cvt2octomath(geometry_msgs::PoseStamped curr_pose){
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

void octomap_only::cvt_octo2pubpc()//octomap::OcTree *octree
{
	sensor_msgs::PointCloud2 pub_octo;
	// sensor_msgs::PointCloud2 pub_octo_free;
	pcl::PointCloud<pcl::PointXYZ>::Ptr octo_pcl_pub(new pcl::PointCloud<pcl::PointXYZ>());
	// pcl::PointCloud<pcl::PointXYZ>::Ptr free_octo_pcl_pub(new pcl::PointCloud<pcl::PointXYZ>());
	// for (octomap::OcTree::iterator it=m_octree->begin(); it!=m_octree->end(); ++it){
	for (octomap::OcTree::leaf_iterator it=m_octree->begin_leafs(); it!=m_octree->end_leafs(); ++it){
		if(m_octree->isNodeOccupied(*it))
		{
			octo_pcl_pub->push_back(pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));//malloc
		}
		// else
		// {
		// 	free_octo_pcl_pub->push_back(pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));
		// }
	}
	pub_octo = cloud2msg(*octo_pcl_pub, "map");
	octo_occu_pub.publish(pub_octo);
	octo_pcl_pub->clear();
	// octo_pcl_pub.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

float octomap_only::normalize(float a, float b)						{float sum;  float result;  sum = pow(a,2) + pow(b,2); result = pow(sum,0.5); return result;}
float octomap_only::normalize(float a, float b, float c)				{float sum;  float result;  sum = pow(a,2) + pow(b,2) + pow(c,2); result = pow(sum,0.5); return result;}
float octomap_only::normalize(float a, float b, float c, float d)		{float sum;  float result;  sum = pow(a,2) + pow(b,2) + pow(c,2) + pow(d,2); result = pow(sum,0.5); return result;}
double octomap_only::normalize(double a, double b)					{double sum; double result; sum = pow(a,2) + pow(b,2); result = pow(sum,0.5); return result;}
double octomap_only::normalize(double a, double b, double c)			{double sum; double result; sum = pow(a,2) + pow(b,2) + pow(c,2); result = pow(sum,0.5); return result;}
double octomap_only::normalize(double a, double b, double c, double d){double sum; double result; sum = pow(a,2) + pow(b,2) + pow(c,2) + pow(d,2); result = pow(sum,0.5); return result;}

// bool octomap_only::check_free(geometry_msgs::PoseStamped point, geometry_msgs::PoseStamped next_point, octomap::point3d& hit_point){
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

// bool octomap_only::check_free(geometry_msgs::PoseStamped point, int depth=0){
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

bool octomap_only::check_free(octomap::point3d check, int depth){
//   octomap::point3d check(point.pose.position.x, point.pose.position.y, point.pose.position.z);
//   OcTreeKey check_key = coordToKey(check);
//   octomap::OcTreeNode* node = ;
  auto *node = m_octree->search(check, depth);
  if (node){
	// #pragma omp parallel
	return !m_octree->isNodeOccupied(node);
  }
  else{
	return false; //unknown
  }  
}


void octomap_only::subs_sensor(const sensor_msgs::PointCloud2::ConstPtr& msg){sensor_data = *msg;}
void octomap_only::subs_pose(const geometry_msgs::PoseStamped::ConstPtr& msg){curr_pose = *msg;}