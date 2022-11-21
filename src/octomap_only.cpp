#include "octomap_only.hpp"

// ros::Duration calc_process_duration(ros::Time& process_start_time)
// {
// 	ros::Duration process_time;
// 	process_time = ros::Time::now() - process_start_time;
// 	process_start_time = ros::Time::now();
// }

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

	Eigen::MatrixXi input_checkmatrix(sensor_ring, sensor_samples);
	// cv::Mat input_checkmatrix(sensor_ring, sensor_samples, CV_32F);

	double timeoffset = fabs(curr_pose.header.stamp.toSec() - input.header.stamp.toSec());
	ros::Time process_start_time = ros::Time::now();
	ros::Duration process_time = ros::Time::now() - process_start_time;
	// ROS_INFO("timeoffset: %lf",timeoffset);
	if (input.width > 0 && timeoffset < update_tolerance){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>());
		*cloudIn = cloudmsg2cloud(input);
		// cout << "data_input size: " << cloudIn->size()<< endl;

		shared_ptr<octomap::Pointcloud> transOcto = make_shared<octomap::Pointcloud>();
		*transOcto = cloud2transOctoCloud(cloudIn, map_t_sensor);

		octomap::point3d sensor_origin(map_t_sensor(0,3),map_t_sensor(1,3),map_t_sensor(2,3));
		m_octree->insertPointCloud(*transOcto, sensor_origin, update_sensor_range);

		transOcto->clear();

		process_time = ros::Time::now() - process_start_time;
		process_start_time = ros::Time::now();
		ROS_INFO("Trans Pointcloud insertion: %lf", process_time.toSec());

		#pragma omp for
		for (const auto& pt: cloudIn->points){
			float d = pow(pow(pt.x,2)+pow(pt.y,2),0.5);
			int pt_ring   = round((atan2(pt.z, d) - sensor_vert_min)/(sensor_vert_max - sensor_vert_min)*((double)sensor_ring-1.0));
			int pt_sample = round((atan2(pt.y, pt.x) - sensor_hori_min)/(sensor_hori_max - sensor_hori_min)*((double)sensor_samples-1.0));
			// cout << "pt x,y,z,d: " << pt.x << ", " << pt.y << ", " << pt.z << ", " << d << endl;
			// cout << "alpha, beta: " << atan2(pt.z, d) << ", " << atan2(pt.y, pt.x) << endl;
			// cout << "pt_ring, pt_sample: " << pt_ring << "," << pt_sample << endl;
			input_checkmatrix(pt_ring,pt_sample) = -1;
		}
		// cout << input_checkmatrix << endl;

		process_time = ros::Time::now() - process_start_time;
		process_start_time = ros::Time::now();
		ROS_INFO("Missing ray check: %lf", process_time.toSec());

		pcl::PointCloud<pcl::PointXYZ>::Ptr free_ends(new pcl::PointCloud<pcl::PointXYZ>());

		#pragma omp for
		// for (int i = 0; i < sensor_ring; ++i){
		for (int i : free_ray_ring){
			#pragma omp for
            for (int j = 0; j < sensor_samples; ++j){
				// if(input_checkmatrix(i,j) != -1){
				if(input_checkmatrix(i,j) != -1 &&
				   input_checkmatrix(i-1,j) != -1 &&
				   input_checkmatrix(i,j-1) != -1 &&
				   input_checkmatrix(i+1,j) != -1 &&
				   input_checkmatrix(i+1,j+1) != -1 &&
				   input_checkmatrix(i+1,j-1) != -1 &&
				   input_checkmatrix(i-1,j+1) != -1 &&
				   input_checkmatrix(i-1,j-1) != -1){ // sliding window for false-positive elimination
					pcl::PointXYZ free_end;
					double free_hori_angle = sensor_hori_min+j*(sensor_hori_max - sensor_hori_min)/((double)sensor_samples - 1.0);
					double free_vert_angle = sensor_vert_min+i*(sensor_vert_max - sensor_vert_min)/((double)sensor_ring - 1.0);
					
					free_end.x = max_sensor_range * cos(free_vert_angle) * cos(free_hori_angle);
					free_end.y = max_sensor_range * cos(free_vert_angle) * sin(free_hori_angle);
					free_end.z = max_sensor_range * sin(free_vert_angle);
					
					free_ends->push_back(free_end);
					// cout << "angles: " << free_hori_angle << ", " << free_vert_angle << endl;
				}
            }
        }

		process_time = ros::Time::now() - process_start_time;
		process_start_time = ros::Time::now();
		ROS_INFO("Free ray generation: %lf", process_time.toSec());
		if(use_free_ray){
			octomap::Pointcloud transFree = cloud2transOctoCloud(free_ends, map_t_sensor);

			sensor_msgs::PointCloud2 freeMsg;
			pcl::toROSMsg(*free_ends, freeMsg);
			freeMsg.header.frame_id = input.header.frame_id;
			free_ray_pub.publish(freeMsg);

			process_time = ros::Time::now() - process_start_time;
			process_start_time = ros::Time::now();
			ROS_INFO("Free ray publish: %lf", process_time.toSec());

			double end_end = (double)floor(max_sensor_range/octomap_resolution);
			// float log_odds_free_update = log(octomap_miss) - log(1 - octomap_miss);
			float log_odds_free_update = log(0.49) - log(1 - 0.49);

			#pragma omp for
			for (auto it = transFree.begin(); it != transFree.end(); ++it)		
			{
				octomap::point3d end = *it;
				octomap::point3d _prev_update = sensor_origin;
				#pragma omp for
				for(int i = 1; i < end_end; i++){
					octomap::point3d _update = sensor_origin+(end-sensor_origin)*((double)i/end_end);
					if((_update.x()<map_min_x ||
						_update.y()<map_min_y ||
						_update.z()<map_min_z ||
						_update.x()>map_max_x ||
						_update.y()>map_max_y ||
						_update.z()>map_max_z) &&
					(_prev_update.x()>map_min_x &&
						_prev_update.y()>map_min_y &&
						_prev_update.z()>map_min_z &&
						_prev_update.x()<map_max_x &&
						_prev_update.y()<map_max_y &&
						_prev_update.z()<map_max_z)){break;}
					if(_update.x()>map_min_x &&
					_update.y()>map_min_y &&
					_update.z()>map_min_z &&
					_update.x()<map_max_x &&
					_update.y()<map_max_y &&
					_update.z()<map_max_z)
					{
						// m_octree->updateNode(_update, false, true);
						m_octree->updateNode(_update, log_odds_free_update, true);
					}
					_prev_update = _update;
				}
			}
			
			process_time = ros::Time::now() - process_start_time;
			process_start_time = ros::Time::now();
			ROS_INFO("Free ray update: %lf", process_time.toSec());

			m_octree->updateInnerOccupancy();
			
			process_time = ros::Time::now() - process_start_time;
			process_start_time = ros::Time::now();
			ROS_INFO("updateInnerOccupancy: %lf", process_time.toSec());
			transFree.clear();
			free_ends->clear();
		}
		cloudIn->clear();

	}
	else if(input.width <= 0){
		if(input_log){
			no_input_cnt += 1;
			std::cout << no_input_cnt << " no input!!!" << std::endl;
			result = false;
		}
	}
	else if(timeoffset > update_tolerance){
		if(input_log){
			std::cout << "time delayed" << std::endl;
			result = false;
		}
	}
	
	return result;
}

void octomap_only::input_timer(const ros::TimerEvent& event)
{
	data_input(sensor_data);
	cvt_octo2pubpc();
	if(input_log){
		std::cout << "m_octree: " << m_octree->size() << std::endl;
	}
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


// octomath::Pose6D octomap_only::cvt2octomath(nav_msgs::Odometry curr_odom){
// 	octomath::Vector3 octo_trans(curr_odom.pose.pose.position.x, curr_odom.pose.pose.position.y, curr_odom.pose.pose.position.z);
// 	octomath::Quaternion octo_quat(curr_odom.pose.pose.orientation.w, curr_odom.pose.pose.orientation.x, curr_odom.pose.pose.orientation.y, curr_odom.pose.pose.orientation.z);
// 	octomath::Pose6D cvt_result(octo_trans, octo_quat);
// 	return cvt_result;
// }
// octomath::Pose6D octomap_only::cvt2octomath(geometry_msgs::PoseStamped curr_pose){
// 	octomath::Quaternion octo_quat(curr_pose.pose.orientation.w, curr_pose.pose.orientation.x, curr_pose.pose.orientation.y, curr_pose.pose.orientation.z);
// 	octomath::Vector3 octo_euler = octo_quat.toEuler();
// 	float x = curr_pose.pose.position.x;
// 	float y = curr_pose.pose.position.y;
// 	float z = curr_pose.pose.position.z;
// 	double roll = octo_euler.x();
// 	double pitch = octo_euler.y();
// 	double yaw = octo_euler.z();
// 	octomath::Pose6D cvt_result(x, y, z, roll, pitch, yaw);
// 	// octomath::Pose6D cvt_result;
// 	return cvt_result;
// }

void octomap_only::cvt_octo2pubpc()//octomap::OcTree *octree
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
	pub_octo = cloud2msg(*octo_pcl_pub, curr_pose.header.frame_id);
	pub_octo_free = cloud2msg(*free_octo_pcl_pub, curr_pose.header.frame_id);
	octo_occu_pub.publish(pub_octo);
	octo_free_pub.publish(pub_octo_free);
	octo_pcl_pub->clear();
	free_octo_pcl_pub->clear();
}

// float octomap_only::normalize(float a, float b)						{float sum;  float result;  sum = pow(a,2) + pow(b,2); result = pow(sum,0.5); return result;}
// float octomap_only::normalize(float a, float b, float c)				{float sum;  float result;  sum = pow(a,2) + pow(b,2) + pow(c,2); result = pow(sum,0.5); return result;}
// float octomap_only::normalize(float a, float b, float c, float d)		{float sum;  float result;  sum = pow(a,2) + pow(b,2) + pow(c,2) + pow(d,2); result = pow(sum,0.5); return result;}
// double octomap_only::normalize(double a, double b)					{double sum; double result; sum = pow(a,2) + pow(b,2); result = pow(sum,0.5); return result;}
// double octomap_only::normalize(double a, double b, double c)			{double sum; double result; sum = pow(a,2) + pow(b,2) + pow(c,2); result = pow(sum,0.5); return result;}
// double octomap_only::normalize(double a, double b, double c, double d){double sum; double result; sum = pow(a,2) + pow(b,2) + pow(c,2) + pow(d,2); result = pow(sum,0.5); return result;}

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
bool octomap_only::opti_check_free(octomap::point3d check, int depth){
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
bool octomap_only::pessi_check_free(octomap::point3d check, int depth){
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
void octomap_only::subs_sensor(const sensor_msgs::PointCloud2::ConstPtr& msg){sensor_data = *msg;}
void octomap_only::subs_pose(const geometry_msgs::PoseStamped::ConstPtr& msg){curr_pose = *msg;}
void octomap_only::subs_odom(const nav_msgs::Odometry::ConstPtr& msg){curr_pose.header = msg->header;curr_pose.pose = msg->pose.pose;}