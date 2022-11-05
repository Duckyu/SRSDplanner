#include "exploration.hpp"

// bool srsd_planner::check_x_priority()
// {
//     bool x_priority = (map_max.x - map_min.x)>(map_max.y - map_min.y);
//     //sensor_hor_range
//     //sensor_ver_range

//     if(x_priority){
//         double start_y = 0;
//         while( (start_y-sensor_hor_range) > map_min.y){start_y = start_y - sensor_hor_range;}
//         start_point.x = 0;
//         start_point.y = start_y;
//         start_point.z = sensor_ver_range/2.0;
//     }
//     else{
//         double start_x = 0;
//         while( (start_x-sensor_hor_range) > map_min.y){start_x = start_x - sensor_hor_range;}
//         start_point.x = start_x;
//         start_point.y = 0;
//         start_point.z = sensor_ver_range/2.0;
//     }

//     return x_priority;
// }


bool srsd_planner::arrival_check(){
    double res_dist = normalize(target.x-curr_pose.pose.position.x,
                                target.y-curr_pose.pose.position.y,
                                target.z-curr_pose.pose.position.z);
    if (res_dist < sensor_hor_range*0.1){
        return true;
    }
    else{
        return false;
    }
}

// bool srsd_planner::key_compare(const octomap::OcTreeKey& a, const octomap::OcTreeKey& b){
// bool srsd_planner::key_compare(const pair<octomap::OcTreeKey,double>& a, const pair<octomap::OcTreeKey,double>& b){
// bool key_compare(const pair<octomap::OcTreeKey, double>& a, const pair<octomap::OcTreeKey, double>& b){
//     // auto a_f = opened_f.find(a)->second;
//     // auto b_f = opened_f.find(b)->second;
//     return a.second > b.second; // end is lowest
// }

bool srsd_planner::astar(octomap::point3d start, octomap::point3d end, nav_msgs::Path& path){
    octomap::point3d bound_min;
    octomap::point3d bound_max;

    float dist = normalize(end.x()-start.x(),
                           end.y()-start.y(),
                           end.z()-start.z());
    
    bound_min.x() = min(end.x(),start.x())-dist/2.0;
    bound_min.y() = min(end.y(),start.y())-dist/2.0;
    bound_min.z() = min(end.z(),start.z())-dist/2.0;
    bound_max.x() = max(end.x(),start.x())+dist/2.0;
    bound_max.y() = max(end.y(),start.y())+dist/2.0;
    bound_max.z() = max(end.z(),start.z())+dist/2.0;

    // vector<int, int, int> key;
    // map<vector<int, int, int>,vector<int, int, int>> neighbor;
    // vector<octomap::OcTreeKey> closed_key_list;
    // vector<octomap::OcTreeKey> opened_key_list;
    // map<octomap::OcTreeKey, octomap::OcTreeKey> child_parent; // key, G distance

    // map<octomap::OcTreeKey, double> closed_g; // key, G distance
    // map<octomap::OcTreeKey, double> closed_h; // key, H distance
    // map<octomap::OcTreeKey, double> closed_f; // key, F distance
    // map<octomap::OcTreeKey, double> opened_g; // key, distance
    // map<octomap::OcTreeKey, double> opened_h; // key, distance
    // map<octomap::OcTreeKey, double> opened_f; // key, distance

    closed_key_list.clear();
    opened_key_list.clear();
    child_parent.clear(); // key, G distance

    closed_g.clear(); // key, G distance
    closed_h.clear(); // key, H distance
    closed_f.clear(); // key, F distance
    opened_g.clear(); // key, distance
    opened_h.clear(); // key, distance
    opened_f.clear(); // key, distance

    octomap::OcTreeKey start_key = m_octree->coordToKey(start, (int)size_depth);
    int selected_id = 0;
    octomap::OcTreeKey selected_key = start_key;
    opened_key_list.emplace(selected_id,selected_key);
    opened_g.emplace(selected_id,0.0);
    opened_h.emplace(selected_id,end.x()+end.y()+end.z()-start.x()-start.y()-start.z());
    opened_f.emplace(selected_id,end.x()+end.y()+end.z()-start.x()-start.y()-start.z());

    // #pragma omp parallel
    for (int i =0 ; i< 6000; i++)//reachable path check with iteration 1000 -> abort the path when 1000 time to reach the goal
    {
        const int id = selected_id;
        closed_g.emplace(selected_id, opened_g[selected_id]);
        closed_h.emplace(selected_id, opened_h[selected_id]);
        closed_f.emplace(selected_id, opened_f[selected_id]);
        closed_key_list.emplace(selected_id, selected_key);
        opened_key_list.erase(id);
        opened_g.erase(id);
        opened_h.erase(id);
        opened_f.erase(id);
        // (int)size_depth

        octomap::point3d selected_point = m_octree->keyToCoord(selected_key);
        octomap::point3d candidate[6];
        octomap::point3d plus_x  = selected_point;
        octomap::point3d plus_y  = selected_point;
        octomap::point3d plus_z  = selected_point;
        octomap::point3d minus_x = selected_point;
        octomap::point3d minus_y = selected_point;
        octomap::point3d minus_z = selected_point;

        plus_x.x() += unit;
        plus_y.y() += unit;
        plus_z.z() += unit;
        minus_x.x() -= unit;
        minus_y.y() -= unit;
        minus_z.z() -= unit;
        candidate[0] = plus_x;
        candidate[1] = plus_y;
        candidate[2] = plus_z;
        candidate[3] = minus_x;
        candidate[4] = minus_y;
        candidate[5] = minus_z;

        // #pragma omp parallel
        for(int j=0; j<6;j++){
            if(candidate[j].x()>bound_min.x() &&
               candidate[j].y()>bound_min.y() &&
               candidate[j].z()>bound_min.z() &&
               candidate[j].x()<bound_max.x() &&
               candidate[j].y()<bound_max.y() &&
               candidate[j].z()<bound_max.z()){
                octomap::OcTreeKey candidate_key = m_octree->coordToKey(candidate[j],(int)size_depth);
                map<int,octomap::OcTreeKey>::iterator it;
                map<int,octomap::OcTreeKey>::iterator closed_search = closed_key_list.end();
                map<int,octomap::OcTreeKey>::iterator opened_search = opened_key_list.end();
                for(it = closed_key_list.begin();it!=closed_key_list.end();it++){
                    if(it->second == candidate_key){
                        closed_search = it;
                        break;
                    }
                }
                for(it = opened_key_list.begin();it!=opened_key_list.end();it++){
                    if(it->second == candidate_key){
                        opened_search = it;
                        break;
                    }
                }
                double g = closed_g.at(selected_id);
                g += unit;
                //manhatten distance
                double h = end.x()+end.y()+end.z()-candidate[j].x()-candidate[j].y()-candidate[j].z();
                double f = g + h;
                if(closed_search == closed_key_list.end() && opened_search == opened_key_list.end() ){
                    if(check_free(candidate[j],(int)size_depth)){
                        opened_g.emplace(opened_key_list.size()+closed_key_list.size(),g);
                        opened_h.emplace(opened_key_list.size()+closed_key_list.size(),h);
                        opened_f.emplace(opened_key_list.size()+closed_key_list.size(),f);
                        opened_key_list.emplace(opened_key_list.size()+closed_key_list.size(),candidate_key);
                        cout << "open cell registered!!!!"<<endl;
                    }
                }
            }
        }

        int min_id = 0;
        double min_val = 0.0;
        map<int,double>::iterator opened_f_it;
        for (opened_f_it = opened_f.begin();opened_f_it != opened_f.end();opened_f_it++){
            if(min_val == 0){min_val = opened_f_it->second;}
            else{
                if (min_val>opened_f_it->second){
                    min_id = opened_f_it->first;
                    min_val = opened_f_it->second;
                }
            }
        }

        selected_id = min_id;
        
        octomap::OcTreeKey prev_selected_key = selected_key;
        int prev_selected_id = selected_id;
        selected_key = opened_key_list[selected_id];
        child_parent.emplace(selected_id, prev_selected_id);
        
        octomap::point3d selected_coord = m_octree->keyToCoord(selected_key);
        cout <<"opened_key_list.size()" << opened_key_list.size() <<endl;
        cout << "Selected id: " << selected_id << "," "Selected coord: " << selected_coord.x() << "," << selected_coord.y() << "," << selected_coord.z() << "," << endl;
        cout << "G,H,F : " << opened_g.at(selected_id) << " ," << opened_h.at(selected_id) << " ," << opened_f.at(selected_id)<< endl;
        cout << "------------------------------------"<< endl;

        // opened_key_list.pop_back();
        // opened_key_list.erase(std::remove(opened_key_list.begin(), opened_key_list.end(), selected_key), opened_key_list.end());
        if(opened_key_list.size() == 1){return false;}

        float res_dist = normalize(end.x()-selected_coord.x(),
                                   end.y()-selected_coord.y(),
                                   end.z()-selected_coord.z());
        if(res_dist < sensor_hor_range*0.1){
            vector<geometry_msgs::PoseStamped> poses;
            path.poses.clear();
            geometry_msgs::PoseStamped temp_pose;
            temp_pose.pose.orientation.x = 0;
            temp_pose.pose.orientation.y = 0;
            temp_pose.pose.orientation.z = 0;
            temp_pose.pose.orientation.w = 1;
            temp_pose.pose.position.x = end.x();
            temp_pose.pose.position.y = end.y();
            temp_pose.pose.position.z = end.z();
            poses.emplace_back(temp_pose);
                
            octomap::OcTreeKey parent_key = closed_key_list.at(child_parent.at(selected_id));

            #pragma omp parallel
            while (parent_key != start_key){
                octomap::point3d parent_point = m_octree->keyToCoord(parent_key);
                temp_pose.pose.position.x = parent_point.x();
                temp_pose.pose.position.y = parent_point.y();
                temp_pose.pose.position.z = parent_point.z();
                poses.emplace_back(temp_pose);
                parent_key = closed_key_list.at(child_parent.at(child_parent.at(selected_id)));
            }

            temp_pose.pose.position.x = start.x();
            temp_pose.pose.position.y = start.y();
            temp_pose.pose.position.z = start.z();
            poses.emplace_back(temp_pose);

            for (int k=0; k<poses.size();k++){
                path.poses.emplace_back(poses[poses.size()-1-k]);
            }            
            return true;
        }
    }
    return false;
}

void srsd_planner::planner_timer(const ros::TimerEvent& event)
{
	//planner code
    if(require_target && start_flag)
    {
        for(int i =0; i<6;i++){
            octomap::point3d start(curr_pose.pose.position.x,curr_pose.pose.position.y,curr_pose.pose.position.z);
            octomap::point3d temp_end(start_point.x,start_point.y,start_point.z);
            temp_end.x() += now[0]*sensor_hor_range;
            temp_end.y() += now[1]*sensor_hor_range;
            temp_end.z() += now[2]*sensor_ver_range;

            int current[3];
            current[0] = now[0];
            current[1] = now[1];
            current[2] = now[2];

            if(i == 0)     {temp_end.z() -= sensor_ver_range; current[2] -= 1;}
            else if(i == 1){temp_end.y() -= sensor_hor_range; current[1] -= 1;}
            else if(i == 2){temp_end.x() -= sensor_hor_range; current[0] -= 1;}
            else if(i == 3){temp_end.x() += sensor_hor_range; current[0] += 1;}
            else if(i == 4){temp_end.y() += sensor_hor_range; current[1] += 1;}
            else if(i == 5){temp_end.z() += sensor_ver_range; current[2] += 1;}

            const octomap::point3d checker(current[0],current[1],current[2]);
            auto it = find(ban_list.begin(),ban_list.end(),checker);
            if(temp_end.x()>map_min.x &&
               temp_end.y()>map_min.y &&
               temp_end.z()>map_min.z &&
               temp_end.x()<map_max.x &&
               temp_end.y()<map_max.y &&
               temp_end.z()<map_max.z &&
               it == ban_list.end()){
                if(astar(start, temp_end, calculated_path)){
                    now[0] = current[0];
                    now[1] = current[1];
                    now[2] = current[2];
                    octomap::point3d temp_ban(current[0],current[1],current[2]);
                    ban_list.push_back(temp_ban);
                    octomap::point3d start(now[0],now[1],now[2]);
                    require_target = false;
                    break;
                }
            }
        }
        end_flag = true;
    }
}


void srsd_planner::control_timer(const ros::TimerEvent& event)
{
    if(armed_flag){
        if(!offb_flag && (ros::Time::now() - offb_request > ros::Duration(1.0))){
            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            set_mode_client.call(offb_set_mode);
            offb_request = ros::Time::now();
            cout << "takeoff"<< endl;
        }
        //control code
        if(require_target || end_flag || !start_flag)
        {
            hover_flag = true;
            // cout << "hover"<< endl;
            // cout << "hover_point: "<< hover_point.x << ", "<< hover_point.y << ", "<< hover_point.z << endl;
            //hover
            control_msg.type_mask = position_control_type;
            control_msg.position = hover_point;
        }
        else
        {
            hover_flag = false;
            //arrival check -> if true, require_target = true
            if (!arrival_check()){
                //FOLLOW PATH
                control_msg.type_mask = position_control_type;
                control_msg.position = calculated_path.poses[path_idx].pose.position;
            }
            else{
                path_idx++;
                if(path_idx == calculated_path.poses.size()){
                    require_target = true;
                    path_idx = 0;
                    cout << "require new path" <<endl;
                }
                else{
                    cout << "next!" <<endl;
                    target = calculated_path.poses[path_idx].pose.position;
                }
            }
        }
    }
    setpoint_raw_pub.publish(control_msg);
}
