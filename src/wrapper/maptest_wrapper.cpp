#include <wrapper/maptest_wrapper.h>
#include <utils/ros_utils.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace traj_lib;
using namespace std;
using namespace transform_utils;

namespace online_planner{
MaptestWrapper::MaptestWrapper():is_trajectory_set(false), trajectory(3), kf_callback_timer(" - Whole KfInfoCallback timer"){
    ot_handle_ = std::make_shared<OctomapHandler>(loadOctomapHandlerParam(nh_custom_));
    fm_handle_ = std::make_shared<FeatureMapHandler>(loadFmParam(nh_custom_));
    fm_handle_->setOctHandle(ot_handle_);
    trajectory.setDuration(1.0);
    trajectory.reset(0.0);
    M.setZero(1, 3);
    M<<10.0, -20.0, 20.0; // duration = 1.0
    last_valid_yaw = 0.0;
    current_best_trajectory = &trajectory;
    
    //params
    T_bc = loadTransformFromRos(std::string("T_bc"), nh_default_);
    nh_default_.param("plan_delay", t_plan_delay, 0.01);
    nh_default_.param("v_des", v_des, 1.5);
    nh_default_.param("d_fcut", d_fcut, 20.0);
    double h_fov, v_fov;
    nh_default_.param("h_fov", h_fov, 90.0);
    nh_default_.param("v_fov", v_fov, 73.74);
    double h_fov_rad = h_fov * M_PI / 180.0;
    double v_fov_rad = v_fov * M_PI / 180.0;
    fov_rads = make_pair(h_fov_rad, v_fov_rad);
    nh_default_.param("print_setstate", print_setstate, false);
    nh_default_.param("print_trajectory", print_trajectory, false);
    
    //visualization
    nh_default_.param("query_viz/l_arrow", l_arrow, 1.0);
    nh_default_.param("query_viz/d_cam", d_cam, 4.0);
    nh_default_.param("query_viz/s_markers", s_markers, 0.05);

    //verbosity
    nh_default_.param("ot_verbose", ot_verbose_, true);
    nh_default_.param("fm_verbose", fm_verbose_, true);

    //define actors in nh_default_
    depth_img_sub = nh_default_.subscribe("/camera/depth/image_raw", 1, &MaptestWrapper::depthImgCallback, this);
    depth_cam_info_sub = nh_default_.subscribe("/camera/depth/camera_info", 5, &MaptestWrapper::depthcamInfoCallback, this);
    kf_info_sub = nh_default_.subscribe("/vins_estimator/keyframe_info", 3, &MaptestWrapper::KfInfoCallback, this);
    visualize_timer = nh_default_.createTimer(ros::Duration(1.0), &MaptestWrapper::visualizeCallback, this);
    
    octomap_pub = nh_default_.advertise<octomap_msgs::Octomap>("octomap", 1);
    //fm_pub = nh_default_.advertise<sensor_msgs::PointCloud2>("feature_map", 1); not for now
    query_result_pub = nh_default_.advertise<visualization_msgs::MarkerArray>("query_result", 1);

    //define actors in nh_private_
    global_plan_timer = nh_custom_.createTimer(ros::Duration(dt_global_planning), &MaptestWrapper::airsimGlobalCallback, this);
    local_plan_timer = nh_custom_.createTimer(ros::Duration(dt_local_planning), &MaptestWrapper::localPlannerCallback, this, false, false);


    custom_spinner->start();
}

void MaptestWrapper::depthImgCallback(const sensor_msgs::ImageConstPtr& img_msg){
    if(!(ot_handle_->isCamModelSet())) return;
    if(!transformStabilized()) return;
    MavState mav_state = getCurrState();
    SE3 T_wb = from_Rp(mav_state.R, mav_state.p);
    SE3 T_wc = T_wb * T_bc;
    octomath::Pose6D T_wc_oct = SE3tooctPose(T_wc);

    ot_handle_->insertPointcloud(img_msg, T_wc_oct);
    pcl::PointCloud<pcl::PointXYZI> changed = ot_handle_->trackChanges();
    ot_handle_->insertUpdate(changed);
}

void MaptestWrapper::depthcamInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg){
    ot_handle_->setDepthcamModel(*cam_info_msg);
}

void MaptestWrapper::KfInfoCallback(const vins_vio_mod::KeyframeInfoConstPtr& kf_msg){
    kf_callback_timer.tic();
    KFInfo kf_info;
    vins_vio_mod::KeyframeInfo kf = *kf_msg;
    kfMsgToKfInfo(kf, kf_info, d_fcut);
    fm_handle_->updateKfInfo(kf_info);
    kf_callback_timer.toc();
}

void MaptestWrapper::airsimGlobalCallback(const ros::TimerEvent& e){
    ros::Time start = ros::Time::now();
    ros::Duration dur_ctrl(dt_control);
    switch(status_){
        case Status::ODOM_NOT_SET:{
            ROS_WARN_ONCE("Odom not set. waiting for odometry info...");
            while(true){
                airsimVioInitSingleIter();
                if((ros::Time::now() - start).toSec() >= dt_global_planning - dt_control) break;
                dur_ctrl.sleep();
            }
            break;
        }
        case Status::ODOM_INIT:
            while(true){
                airsimVioInitSingleIter();
                if((ros::Time::now() - start).toSec() >= dt_global_planning - dt_control) break;
                dur_ctrl.sleep();
            }
            break;
        case Status::TAKEOFF_SEQUENCE:
            while(true){
                airsimTakeoffSingleIter();
                if((ros::Time::now() - start).toSec() >= dt_global_planning - dt_control) break;
                dur_ctrl.sleep();
            }
            break;
        case Status::INIT_HOVER:{
            while(true){
                airsimHoverSingleIter();
                if((ros::Time::now() - start).toSec() >= dt_global_planning - dt_control) break;
                dur_ctrl.sleep();
            }
            break;
        }
        case Status::FLIGHT:
            if(!local_plan_timer.hasStarted()){
                local_plan_timer.start();
            }
            checkGoalReached();
            break;
        default:
            if(local_plan_timer.hasStarted()) local_plan_timer.stop();
            vel_cmd_.twist.linear.x = 0.0;
            vel_cmd_.twist.linear.y = 0.0;
            vel_cmd_.twist.linear.z = 0.0;
            airsim_vel_pub.publish(vel_cmd_);
            break;
    }
    return;
}


void MaptestWrapper::localPlannerCallback(const ros::TimerEvent& e){
    airsim_controller::PositionTargets traj;
    ros::Time t_preplan = ros::Time::now();
    FlatState curr_flat_state = getCurrFlatState();
    last_valid_yaw = curr_flat_state.states[0].yaw; 
    FlatState init_state = getInitState();
    if(print_setstate){
        cout<<"===next initial state : "<<endl;
        cout<<"p : "<<init_state.states[0].p.transpose()<<", yaw : "<<init_state.states[0].yaw<<endl;
        cout<<"v : "<<init_state.states[1].p.transpose()<<endl;
        cout<<"a : "<<init_state.states[2].p.transpose()<<endl;
    }
    computePlan(t_preplan, init_state); 
    double t_start, t_end;
    getTimeSpan(t_start, t_end);
    t_start = getSimTime(t_preplan) + t_plan_delay;
    traj.header.frame_id = world_frame_name;
    traj.header.stamp = ros::Time::now();
    getSetpoints(t_start, t_end, traj.setpoints);
    sp_pub.publish(traj);
}

void MaptestWrapper::visualizeCallback(const ros::TimerEvent& e){
    //do some featuremap visibility query along the trajectory to check query time
    if(!transformStabilized()) return;
    double t_start, t_end;
    getTimeSpan(t_start, t_end);
    if(ot_verbose_) ot_handle_->printTimers(true, true); // ot->lock_sub, lock_pub
    auto viz_info = visibilityQueryTest(t_start, t_end, 0.1);
    if(fm_verbose_) {
        fm_handle_->printTimers(true, true); // fm->lock
        kf_callback_timer.print(false, true);
    }
    octomap_msgs::Octomap oct_msg;
    oct_msg.header.stamp = ros::Time::now();
    oct_msg.header.frame_id = world_frame_name;
    ot_handle_->getOctomapMsg(oct_msg);
    octomap_pub.publish(oct_msg);
    sensor_msgs::PointCloud2 pcd_msg;
    FeatureVoxelMap::type fm;
    fm_handle_->getFeatureMap(&fm);
    FeatureMapToPointCloud2(fm, pcd_msg, fm_handle_->getNfeatures());
    pcd_msg.header.stamp = ros::Time::now();
    pcd_msg.header.frame_id = world_frame_name;
    //fm_pub.publish(pcd_msg);
    
    //generate query Result visualization message
    visualization_msgs::MarkerArray query_marker;
    generateQueryResultViz(query_marker, viz_info);
    query_result_pub.publish(query_marker);
}

//visualizing query result of visualization query
// 0) arrow type to draw camera optical axis
// 1) line segments for drawing camera frustum
// 2) points 
void MaptestWrapper::generateQueryResultViz(visualization_msgs::MarkerArray& qr_msg, const visQueryResult& res){
    qr_msg.markers.resize(3);
    SE3 T_wc = res.T_wc;
    Eigen::Vector3d cam_position = T_wc.translation();
    geometry_msgs::Point cam_pos_g = eigen_to_geovec<geometry_msgs::Point>(cam_position);
    Eigen::Vector3d z_wc = T_wc.rotation().block<3, 1>(0, 2);
    //common settings for markers
    for(auto& marker: qr_msg.markers){
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = world_frame_name;
        marker.frame_locked = false;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.0);
    }

    //First, draw an arrow indicating the optical axis of the camera
    qr_msg.markers[0].type = visualization_msgs::Marker::ARROW;
    qr_msg.markers[0].points.push_back(cam_pos_g);
    Eigen::Vector3d tip_position = cam_position + z_wc * l_arrow;
    qr_msg.markers[0].points.push_back(eigen_to_geovec<geometry_msgs::Point>(tip_position));
    qr_msg.markers[0].scale.x = l_arrow/10.0;
    qr_msg.markers[0].scale.y = l_arrow/5.0;

    //Next, draw the camera frustum 
    qr_msg.markers[1].type = visualization_msgs::Marker::LINE_LIST;
    qr_msg.markers[1].scale.x = 0.1;
    qr_msg.markers[1].color.a = 1.0; qr_msg.markers[1].color.r = 0.0; qr_msg.markers[1].color.g = 1.0; qr_msg.markers[1].color.b = 0.0; 
    qr_msg.markers[1].pose = utils::SE3_to_gpose(T_wc); // camera pose transform
    double tan_hfov = tanf64(fov_rads.first);
    double tan_vfov = tanf64(fov_rads.second);
    Eigen::Vector3d v_frame[4];
    v_frame[0] = Eigen::Vector3d(-tan_hfov, -tan_vfov, d_cam);
    v_frame[1] = Eigen::Vector3d(tan_hfov, -tan_vfov, d_cam);
    v_frame[2] = Eigen::Vector3d(tan_hfov, tan_vfov, d_cam);
    v_frame[3] = Eigen::Vector3d(-tan_hfov, tan_vfov, d_cam);
    geometry_msgs::Point p_=eigen_to_geovec<geometry_msgs::Point>(cam_position + v_frame[0]);
    for(int i=0; i<4; ++i){
        //skeleton
        qr_msg.markers[1].points.emplace_back(cam_pos_g);
        qr_msg.markers[1].points.emplace_back(p_);
        //frame
        qr_msg.markers[1].points.emplace_back(p_);
        p_ = eigen_to_geovec<geometry_msgs::Point>(cam_position + v_frame[(i+1)%4]);
        qr_msg.markers[1].points.emplace_back(p_);
    }

    //Next, draw the points
    qr_msg.markers[2].type = visualization_msgs::Marker::POINTS;
    qr_msg.markers[2].color.a = 1.0; qr_msg.markers[2].color.r = 0.0; 
    qr_msg.markers[2].color.g = 0.0; qr_msg.markers[2].color.b = 1.0; //blue points
    SE3 T_I = from_Matrix4d(Eigen::Matrix4d::Identity());
    qr_msg.markers[2].pose = utils::SE3_to_gpose(T_I);
    for(const auto& f : res.f_vec){
        qr_msg.markers[2].points.emplace_back(eigen_to_geovec<geometry_msgs::Point>(f.pos));
    }
    return;
}

//functions which earn state_mtx_ or traj_path_mtx_ on its own
FlatState MaptestWrapper::getInitState(){
    traj_lib::FlatState flat_state = getCurrFlatState(); //gets state_lock
    if(print_setstate){
        cout<<"Last odom input -> p "<<flat_state.states[0].p.transpose()<<", v : "<<flat_state.states[1].p.transpose()<<endl;
    }
    unique_lock<mutex> traj_lock(traj_path_mtx_);
    //modify flat_state based on constant velocity and time delay
    ros::Time now = ros::Time::now();
    double t_from_last_odom = (now - t_last_odom_input).toSec();
    if(!is_trajectory_set) return flat_state;
    FlatState next_sp = dynamic_cast<MinJerkPolyTraj*>(current_best_trajectory)->getFlatState(getSimTime(now) + t_plan_delay);
    flat_state.states[0].p += flat_state.states[1].p * t_from_last_odom + next_sp.states[2].p * (t_from_last_odom) * (t_from_last_odom)/2;
    flat_state.states[1].p += next_sp.states[2].p * t_from_last_odom;
    flat_state.states[2].p = next_sp.states[2].p;
    flat_state.states[0].yaw = flat_state.states[0].yaw;
    return flat_state;
}

void MaptestWrapper::computePlan(ros::Time t_preplan, FlatState init_state){
    //compute subgoal
    unique_lock<mutex> traj_lock(traj_path_mtx_);
    trajectory.reset(getSimTime(t_preplan));
    trajectory.setInitState(init_state);
    Eigen::Vector3d p = init_state.states[0].p;
    //ROS_DEBUG_STREAM("current position : "<<p.transpose());
    Eigen::Vector3d direction = (goal_o - p).normalized();
    Eigen::Vector3d sub_goal = p + direction * v_des * 1.0;
    
    FlatState end_state;
    end_state.pos_order = 0;
    end_state.states.resize(1);
    end_state.states[0].p = sub_goal;
    //ROS_INFO_STREAM("sub_goal : "<<sub_goal.transpose());
    trajectory.setEndState(end_state);
    trajectory.computeCoeffs(M);
    is_trajectory_set = true;
    Eigen::MatrixXd coeffs;
    if(print_trajectory){
        double start, end;
        trajectory.getTimeSpan(start, end);
        ROS_INFO_STREAM("Trajectory start at : "<<start);
        FlatState next_sp = trajectory.getFlatState(start+dt_control, 0.0, 2);
        ROS_INFO_STREAM("next setpoint-> p: " << next_sp.states[0].p.transpose()<<" v: "<<next_sp.states[1].p.transpose()<<" yaw : "<<next_sp.states[0].yaw);
    }
    
}

bool MaptestWrapper::getTimeSpan(double& start, double& end){
    unique_lock<mutex> traj_lock(traj_path_mtx_);
    if(!is_trajectory_set) return false;
    trajectory.getTimeSpan(start, end);
    return true;
}

visQueryResult MaptestWrapper::visibilityQueryTest(const double& t_start, const double& t_end, const double& ivl){
    unique_lock<mutex> traj_lock(traj_path_mtx_);
    if(!is_trajectory_set){
        ROS_WARN("Visibility Query Test is available after trajectory set");
        return visQueryResult();
    }
    MavState m_state;
    SE3 T_wc;
    vector<FeatureInfo> f_vec;
    ros::Time query_start = ros::Time::now();
    int num_query = 0;
    for(double t = t_start; t < t_end; t += 0.1){
        f_vec.clear();
        FlatState flat_state = trajectory.getFlatState(t, last_valid_yaw, 3);
        MavStateFromFlatState(flat_state, m_state);
        SE3 T_wb = from_Rp(m_state.R, m_state.p);
        T_wc = T_wb*T_bc;
        fm_handle_->queryVisible(T_wc, f_vec, fov_rads, true);
        ++num_query;
        if((ros::Time::now() - query_start).toSec() > 0.2) break; // break if too much query ruins the whole process
    }
    return visQueryResult(T_wc, f_vec);
}

bool MaptestWrapper::getSetpoints(const double& t_start, const double& t_end, std::vector<mavros_msgs::PositionTarget>& sp_vec){
    unique_lock<mutex> traj_lock(traj_path_mtx_);
    if(!is_trajectory_set) return false;
    int i=0;
    for(double t = t_start + dt_control; t_start < t_end && i<sp_per_plan;t += dt_control){
        SetPoint sp;
        FlatState flat_state = trajectory.getFlatState(t, last_valid_yaw);
        sp.t = t;
        sp.p_yaw = flat_state.states[0];
        sp.v_yawr.p = flat_state.states[1].p;
        sp.a = flat_state.states[2].p;
        last_valid_yaw = sp.p_yaw.yaw;
        sp_vec.push_back(SetPointToPt(sp));
        ++i;
    }
    return true;
}
}

int main(int argc, char** argv){
    ros::init(argc, argv, "maptest_node");
    google::InitGoogleLogging(argv[0]); // args="--alsologtostderr"
    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InstallFailureSignalHandler();
    online_planner::MaptestWrapper map_test;
    ros::spin();
    return 1;
}