#include <wrapper/maptest_wrapper.h>
#include <utils/ros_utils.h>

using namespace traj_lib;
using namespace std;

namespace online_planner{
MaptestWrapper::MaptestWrapper(){
    ot_handle_ = std::make_shared<OctomapHandler>(loadOctomapHandlerParam(nh_custom_));
    fm_handle_ = std::make_shared<FeatureMapHandler>(loadFmParam(nh_custom_));
    fm_handle_->setOctHandle(ot_handle_);
    trajectory.setNumAxis(3);
    trajectory.setDuration(1.0);
    trajectory.reset(0.0);
    M.setZero(1, 3);
    M<<10.0, -20.0, 20.0;
    last_valid_yaw = 0.0;
    
    //params
    Eigen::Vector3d t_bc; Eigen::Quaterniond q_bc;
    t_bc.x() = nh_default_.param("T_bc_x", 0.3);
    t_bc.y() = nh_default_.param("T_bc_y", 0.0);
    t_bc.z() = nh_default_.param("T_bc_z", 0.1);
    q_bc.x() = nh_default_.param("T_bc_qx", 0.5);
    q_bc.y() = nh_default_.param("T_bc_qy", -0.5);
    q_bc.z() = nh_default_.param("T_bc_qz", 0.5);
    q_bc.w() = nh_default_.param("T_bc_qw", -0.5);
    T_bc = transform_utils::utils::from_vec3_quat(t_bc, q_bc);
    nh_default_.param("plan_delay", t_plan_delay, 0.01);
    nh_default_.param("v_des", v_des, 1.5);
    double h_fov, v_fov;
    nh_default_.param("h_fov", h_fov, 90.0);
    nh_default_.param("v_fov", v_fov, 73.74);
    double h_fov_rad = h_fov * M_PI / 180.0;
    double v_fov_rad = v_fov * M_PI / 180.0;
    fov_rads = make_pair(h_fov_rad, v_fov_rad);

    //verbosity
    nh_default_.param("ot_verbose", ot_verbose_, true);
    nh_default_.param("fm_verbose", fm_verbose_, true);

    //define actors in nh_default_
    depth_img_sub = nh_default_.subscribe("/camera/depth/image_raw", 1, &MaptestWrapper::depthImgCallback, this);
    depth_cam_info_sub = nh_default_.subscribe("/camera/depth/camera_info", 5, &MaptestWrapper::depthcamInfoCallback, this);
    visualize_timer = nh_default_.createTimer(ros::Duration(1.0), &MaptestWrapper::visualizeCallback, this);
    
    octomap_pub = nh_default_.advertise<octomap_msgs::Octomap>("octomap", 1);
    fm_pub = nh_default_.advertise<sensor_msgs::PointCloud2>("feature_map", 1);

    //define actors in nh_private_
    global_plan_timer = nh_custom_.createTimer(ros::Duration(dt_global_planning), &MaptestWrapper::airsimGlobalCallback, this);
    local_plan_timer = nh_custom_.createTimer(ros::Duration(dt_local_planning), &MaptestWrapper::localPlannerCallback, this, false, false);

    custom_spinner->start();
}

void MaptestWrapper::depthImgCallback(const sensor_msgs::ImageConstPtr& img_msg){
    if(!(ot_handle_->isCamModelSet())) return;
    if(!transformStabilized()) return;
    MavState mav_state = getCurrState();
    transform_utils::SE3 T_wb = transform_utils::from_Rp(mav_state.R, mav_state.p);
    transform_utils::SE3 T_wc = T_wb * T_bc;
    octomath::Pose6D T_wc_oct = SE3tooctPose(T_wc);

    ot_handle_->insertPointcloud(img_msg, T_wc_oct);
    pcl::PointCloud<pcl::PointXYZI> changed = ot_handle_->trackChanges();
    ot_handle_->insertUpdate(changed);
}

void MaptestWrapper::depthcamInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg){
    ot_handle_->setDepthcamModel(*cam_info_msg);
}

void MaptestWrapper::airsimGlobalCallback(const ros::TimerEvent& e){
    switch(status_){
        case Status::ODOM_NOT_SET:
            ROS_WARN_ONCE("Odom not set. waiting for odometry info...");
            airsimVioInitSingleIter();
            break;
        case Status::ODOM_INIT:
            airsimVioInitSingleIter();
            break;
        case Status::TAKEOFF_SEQUENCE:
            airsimTakeoffSingleIter();
            break;
        case Status::INIT_HOVER:{
            airsimHoverSingleIter();
            break;
        }
        case Status::FLIGHT:
            if(!local_plan_timer.hasStarted()){
                PolyTraj::Param p;
                p.degree = 5;
                p.duration = 1.0;
                p.num_axis = 3;
                p.start_time = getSimTime(ros::Time::now()); 
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
    Eigen::Vector3d rpy = transform_utils::utils::R2rpy(curr_state.R);
    unique_lock<mutex> traj_lock(traj_path_mtx_);
    last_valid_yaw = curr_flat_state.states[0].yaw; 
    trajectory.reset(getSimTime(t_preplan));
    trajectory.setInitState(getInitState());
    computePlan();
    double t_start, t_end;
    trajectory.getTimeSpan(t_start, t_end);
    t_start = getSimTime(t_preplan) + t_plan_delay;
    int i=0;
    for(double t = t_start + dt_control; t_start < t_end && i<sp_per_plan;t += dt_control){
        SetPoint sp;
        FlatState flat_state = trajectory.getFlatState(t, last_valid_yaw);
        sp.t = t;
        sp.p_yaw = flat_state.states[0];
        sp.v_yawr.p = flat_state.states[1].p;
        sp.a = flat_state.states[2].p;
        last_valid_yaw = sp.p_yaw.yaw;
        traj.header.frame_id = world_frame_name;
        traj.header.stamp = ros::Time::now();
        traj.setpoints.push_back(SetPointToPt(sp));
        ++i;
    }
    traj_lock.unlock();
}

void MaptestWrapper::visualizeCallback(const ros::TimerEvent& e){
    //do some featuremap visibility query along the trajectory to check query time
    double t_start, t_end;
    unique_lock<mutex> traj_lock(traj_path_mtx_);
    trajectory.getTimeSpan(t_start, t_end);
    MavState m_state;
    transform_utils::SE3 T_wc;
    vector<FeatureInfo> f_vec;
    ros::Time query_start = ros::Time::now();
    int num_query = 0;
    for(double t = t_start; t < t_end; t += 0.1){
        FlatState flat_state = trajectory.getFlatState(t, last_valid_yaw, 3);
        MavStateFromFlatState(flat_state, m_state);
        transform_utils::SE3 T_wb = transform_utils::from_Rp(m_state.R, m_state.p);
        T_wc = T_wb*T_bc;
        fm_handle_->queryVisible(T_wc,  f_vec, fov_rads, true);
        ++num_query;
        if((ros::Time::now() - query_start).toSec() > 0.2) break; // break if too much query ruins the whole process
    }
    traj_lock.unlock();
    if(ot_verbose_) ot_handle_->printTimers(true, true); // ot->lock_sub, lock_pub
    if(fm_verbose_) fm_handle_->printTimers(true, true); // fm->lock
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
    fm_pub.publish(pcd_msg);
}

FlatState MaptestWrapper::getInitState(){
    traj_lib::FlatState flat_state = getCurrFlatState();
    //modify flat_state based on constant velocity and time delay
    ros::Time now = ros::Time::now();
    double t_from_last_odom = (now - t_last_odom_input).toSec();
    unique_lock<mutex> traj_lock(traj_path_mtx_);
    if(current_best_trajectory == nullptr) return flat_state;
    auto next_sp = dynamic_cast<MinJerkPolyTraj*>(current_best_trajectory)->getFlatState(getSimTime(now) + t_plan_delay);
    flat_state.states[0].p += flat_state.states[1].p * t_from_last_odom + next_sp.states[2].p * (t_from_last_odom) * (t_from_last_odom)/2;
    flat_state.states[1].p += next_sp.states[2].p * t_from_last_odom;
    flat_state.states[2].p = next_sp.states[2].p;
    flat_state.states[0].yaw = flat_state.states[0].yaw;
    traj_lock.unlock();
}

void MaptestWrapper::computePlan(){
    //compute subgoal
    Eigen::Vector3d p = curr_state.p;
    ROS_DEBUG_STREAM("current position : "<<p.transpose());
    Eigen::Vector3d direction = (goal_o - p).normalized();
    Eigen::Vector3d sub_goal = p + direction * v_des * 1.0;
    
    FlatState end_state;
    end_state.pos_order = 0;
    end_state.states.resize(1);
    end_state.states[0].p = sub_goal;
    unique_lock<mutex> traj_lock(traj_path_mtx_);
    trajectory.setEndState(end_state);
    trajectory.computeCoeffs(M);
    Eigen::MatrixXd coeffs;
    trajectory.getCoeffsMatrix(coeffs);
    ROS_DEBUG_STREAM("coeffs : "<<coeffs);
    traj_lock.unlock();

}

}

int main(int argc, char** argv){
    ros::init(argc, argv, "maptest_node");
    online_planner::MaptestWrapper map_test;
    ros::spin();
    return 1;
}