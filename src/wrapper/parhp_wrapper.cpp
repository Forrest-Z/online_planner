#include <wrapper/parhp_wrapper.h>
#include <utils/ros_utils.h>

using namespace std;
using namespace traj_lib;
//extern shared_ptr<ros::AsyncSpinner> custom_spinner;

namespace online_planner{
PaRhpWrapper::PaRhpWrapper(){
    mp_planner_ = new MpPlanner(loadMpPlannerParam(nh_custom_));
    ot_handle_ = std::make_shared<OctomapHandler>(loadOctomapHandlerParam(nh_custom_));
    fm_handle_ = std::make_shared<FeatureMapHandler>(loadFmParam(nh_custom_));
    fm_handle_->setOctHandle(ot_handle_);
    mp_planner_->setMapPtrs(ot_handle_, fm_handle_);
    tictoc_localplanning = Timer(string("- Local planning timer(per plan)"));

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

    //verbosity
    nh_default_.param("ot_verbose", ot_verbose_, true);
    nh_default_.param("fm_verbose", fm_verbose_, true);

    //define actors in nh_default_
    depth_img_sub = nh_default_.subscribe("/camera/depth/image_raw", 1, &PaRhpWrapper::depthImgCallback, this);
    depth_cam_info_sub = nh_default_.subscribe("/camera/depth/camera_info", 5, &PaRhpWrapper::depthcamInfoCallback, this);
    visualize_timer = nh_default_.createTimer(ros::Duration(1.0), &PaRhpWrapper::visualizeCallback, this);
    octomap_pub = nh_default_.advertise<octomap_msgs::Octomap>("octomap", 1);

    //TODO visualization actors

    //define actors in nh_private_
    if(is_mavros) global_plan_timer = nh_custom_.createTimer(ros::Duration(dt_global_planning), &PaRhpWrapper::mavrosGlobalCallback, this);
    else global_plan_timer = nh_custom_.createTimer(ros::Duration(dt_global_planning), &PaRhpWrapper::airsimGlobalCallback, this);
    //local plan thread manually starts after takeoff
    local_plan_timer = nh_custom_.createTimer(ros::Duration(dt_local_planning), &PaRhpWrapper::localPlannerCallback, this, false, false);
    
    custom_spinner->start();
}

PaRhpWrapper::~PaRhpWrapper(){
    custom_spinner->stop();
}

void PaRhpWrapper::depthImgCallback(const sensor_msgs::ImageConstPtr& img_msg){
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

void PaRhpWrapper::depthcamInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg){
    ot_handle_->setDepthcamModel(*cam_info_msg);
}

void PaRhpWrapper::mavrosGlobalCallback(const ros::TimerEvent& e){
    //TODO : implement
    return;
}

void PaRhpWrapper::airsimGlobalCallback(const ros::TimerEvent& e){
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
            if(transform_stabilized) mp_planner_->setGoal(goal_o);
            airsimHoverSingleIter();
            break;
        }
        case Status::FLIGHT:
            if(!local_plan_timer.hasStarted()) local_plan_timer.start();
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

void PaRhpWrapper::localPlannerCallback(const ros::TimerEvent& e){
    unique_lock<mutex> lock_(t_mtx_);
    tictoc_localplanning.tic();
    vector<SetPoint> sp_vec;
    ros::Time t_preplan = ros::Time::now();
    mp_planner_->reset((t_preplan - reference_time).toSec());
    mp_planner_->setInitState(getInitState()); //current best trajectory. gets state_mtx_ inside
    mp_planner_->planTrajectory(vector<globalPlan>());
    mp_planner_-> findBestMp();
    unique_lock<mutex> lock2(traj_path_mtx_);
    current_best_trajectory = mp_planner_-> findBestMp();
    double t_now = (ros::Time::now() - reference_time).toSec(); //time after planning
    mp_planner_->getNextSetpoints(t_now, dt_control, sp_vec, sp_per_plan);
    if(!is_mavros){
        size_t sp_num = sp_vec.size();
        airsim_controller::PositionTargets pts;
        pts.setpoints.resize(sp_num);
        for(size_t i = 0; i < sp_num; ++i){
            pts.setpoints[i] = SetPointToPt(sp_vec[i]);
        }
        sp_pub.publish(pts);
    }
    else{
        //what?
    }
    lock2.unlock();
    tictoc_localplanning.toc();
    lock_.unlock();
}

FlatState PaRhpWrapper::getInitState(){
    traj_lib::FlatState flat_state = getCurrFlatState();
    //modify flat_state based on constant velocity
    ros::Time now = ros::Time::now();
    double dt_delay = (now - t_last_odom_input).toSec();
    flat_state.states[0].p += flat_state.states[1].p * dt_delay; 
    unique_lock<mutex> lock2(traj_path_mtx_);
    if(current_best_trajectory == nullptr) return flat_state; 
    else{
        //compensate for time delay
        auto next_sp = mp_planner_->
            getFlatState(dynamic_cast<MinJerkPolyTraj*>(current_best_trajectory), (now - reference_time).toSec()+0.1);
        flat_state.states[0].p = 0.5*next_sp.states[0].p + 0.5*flat_state.states[0].p;
        flat_state.states[1].p = 0.5*next_sp.states[1].p + 0.5*flat_state.states[1].p;
        flat_state.states[2].p = next_sp.states[2].p;
        flat_state.states[0].yaw = 0.5*next_sp.states[0].yaw + 0.5*flat_state.states[0].yaw;
    }
    lock2.unlock();
}

void PaRhpWrapper::visualizeCallback(const ros::TimerEvent& e){
    //first, octomap
    if(ot_verbose_) ot_handle_->printTimers(true, true); // ot->lock_sub, lock_pub
    if(fm_verbose_) fm_handle_->printTimers(true, true); // fm->lock
    unique_lock<mutex> lock_(t_mtx_, defer_lock); //t_mtx_ lock
    if(lock_.try_lock()){
        tictoc_localplanning.print(false, true);
        lock_.unlock();
    }
    octomap_msgs::Octomap oct_msg;
    oct_msg.header.stamp = ros::Time::now();
    oct_msg.header.frame_id = world_frame_name;
    ot_handle_->getOctomapMsg(oct_msg);
    octomap_pub.publish(oct_msg);
}
}//namespace online planner


int main(int argc, char** argv){
    ros::init(argc, argv, "parhp_node");
    online_planner::PaRhpWrapper parhp;
    ros::spin();
}
