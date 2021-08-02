#include <wrapper/parhp_wrapper.h>
#include <utils/ros_utils.h>

using namespace std;

//extern shared_ptr<ros::AsyncSpinner> custom_spinner;

namespace online_planner{
PaRhpWrapper::PaRhpWrapper(){
    mp_planner_ = new MpPlanner(loadMpPlannerParam(nh_custom_));
    ot_handle_ = std::make_shared<OctomapHandler>(loadOctomapHandlerParam(nh_custom_));
    mp_planner_->setOctomapPtr(ot_handle_);

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
    local_plan_timer = nh_custom_.createTimer(ros::Duration(dt_local_planning), &PaRhpWrapper::localPlannerCallback, this, false);
    
    custom_spinner->start();
}

PaRhpWrapper::~PaRhpWrapper(){
    custom_spinner->stop();
}

void PaRhpWrapper::depthImgCallback(const sensor_msgs::ImageConstPtr& img_msg){
    if(!(ot_handle_->isCamModelSet())) return;
    unique_lock<mutex> lock2(state_mtx_);
    if(!transform_stabilized) return;
    geometry_msgs::Pose pose = curr_odom.pose.pose;
    lock2.unlock();
    transform_utils::SE3 T_wb = transform_utils::utils::from_vec3_quat(pose.position, pose.orientation);
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
            break;
        case Status::ODOM_INIT:
            airsimVioInitSingleIter();
            break;
        case Status::TAKEOFF_SEQUENCE:
            airsimTakeoffSingleIter();
            break;
        case Status::INIT_HOVER:{
            mp_planner_->setGoal(goal_o);
            airsimHoverSingleIter();
            break;
        }
        case Status::FLIGHT:
            if(!local_plan_timer.hasStarted()) local_plan_timer.start();
            break;
        default:
            if(local_plan_timer.hasStarted()) local_plan_timer.stop();
            break;
    }
    return;
}

void PaRhpWrapper::localPlannerCallback(const ros::TimerEvent& e){
    ros::Time now = ros::Time::now();
    mp_planner_->reset((now - reference_time).toSec());
    mp_planner_->setInitState(getInitState());
    mp_planner_->planTrajectory(vector<globalPlan>());
    mp_planner_->findBestMp();
    unique_lock<mutex> lock2(traj_path_mtx_);
    mp_planner_->getNextSetpoints(dt_control, current_best_trajectory, sp_per_plan);
    if(!is_mavros){
        airsim_controller::PositionTargets pts;
        for(int i=0; i < sp_per_plan; ++i){
            pts.setpoints[i] = flatStateToPt(current_best_trajectory[i]);
        }
        sp_pub.publish(pts);
    }
    else{
        //what?
    }
    lock2.unlock();
}

void PaRhpWrapper::visualizeCallback(const ros::TimerEvent& e){
    //first, octomap
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
