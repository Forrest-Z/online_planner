#include <wrapper/base_wrapper.h>
#include <memory>

using namespace std;
using namespace traj_lib;
using namespace transform_utils;

namespace online_planner{

BaseWrapper::BaseWrapper():transform_stabilized(false), current_best_trajectory(nullptr){
    nh_default_ = ros::NodeHandle("~"); //private
    nh_custom_ = ros::NodeHandle("~");
    nh_custom_.setCallbackQueue(&custom_queue);
    custom_spinner.reset(new ros::AsyncSpinner(1, &custom_queue)); //for global planning, local planning each
    
    sp_per_plan = nh_custom_.param("sp_per_plan", 5);
    dt_global_planning = nh_custom_.param("dt_global_planning", 0.5);
    dt_local_planning = nh_custom_.param("dt_local_planning", 0.1);
    dt_control = nh_custom_.param("dt_control", 0.025);
    takeoff_height = nh_custom_.param("takeoff_height", 1.5);
    takeoff_speed = nh_custom_.param("takeoff_speed", 0.25);
    nh_custom_.param("world_frame_name", world_frame_name, std::string("world"));

    is_odom_ned = nh_custom_.param("is_odom_ned", false);
    is_mavros = nh_custom_.param("is_mavros", true);
    
    //initial pose in interface frame
    T_bi = loadTransformFromRos(std::string("T_bi"), nh_custom_);
    T_ub_init = loadTransformFromRos(std::string("T_ub_init"), nh_custom_);

    goal_u.x() = nh_custom_.param("goal_x", 20.0);
    goal_u.y() = nh_custom_.param("goal_y", 0.0);
    goal_u.z() = nh_custom_.param("goal_z", 5.0);

    odom_topic_name = nh_custom_.param("odom_topic_name", string("/mavros/local_position/odom"));
    odom_sub = nh_custom_.subscribe(odom_topic_name, 10, &BaseWrapper::odomCallback, this);

    if(is_mavros){
        sp_pub = nh_custom_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        stat_sub = nh_custom_.subscribe("/mavros/state", 10, &BaseWrapper::statCallback, this);
        arming_client = nh_custom_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client = nh_custom_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    }
    else{ //assume airsim
        sp_pub = nh_custom_.advertise<airsim_controller::PositionTargets>("reference_trajectory", 10);
        airsim_vel_pub = nh_custom_.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/SimpleFlight/vel_cmd_world_frame", 10);
        n_executed = 0;
        n_bound = static_cast<int>(takeoff_height / takeoff_speed / dt_control) + 1;
        n_hover = static_cast<int>(1.0 / dt_control);
        n_stable = n_bound/3;
        vel_cmd_.twist.linear.x = vel_cmd_.twist.linear.y = 0.0;
        control_stop_client = nh_custom_.serviceClient<std_srvs::Trigger>("/geometric_controller/stop");
    }
    mavros_okay = !is_mavros;
    status_ = Status::ODOM_NOT_SET;
    reference_time = ros::Time::now(); //start
    //custom_spinner->start();
}

FlatState BaseWrapper::getCurrFlatState(){
    unique_lock<mutex> state_lock(state_mtx_);
    return curr_flat_state;
}

MavState BaseWrapper::getCurrState(){
    unique_lock<mutex> state_lock(state_mtx_);
    return curr_state;
}

bool BaseWrapper::transformStabilized(){
    unique_lock<mutex> state_lock(state_mtx_);
    return transform_stabilized;
}

void BaseWrapper::statCallback(const mavros_msgs::StateConstPtr& stat_msg){
    unique_lock<mutex> lock(state_mtx_);
    mav_stat = *stat_msg;
    if(mav_stat.armed && mav_stat.connected && mav_stat.mode == "OFFBOARD")
        mavros_okay = true;
    if(status_ == Status::ODOM_INIT && mavros_okay){
        status_ = Status::TAKEOFF_SEQUENCE; 
        ROS_WARN_ONCE("[Status transition] : to TAKEOFF_SEQUENCE");
        // this is okay cause in real life excitation will be done manually,and this callback is only activated under mavros setting
        // in airsim, alternative 
    }
}

void BaseWrapper::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg){
    unique_lock<mutex> state_lock(state_mtx_);
    if(status_ == Status::ODOM_NOT_SET){
        status_ = Status::ODOM_INIT;
        ROS_WARN("Got odometry info");
        ROS_WARN_ONCE("[Status transition] : to ODOM_INIT");
    }
    imu_odom = *odom_msg; //Note that imu_odom is defined for imu frame
    t_last_odom_input = imu_odom.header.stamp;
    auto pos = imu_odom.pose.pose.position;
    auto ori = imu_odom.pose.pose.orientation;
    if(is_odom_ned){
        imu_odom.pose.pose.position.y = -odom_msg->pose.pose.position.y;
        imu_odom.pose.pose.position.z = -odom_msg->pose.pose.position.z;
        imu_odom.pose.pose.orientation.y = -odom_msg->pose.pose.orientation.y;
        imu_odom.pose.pose.orientation.z = -odom_msg->pose.pose.orientation.z;
    }
    Eigen::Vector3d p_oi = geovec_to_eigen<geometry_msgs::Point>(imu_odom.pose.pose.position);
    Eigen::Matrix3d R_oi = utils::gquat2rot(imu_odom.pose.pose.orientation);
    SE3 T_oi = from_Rp(R_oi, p_oi);
    SE3 T_ob = T_oi * T_bi.inverse(); //body frame
    curr_state.p = T_ob.translation();
    curr_state.R = T_ob.rotation();
    curr_state.v_w = geovec_to_eigen<geometry_msgs::Vector3>(imu_odom.twist.twist.linear); //TODO : does this need modification or is it descripted in world frame?
    curr_flat_state.states.resize(3);
    curr_flat_state.states[0].p = curr_state.p;
    curr_flat_state.states[1].p = curr_state.v_w;
    curr_flat_state.states[2].p.setZero();
    Eigen::Vector3d rpy = utils::R2rpy(curr_state.R);
    curr_flat_state.states[0].yaw = rpy(2);

    if(status_ == Status::TAKEOFF_SEQUENCE && !transform_stabilized){
        SE3 T_ou = T_ob*T_ub_init.inverse(); 
        ROS_WARN_STREAM("odometry - User frame transform stabilized : "<<endl<<T_ou.matrix());
        //goal_o = T_ou.rotation() * goal_u + T_ou.translation();
        goal_o = goal_u + T_ou.translation();
        ROS_WARN_STREAM("Goal position in Odom frame : "<<goal_o.x()<<", "<<goal_o.y()<<", "<<goal_o.z());
        transform_stabilized = true;
    }
    state_lock.unlock();
}

void BaseWrapper::airsimVioInitSingleIter(){
    if(is_mavros){
        ROS_INFO("airsimVioInitSingleIter : Currently assuming non-mavros simulation only. For Real experiment, Please do it manually");
        return;
    }
    if(status_ >= Status::TAKEOFF_SEQUENCE){
        return;
    }
    if(n_executed < n_bound / 3){
        vel_cmd_.twist.linear.z = - takeoff_speed * 1.0; //these assume downward z axis
    }
    else if(n_executed == n_bound / 3){
        vel_cmd_.twist.linear.z = 0.0;
    }
    else if(n_executed  < n_bound * 2/3){
        vel_cmd_.twist.linear.z = takeoff_speed * 1.0;
    }
    else if(n_executed < n_bound){
        vel_cmd_.twist.linear.z = 0.0;
    }
    if(n_executed > n_bound){
        status_ = Status::TAKEOFF_SEQUENCE;
        vel_cmd_.twist.linear.z = 0.0;
        n_executed = 0;
        ROS_WARN_ONCE("[Status transition] : to TAKEOFF_SEQUENCE");
        return;
    }
    airsim_vel_pub.publish(vel_cmd_);
    ++n_executed;
}

void BaseWrapper::airsimTakeoffSingleIter(){
    if(status_ != Status::TAKEOFF_SEQUENCE || is_mavros){
        return;
    }
    if(n_executed < n_bound){
        vel_cmd_.twist.linear.z = -takeoff_speed;
        ++n_executed;
    }
    else{
        n_executed = 0;
        status_ = Status::INIT_HOVER;
        ROS_WARN_ONCE("[Status transition] : to INIT_HOVER");
        return;
    }
    airsim_vel_pub.publish(vel_cmd_);
    return;
}

void BaseWrapper::airsimHoverSingleIter(){
    if(n_executed < n_hover){
        vel_cmd_.twist.linear.x = 0.0;
        vel_cmd_.twist.linear.y = 0.0;
        vel_cmd_.twist.linear.z = 0.0;
        airsim_vel_pub.publish(vel_cmd_);
        ++n_executed;
    }
    else{
        n_executed = 0;
        status_ = Status::FLIGHT;
        ROS_WARN_ONCE("[Status transition] : to FLIGHT");
        return;
    }
    airsim_vel_pub.publish(vel_cmd_);
}

BaseWrapper::~BaseWrapper(){
    custom_spinner->stop();
    custom_spinner.reset();
}

mavros_msgs::PositionTarget BaseWrapper::SetPointToPt(const SetPoint& state){
    mavros_msgs::PositionTarget pt;
    pt.header.frame_id = world_frame_name;
    pt.type_mask = IG_YAWR;
    pt.position = transform_utils::eigen_to_geovec<geometry_msgs::Point>(state.p_yaw.p);
    pt.velocity = transform_utils::eigen_to_geovec<geometry_msgs::Vector3>(state.v_yawr.p);
    pt.acceleration_or_force = transform_utils::eigen_to_geovec<geometry_msgs::Vector3>(state.a);
    pt.yaw = state.p_yaw.yaw;
    pt.header.stamp = reference_time + ros::Duration(state.t);
    pt.header.frame_id = world_frame_name;
    return pt;
}

void BaseWrapper::checkGoalReached(){
    unique_lock<mutex> lock(state_mtx_);
    Eigen::Vector3d curr_p = curr_state.p;
    Eigen::Vector3d goal = goal_o;
    lock.unlock();
    if((curr_p - goal).norm() < 0.4){
        control_stop_client.call(trig);
        status_ = Status::REACHED_GOAL;
        ROS_WARN_ONCE("[Status transition] : to REACHED_GOAL");
    }
    return;
}
}