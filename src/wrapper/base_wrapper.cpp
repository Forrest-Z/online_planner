#include <wrapper/base_wrapper.h>
#include <memory>

namespace online_planner{

using namespace std;

using namespace transform_utils;

BaseWrapper::BaseWrapper():transform_stabilized(false){
    nh_default_ = ros::NodeHandle("~"); //private
    nh_custom_ = ros::NodeHandle("~");
    nh_custom_.setCallbackQueue(&custom_queue);
    custom_spinner.reset(new ros::AsyncSpinner(2, &custom_queue)); //for global planning, local planning each
    
    sp_per_plan = nh_custom_.param("sp_per_plan", 5);
    dt_global_planning = nh_custom_.param("dt_global_planning", 0.5);
    dt_local_planning = nh_custom_.param("dt_local_planning", 0.1);
    dt_control = nh_custom_.param("dt_control", 0.025);
    verbose = nh_custom_.param("verbose", false);
    takeoff_height = nh_custom_.param("takeoff_height", 1.5);
    takeoff_speed = nh_custom_.param("takeoff_speed", 0.25);

    is_odom_ned = nh_custom_.param("is_odom_ned", false);
    is_mavros = nh_custom_.param("is_mavros", true);
    
    //initial pose in interface frame
    double x, y, z, qx, qy, qz, qw;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    p.x() = nh_custom_.param("T_ib_x", 0.0);
    p.y() = nh_custom_.param("T_ib_y", 0.0);
    p.z() = nh_custom_.param("T_ib_z", 0.0);
    q.x() = nh_custom_.param("T_ib_qx", 0.0);
    q.y() = nh_custom_.param("T_ib_qy", 0.0);
    q.z() = nh_custom_.param("T_ib_qz", 0.0);
    q.w() = nh_custom_.param("T_ib_qw", 1.0);
    T_ib_init = utils::from_vec3_quat(p, q);

    goal_i.x() = nh_custom_.param("goal_x", 20.0);
    goal_i.y() = nh_custom_.param("goal_y", 0.0);
    goal_i.z() = nh_custom_.param("goal_z", 5.0);

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
        vel_cmd_.twist.linear.x = vel_cmd_.twist.linear.y = 0.0;
    }
    mavros_okay = !is_mavros;
    status_ = Status::ODOM_NOT_SET;
    reference_time = ros::Time::now(); //start
    //custom_spinner->start();
}


void BaseWrapper::statCallback(const mavros_msgs::StateConstPtr& stat_msg){
    unique_lock<mutex> lock(state_mtx_);
    mav_stat = *stat_msg;
    if(mav_stat.armed && mav_stat.connected && mav_stat.mode == "OFFBOARD")
        mavros_okay = true;
    if(status_ == Status::ODOM_INIT && mavros_okay){
        status_ = Status::TAKEOFF_SEQUENCE; 
        // this is okay cause in real life excitation will be done manually,and this callback is only activated under mavros setting
        // in airsim, alternative 
    }
}

void BaseWrapper::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg){
    unique_lock<mutex> lock(state_mtx_);
    if(status_ == Status::ODOM_NOT_SET){
        status_ = Status::ODOM_INIT;
        ROS_WARN("Got odometry info");
    }
    curr_odom = *odom_msg;
    auto pos = curr_odom.pose.pose.position;
    auto ori = curr_odom.pose.pose.orientation;
    if(is_odom_ned){
        curr_odom.pose.pose.position.y = -odom_msg->pose.pose.position.y;
        curr_odom.pose.pose.position.z = -odom_msg->pose.pose.position.z;
        curr_odom.pose.pose.orientation.y = -odom_msg->pose.pose.orientation.y;
        curr_odom.pose.pose.orientation.z = -odom_msg->pose.pose.orientation.z;
    }
    curr_state.p = geovec_to_eigen<geometry_msgs::Point>(curr_odom.pose.pose.position);
    curr_state.R = utils::gquat2rot(curr_odom.pose.pose.orientation);
    curr_state.v_w = geovec_to_eigen<geometry_msgs::Vector3>(curr_odom.twist.twist.linear);
    curr_flat_state.states.resize(3);
    curr_flat_state.states[0].p = curr_state.p;
    curr_flat_state.states[1].p = curr_state.v_w;
    curr_flat_state.states[2].p.setZero();
    Eigen::Vector3d rpy = utils::R2rpy(curr_state.R);
    curr_flat_state.states[0].yaw = rpy(2);
    lock.unlock();

    //unique_lock<mutex> lock2(traj_path_mtx_);
    //curr_flat_state.states[2].p = current_best_trajectory[0].states[2].p;
    //lock2.unlock();

    if(status_ == Status::TAKEOFF_SEQUENCE && !transform_stabilized){
        SE3 T_ab = utils::from_vec3_quat(pos, ori);
        T_oi = T_ab*T_ib_init.inverse();
        ROS_WARN_STREAM("odometry - Interface frame transform stabilized");
        goal_o = T_oi.rotation() * goal_i + T_oi.translation();
        ROS_WARN_STREAM("Goal position in Odom frame : "<<goal_o.x()<<", "<<goal_o.y()<<", "<<goal_o.z());
        transform_stabilized = true;
    }
}

void BaseWrapper::airsimVioInitSingleIter(){
    if(is_mavros){
        ROS_INFO("airsimVioInitSingleIter : Currently assuming non-mavros simulation only. For Real experiment, Please do it manually");
        return;
    }
    if(status_ >= Status::TAKEOFF_SEQUENCE){
        return;
    }
    if(n_executed < n_bound / 2){
        vel_cmd_.twist.linear.z = - takeoff_speed * 2.0; //these assume downward z axis
    }
    else if(n_executed == n_bound / 2){
        vel_cmd_.twist.linear.z = 0.0;
    }
    else{
        vel_cmd_.twist.linear.z = takeoff_speed * 2.0;
    }
    if(n_executed > n_bound){
        status_ = Status::TAKEOFF_SEQUENCE;
        n_executed = 0;
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
        return;
    }
    airsim_vel_pub.publish(vel_cmd_);
    return;
}

void BaseWrapper::airsimHoverSingleIter(){
    if(n_executed < n_hover){
        vel_cmd_.twist.linear.z = 0.0;
        airsim_vel_pub.publish(vel_cmd_);
        ++n_executed;
    }
    else{
        n_executed = 0;
        status_ = Status::FLIGHT;
        return;
    }
    airsim_vel_pub.publish(vel_cmd_);
}

BaseWrapper::~BaseWrapper(){
    custom_spinner->stop();
    custom_spinner.reset();
}

traj_lib::FlatState BaseWrapper::getInitState(){
    unique_lock<mutex> lock(state_mtx_);
    traj_lib::FlatState flat_state = curr_flat_state;
    lock.unlock();
    unique_lock<mutex> lock2(traj_path_mtx_);
    if(current_best_trajectory.size() == 0) return flat_state; 
    else{
        auto next_sp = current_best_trajectory[0].states;
        flat_state.states[0].p = 0.8*next_sp[0].p + 0.2*flat_state.states[0].p;
        flat_state.states[1].p = 0.8*next_sp[1].p + 0.2*flat_state.states[1].p;
        flat_state.states[2].p = next_sp[2].p;
        flat_state.states[0].yaw = 0.8*next_sp[0].yaw + 0.2*flat_state.states[0].yaw;
    }
    lock2.unlock();
}

mavros_msgs::PositionTarget BaseWrapper::flatStateToPt(const traj_lib::FlatState state){
    mavros_msgs::PositionTarget pt;
    pt.header.frame_id = world_frame_name;
    pt.type_mask = IG_YAWR;
    pt.position = transform_utils::eigen_to_geovec<geometry_msgs::Point>(state.states[0].p);
    pt.velocity = transform_utils::eigen_to_geovec<geometry_msgs::Vector3>(state.states[1].p);
    pt.acceleration_or_force = transform_utils::eigen_to_geovec<geometry_msgs::Vector3>(state.states[2].p);
    pt.yaw = state.states[0].yaw;
    return pt;
}
}