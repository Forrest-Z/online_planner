#ifndef WRAPPER_BASE_H_
#define WRAPPER_BASE_H_

#include <global_planner/global_planner.h>
#include <local_planner/local_planner.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <transform_utils/transform_utils.h>
#include <string>

//status
#define NOT_READY 0
#define PREFLIGHT 1
#define FLIGHT 2

// Describes basic threading for wrapper nodes
namespace online_planner{

class BaseWrapper{
public:
    BaseWrapper();
    ~BaseWrapper();
    virtual int run(){} //run the whole node
    virtual int run_map_and_viz(){}  //run mapping(subscription to sensor msgs) and publish visualization msgs when available
    virtual int run_global_planner(){} //run global planning in global planning thread
    virtual int run_local_planner(){} //run local planning in local planning thread
    virtual int run_setpoint_publisher(){} //run setpoint publisher in sp thread
protected:
    void setupRos(); // setup ros related stuffs.
    ros::NodeHandle nh_default_; // handles heavy message processing.
    ros::NodeHandle nh_custom_; // handles more lightweight, high priority message / services 
    
    //All ros related actors are defined under nh_custom_.
    //publisher
    ros::Publisher sp_pub;
    mavros_msgs::PositionTarget pos_sp;
    
    //subscriber
    ros::Subscriber stat_sub;
    void statCallback(const mavros_msgs::StateConstPtr& msg);
    ros::Subscriber odom_sub;
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    //client
    ros::ServiceClient arming_client, setmode_client;

    void initializeFlight(); // executed in setpoint publishing thread
    
    //variables
    transform_utils::SE3 T_ib_init; // initial pose in  "interface frame"(user defined frame)
    transform_utils::SE3 T_ai;      // Airsim frame to interface frame
    std::vector<transform_utils::SE3> T_ab_init_vec;    //used to compute initial relative transform : T_ai
    std::string world_frame_name;
    mavros_msgs::State mav_stat;
    geometry_msgs::Pose curr_pose;
    nav_msgs::Odometry curr_odom;
    pvaState curr_state;
    std::vector<SetPoint> current_best_trajectory; //whole trajectory

    //loaded from ros
    int type_mask; //type_mask to use in planning(for initial takeoff sequence, use takeoff specific mask)
    bool verbose; 
    double sp_publishing_rate; //setpoint publishing rate
    double local_planning_rate; // local planning rate
    double global_planning_rate; // global planning rate
    int sp_per_plan; //how many setpoints are published per local trajectory plan?
    
    //status
    bool transform_stabilized;  //transformation confirmed
    bool status_okay;           //offboard & armed
    bool flight_ready;         //takeoff_ready
};

}

#endif