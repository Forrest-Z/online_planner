#include <wrapper/base_wrapper.h>
#include <memory>

namespace online_planner{

std::shared_ptr<ros::AsyncSpinner> custom_spinner;

BaseWrapper::BaseWrapper(){
    nh_default_ = ros::NodeHandle("~"); //private
    nh_custom_ = ros::NodeHandle("~");
    nh_custom_.setCallbackQueue(&custom_queue);
    custom_spinner.reset(new ros::AsyncSpinner(1, &custom_queue));
    
    //ros related
    sp_pub = nh_custom_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    stat_sub = nh_custom_.subscribe("/mavros/state", 10, &BaseWrapper::statCallback, this);
    odom_sub = nh_custom_.subscribe("/mavros/local_position/odom", 10, &BaseWrapper::odomCallback, this);
    
    arming_client = nh_custom_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh_custom_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    custom_spinner->start();
}

BaseWrapper::~BaseWrapper(){
    custom_spinner->stop();
    custom_spinner.reset();
}


}