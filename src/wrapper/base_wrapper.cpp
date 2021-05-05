#include <wrapper/base_wrapper.h>

namespace online_planner{

BaseWrapper::BaseWrapper(){
    nh_default_ = ros::NodeHandle("~"); //private
    nh_custom_ = ros::NodeHandle("~");
    nh_custom_.setCallbackQueue(&custom_queue);
    setupRos();
    run();
}

BaseWrapper::~BaseWrapper(){


}

void BaseWrapper::setupRos(){

}

int BaseWrapper::run(){
    //initialize threads : might not work as intended, due to overriding issue
    setpoint_publishing_thread = new std::thread(&run_setpoint_publisher, this); //This might not work(not correctly call the override function.)
    global_planning_thread = new std::thread(&run_global_planner, this);
    local_planning_thread = new std::thread(&run_local_planner, this);
    map_and_viz_thread = new std::thread(&run_map_and_viz, this);
}

}