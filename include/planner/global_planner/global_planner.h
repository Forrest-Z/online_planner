#ifndef ONLINE_GLOBAL_PLANNER_H_
#define ONLINE_GLOBAL_PLANNER_H_

#include <traj_lib/MavState.h>
#include <memory>
#include <vector>
#include <mutex>

namespace online_planner{

struct globalPlan{
    std::vector<Eigen::Vector3d> waypoints;
};

using globalPlan = struct globalPlan;

//Interface Class
class GlobalPlannerBase{
public:
    virtual ~GlobalPlannerBase(){}
    virtual void reset(double t) = 0;
    virtual void setInitPoint(traj_lib::FlatState p_init) = 0;
    virtual std::vector<globalPlan> findGlobalPath() = 0;
    virtual void setGoal(Eigen::Vector3d goal)=0;
};

class DummyGlobalPlanner : public GlobalPlannerBase{
public:
    struct Param{
        bool dummy;
    };
    DummyGlobalPlanner(Param param){}
    ~DummyGlobalPlanner(){}
    void reset(double t){return;}
    void setInitPoint(traj_lib::FlatState x_init){x_init_ = x_init;}
    std::vector<globalPlan> findGlobalPath(){
        std::unique_lock<std::mutex> lock(gplan_mtx_);
        std::vector<globalPlan> plans;
        globalPlan plan;
        plan.waypoints.push_back(x_init_.states[0].p);
        plan.waypoints.push_back(goal_);
        plans.push_back(plan);
        return plans;
    }
    void setGoal(Eigen::Vector3d goal){goal_ = goal;}
protected:
    Eigen::Vector3d goal_;
    traj_lib::FlatState x_init_;
    std::mutex gplan_mtx_;
};


} //online_planner
#endif