#ifndef ONLINE_GLOBAL_PLANNER_H_
#define ONLINE_GLOBAL_PLANNER_H_

#include <planner/planner_common.h>
#include <memory>
#include <vector>

namespace online_planner{

struct globalPlan{
    std::vector<Eigen::Vector3d> waypoints;
};

using globalPlan = struct globalPlan;

//Interface Class
class GlobalPlannerBase{
public:
    virtual ~GlobalPlannerBase(){}
    virtual void reset() = 0;
    virtual void setInitPoint(pvaState p_init) = 0;
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
    void reset(){return;}
    void setInitPoint(pvaState x_init){x_init_ = x_init;}
    std::vector<globalPlan> findGlobalPath(){
        std::vector<globalPlan> plans;
        globalPlan plan;
        plan.waypoints.push_back(x_init_.position);
        plan.waypoints.push_back(goal_);
        plans.push_back(plan);
        return plans;
    }
    void setGoal(Eigen::Vector3d goal){goal_ = goal;}
private:
    Eigen::Vector3d goal_;
    pvaState x_init_;
};


} //online_planner
#endif