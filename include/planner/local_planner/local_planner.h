#ifndef ONLINE_LOCAL_PLANNER_H_
#define ONLINE_LOCAL_PLANNER_H_

#include <global_planner/global_planner.h>

//Interface class of local planner
namespace online_planner{
class LocalPlannerBase{
public:
    virtual ~LocalPlannerBase(){}
    virtual void reset() = 0;
    virtual void setInitState(pvaState p_init)=0;
    virtual void planTrajectory(std::vector<globalPlan>) = 0; //ToDo : do something with global path plan
    virtual void getNextSetpoints(double dt, std::vector<SetPoint>& sp_vec, int m) = 0;
    virtual void getBestTrajectory(double dt, std::vector<Eigen::Vector3d>& sp_vec)=0;
    virtual void setGoal(Eigen::Vector3d goal)=0;
};
}
#endif