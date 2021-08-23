#ifndef ONLINE_LOCAL_PLANNER_H_
#define ONLINE_LOCAL_PLANNER_H_

#include <planner/global_planner/global_planner.h>

//Interface class of local planner
namespace online_planner{
class LocalPlannerBase{
public:
    virtual ~LocalPlannerBase(){}
    virtual void reset(double t) = 0; //t : next starting time
    virtual void setInitState(traj_lib::FlatState x_init)=0;
    virtual void planTrajectory(std::vector<globalPlan>) = 0; //ToDo : do something with global path plan
    virtual void getNextSetpoints(double t_now, double dt, std::vector<traj_lib::SetPoint>& sp_vec, int m) = 0;
    virtual void getTrajectoryForViz(double dt, std::vector<Eigen::Vector3d>& sp_vec)=0;
    virtual void setGoal(Eigen::Vector3d goal)=0;
};
}
#endif