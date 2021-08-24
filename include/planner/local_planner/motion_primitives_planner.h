#ifndef MOTION_PRIMITIVE_PLANNER_H_
#define MOTION_PRIMITIVE_PLANNER_H_

#include <planner/global_planner/global_planner.h>
#include <planner/local_planner/local_planner.h>
#include <mapping/octomap_handler.h>
#include <mapping/featuremap_handler.h>
#include <traj_lib/MinJerkPolyTraj.h>
#include <traj_lib/MavState.h>
#include <Eigen/Dense>
#include <memory>

namespace online_planner{

class MpPlanner;

class MpEvaluator{
public:
    struct Param{
        double k_col;           // collision cost weight
        double k_per;           // perception cost weight
        double k_prox;          // goal cost weight
        double k_end;           // cost related to end point feasibility
        double d_critic;        // distance of attraction area near the goal 
        double safe_dist;
        int power_dist;      

        double dist_th;         //distance threshold of collision

        //Feasibility of the trajectory
        double v_max, a_max;

        int J; // number of sample poses
        //Camera parameters 
        double h_fov; 
        double v_fov;
    };
    MpEvaluator(Param param):k_col(param.k_col), k_per(param.k_per), k_prox(param.k_prox), k_end(param.k_end), 
                             safe_dist(param.safe_dist), d_critic(param.d_critic), dist_th(param.dist_th), power_dist(param.power_dist),
                              v_max(param.v_max), a_max(param.a_max), J(param.J), h_fov(param.h_fov), v_fov(param.v_fov){}
    void setGoal(Eigen::Vector3d g){goal = g;}
    void setMapPtrs(std::shared_ptr<OctomapHandler> ot_handle, std::shared_ptr<FeatureMapHandler> fm_handle);
    void setInitState(traj_lib::FlatState x_init);
    friend class MpPlanner;
private:
    std::shared_ptr<OctomapHandler> ot_handle_;
    std::shared_ptr<FeatureMapHandler> fm_handle_;
    MpPlanner* mp_planner_;
    double evaluateMp(traj_lib::MinJerkPolyTraj* mp);
    double computePerceptionCost(traj_lib::MinJerkPolyTraj* mp);
    double computeCollisionCost(traj_lib::MinJerkPolyTraj* mp);
    double computeGoalProxCost(traj_lib::MinJerkPolyTraj* mp);
    double computeEndpointCost(traj_lib::MinJerkPolyTraj* mp);
    //loaded from param
    double k_col;           
    double k_per;           
    double k_prox;
    double k_end;
    double safe_dist;           
    double d_critic;       
    int power_dist;        

    double v_max, a_max;
    double dist_th;         
    Eigen::Vector3d goal;
    traj_lib::FlatState x_init_;   //reset by mp_planner

    int J; 
    //Camera parameters 
    double h_fov; 
    double v_fov; 
};

class MpPlanner:public LocalPlannerBase{
public:
    struct Param{
        //Generation specific
        double L; //planning horizon length 
        double h_ang_max; // horizontal angle for planning
        double v_ang; // vertical angle. 0.0 -> just plan in x-y plane
        int h_res, v_res; // total number of motion primitives = h_res X v_res
        double v_des; //desired velocity
        double k_th;
        MpEvaluator::Param eval_param; //Evaluation specific 
    };
    MpPlanner(Param param);
    ~MpPlanner();
    //basis functions
    virtual void reset(double t) override;
    virtual void setInitState(traj_lib::FlatState x_init) override;
    virtual void setGoal(Eigen::Vector3d g) override{mp_eval_->setGoal(g);}
    virtual void planTrajectory(std::vector<globalPlan>) override;
    virtual void getNextSetpoints(double t_now, double t_query, std::vector<traj_lib::SetPoint>& sp_vec, int m) override;
    virtual void getTrajectoryForViz(double t_query, std::vector<Eigen::Vector3d>& pos_vec) override;
    std::vector<Eigen::Vector3d> computeMpEndpoints(traj_lib::FlatState x_init);
    traj_lib::MinJerkPolyTraj* findBestMp();

    void setMapPtrs(std::shared_ptr<OctomapHandler> ot_handle, std::shared_ptr<FeatureMapHandler> fm_handle){
        mp_eval_->setMapPtrs(ot_handle, fm_handle);
    }
    MpEvaluator* getMpEvalPtr(){return mp_eval_.get();}
    traj_lib::FlatState getFlatState(traj_lib::MinJerkPolyTraj* mp, double t);
    friend class MpEvaluator;
private:
    //member functions
    void generateMps();
    traj_lib::MinJerkPolyTraj prev_best_mp; //previously best motion primitive
    inline int linearMpIndex(int h_idx, int v_idx){ return v_idx * h_res + h_idx;}

    traj_lib::FlatState x_init_;
    traj_lib::MinJerkPolyTraj* mp_lib; //motion primitives
    std::unique_ptr<MpEvaluator> mp_eval_; //motion primitive evaluator
    //loaded from Param
    double L;
    double h_ang_max;
    double v_ang;
    int h_res, v_res, num_mps;
    double v_des;    
    double k_th;
    int best_idx;
    bool found_best;

    double curr_start_time;
};
}//online_pa_planner


#endif
