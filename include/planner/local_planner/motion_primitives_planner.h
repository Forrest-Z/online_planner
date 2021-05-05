#ifndef MOTION_PRIMITIVE_PLANNER_H_
#define MOTION_PRIMITIVE_PLANNER_H_

#include <planner/planner_common.h>
#include <planner/global_planner/global_planner.h>
#include <planner/local_planner/local_planner.h>
#include <mapping/octomap_handler.h>
#include <Eigen/Dense>
#include <memory>

namespace online_planner{
//Agnostic to global planner module : just sample end point based on FOV and resolution specified by the parameter
struct MotionPrimitive{
    Eigen::Matrix<double, DIM_PLANNING, MP_ORDER+1> coeffs;
    double Tf; //Duration of the motion primitives
    double yaw_init; //required for stabilizing yaw angle at first
    Eigen::Vector3d p_f; //end point

    Eigen::Vector3d operator()(double t, int order=0) const{
        if(t > Tf) return (*this)(Tf, order);
        else if(t < 0.0) return (*this)(0.0, order);
        if(order < 0) return (*this)(t, 0);
        else if(order > 5) return (*this)(t, 5);
        Eigen::Vector3d val = Eigen::Vector3d::Zero();
        double t_power = 1.0;
        for(int i=order; i<MP_ORDER + 1; ++i){
            val += coeffs.block<3, 1>(0, i) * t_power;
            t_power = t_power * t / (i - order + 1);
        }
        return val;
    }

    pvaState state(double t) const{
        if(t > Tf) return this->state(Tf);
        if(t < 0.0) return this->state(0.0);
        pvaState x;
        x.position = (*this)(t, 0);
        Eigen::Vector3d vel = (*this)(t, 1);
        x.accel = (*this)(t, 2);
        x.velocity = vel;
        if(vel.norm() < 0.1) x.yaw = yaw_init; 
        else if(fabs(vel.y()) < 0.005 ){
            if(vel.x() < 0.0) x.yaw = M_PI;
            else x.yaw = 0.0;
        }
        else{
            x.yaw = atan2(vel.y(), vel.x()); //속력이 0 근처에서 jittering 하는 상황에는 주의할 것.
        }
        return x;
    }

    void computeCoeffs(pvaState x_init){
        auto p0 = x_init.position;
        auto v0 = x_init.velocity;
        auto a0 = x_init.accel;
        Eigen::RowVector3d Vmat = 1/pow(Tf, 5) * Eigen::RowVector3d(10*Tf*Tf, -20*Tf, 20.0);
        coeffs.col(0) = p0;
        coeffs.col(1) = v0;
        coeffs.col(2) = a0;
        Eigen::Vector3d p_del = p_f - p0 - v0*Tf - 0.5*a0*Tf*Tf; 
        coeffs.block<3, 3>(0, 3) = p_del * Vmat;
    }
};

using MotionPrimitive = struct MotionPrimitive;

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
    void setMapPtr(std::shared_ptr<OctomapHandler> ot_handle){ot_handle_ = ot_handle;}
    void setInitState(pvaState x_init);
    friend class MpPlanner;
private:
    std::shared_ptr<OctomapHandler> ot_handle_;
    MpPlanner* mp_planner_;
    double evaluateMp(MotionPrimitive* mp);
    double computePerceptionCost(MotionPrimitive* mp);
    double computeCollisionCost(MotionPrimitive* mp);
    double computeGoalProxCost(MotionPrimitive* mp);
    double computeEndpointCost(MotionPrimitive* mp);
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
    pvaState x_init_;   //reset by mp_planner

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
    void reset();
    void setInitState(pvaState x_init);
    void setGoal(Eigen::Vector3d g){mp_eval_->setGoal(g);}
    void setMapPtr(std::shared_ptr<OctomapHandler> map_handler){mp_eval_->setMapPtr(map_handler);}
    void planTrajectory(std::vector<globalPlan>);
    void getNextSetpoints(double t_query, std::vector<SetPoint>& sp_vec, int m);
    void getBestTrajectory(double t_query, std::vector<Eigen::Vector3d>& pos_vec);
    std::vector<Eigen::Vector3d> computeMpEndpoints(pvaState x_init);
    MpEvaluator* getMpEvalPtr(){return mp_eval_.get();}
    friend class MpEvaluator;
private:
    //member functions
    void generateMps();
    MotionPrimitive findBestMp();
    MotionPrimitive prev_best_mp; //previously best motion primitive
    inline int linearMpIndex(int h_idx, int v_idx){ return v_idx * h_res + h_idx;}
    pvaState x_init_;
    MotionPrimitive* mp_lib; //motion primitives
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
};
}//online_pa_planner


#endif
