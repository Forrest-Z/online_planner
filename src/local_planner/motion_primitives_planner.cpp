#include <planner/local_planner/motion_primitives_planner.h>
#include <assert.h>
#include <cmath>

using namespace traj_lib;

namespace online_planner{
MpPlanner::MpPlanner(MpPlanner::Param param): L(param.L), h_ang_max(param.h_ang_max), k_th(param.k_th),
                                            v_ang(param.v_ang), h_res(param.h_res), v_res(param.v_res), v_des(param.v_des)
{
    mp_eval_ = std::unique_ptr<MpEvaluator>(new MpEvaluator(param.eval_param));
    mp_eval_->mp_planner_ = this;
    num_mps = h_res * v_res;
    mp_lib = new MinJerkPolyTraj[num_mps];
    reset(0.0);
}

MpPlanner::~MpPlanner(){
    delete[] mp_lib;
}

void MpPlanner::reset(double init_time){
    for(int idx = 0; idx < num_mps; ++idx){
        mp_lib[idx].reset(init_time);
    }
    curr_start_time = init_time;
    best_idx = 0;
    found_best = false;
}

void MpPlanner::setInitState(FlatState x_init){
    x_init_ = x_init;
    for(int idx = 0; idx < num_mps; ++idx){
        mp_lib[idx].setInitState(x_init_);
    }
    mp_eval_->setInitState(x_init);
}

void MpPlanner::planTrajectory(std::vector<globalPlan>){
    generateMps();
}

void MpPlanner::getNextSetpoints(double dt, std::vector<traj_lib::FlatState>& sp_vec, int m){
    sp_vec.clear();
    auto best_mp = findBestMp();
    double t_start, t_end;
    best_mp.getTimeSpan(t_start, t_end);
    int i=0;
    for(double t = t_start + dt; t_start < t_end, i < m; t_start += dt){
        traj_lib::FlatState sp = getState(&best_mp, t);
        sp.pos_order = 2;
        sp.yaw_order = 0;
        sp_vec.push_back(sp);
        ++i;
    }
    return;
}

//used for visualization
void MpPlanner::getTrajectoryForViz(double dt, std::vector<Eigen::Vector3d>& pos_vec){
    auto best_mp = findBestMp();
    double t_start, t_end;
    best_mp.getTimeSpan(t_start, t_end);
    for(double t = t_start; t_start < t_end; t_start += dt){
        Eigen::Vector3d pos = best_mp(t).p;
        pos_vec.push_back(pos);
    }
    return;
}

std::vector<Eigen::Vector3d> MpPlanner::computeMpEndpoints(traj_lib::FlatState x_init){
    std::vector<Eigen::Vector3d> Endpoints;
    double speed_init = x_init.states[1].p.norm();
    double yaw = x_init.states[0].yaw;
    double h_ang = (k_th * speed_init < h_ang_max)? k_th * speed_init : h_ang_max;
    double h_ivl, v_ivl;
    double h_ang_rad = h_ang / 180.0 * M_PI;
    double v_ang_rad = v_ang / 180.0 * M_PI; 
    if(h_res != 1) h_ivl = 2*h_ang_rad / (h_res-1);
    else h_ivl = 0.0;
    if(v_res != 1) v_ivl = 2*v_ang_rad / (v_res-1);
    else v_ivl = 0.0;
    int idx = 0;
    double h_init = (h_res == 1)? yaw : yaw - h_ang_rad;
    double v_init = (v_res == 1)? 0.0 : -v_ang_rad;
    double h, v;
    for(int h_idx = 0; h_idx < h_res; ++h_idx){
        h = h_init + h_idx * h_ivl;
        for(int v_idx = 0; v_idx < v_res; ++v_idx, ++idx){
            v = v_init + v_idx * v_ivl;
            //define terminal point
            Eigen::Vector3d displacement = Eigen::Vector3d(L * sin(M_PI_2 - v) *cos(h), L * sin(M_PI_2 - v) * sin(h), L * cos(M_PI_2 - v)); 
            Eigen::Vector3d p_f = x_init.states[0].p + displacement;
            Endpoints.push_back(p_f);
        }
    }
    return Endpoints;
}

void MpPlanner::generateMps(){
    double speed_init = x_init_.states[1].p.norm();
    double yaw = x_init_.states[0].yaw;
    double Tf = (L / (speed_init + 0.5) > L / v_des)? L / (speed_init + 0.5) : L / v_des;
    double h_ang = (k_th * speed_init > h_ang_max)? k_th * speed_init : h_ang_max;
    double h_ivl, v_ivl;
    double h_ang_rad = h_ang / 180.0 * M_PI;
    double v_ang_rad = v_ang / 180.0 * M_PI; 
    if(h_res != 1) h_ivl = 2*h_ang_rad / (h_res-1);
    else h_ivl = 0.0;
    if(v_res != 1) v_ivl = 2*v_ang_rad / (v_res-1);
    else v_ivl = 0.0;
    int idx = 0;
    double h_init = (h_res == 1)? yaw : yaw - h_ang_rad;
    double v_init = (v_res == 1)? 0.0 : -v_ang_rad;
    double h = h_init;
    double v = v_init;
    traj_lib::FlatState ff;
    ff.pos_order = 0;
    ff.yaw_order = 0;
    Eigen::Matrix3Xd M = Eigen::MatrixX3d::Zero(1, 3);
    M(0, 0) = 10.0 / pow(Tf, 3);
    M(0, 1) = -20.0 / pow(Tf, 4);
    M(0, 2) = 20.0 / pow(Tf, 5);
    for(int h_idx = 0; h_idx < h_res; ++h_idx){
        for(int v_idx = 0; v_idx < v_res; ++v_idx, ++idx){
            //define terminal point
            mp_lib[idx].setDuration(Tf);
            Eigen::Vector3d displacement = Eigen::Vector3d(L * sin(M_PI_2 - v) *cos(h), L * sin(M_PI_2 - v) * sin(h), L * cos(M_PI_2 - v)); 
            Eigen::Vector3d p_f = x_init_.states[1].p + displacement;
            ff.states[0].p = p_f;   
            mp_lib[idx].setEndState(ff);
            mp_lib[idx].computeCoeffs(M);
            v = v + v_ivl;
        }
        h = h + h_ivl;
    }
}

traj_lib::FlatState MpPlanner::getState(MinJerkPolyTraj* mp, double t){
    traj_lib::FlatState f;
    f.pos_order = 2;
    f.yaw_order = 0;
    f.states.resize(3);
    f.states[0].p = (*mp)(t).p;
    f.states[1].p = (*mp)(t,1).p;
    f.states[2].p = (*mp)(t,2).p;
    f.states[0].yaw = atan2(f.states[1].p[1], f.states[1].p[0]);
    if(f.states[1].p.norm() < 0.01){
        f.states[0].yaw = x_init_.states[0].yaw; // velocity tracking yaw might be too noisy for small velocity
    }
}

MinJerkPolyTraj MpPlanner::findBestMp(){
    if(found_best) return mp_lib[best_idx];
    double min_cost = 1e10;
    best_idx = 0;
    for(int idx = 0; idx < num_mps; ++idx){
        double cost = mp_eval_->evaluateMp(&mp_lib[idx]);
        if(cost < min_cost){
            best_idx = idx;
            min_cost = cost;
        }
    }
    found_best = true;
    return mp_lib[best_idx];
}

void MpEvaluator::setInitState(traj_lib::FlatState x_init){
    x_init_ = x_init;
}

double MpEvaluator::evaluateMp(MinJerkPolyTraj* mp){
    double p_cost = computePerceptionCost(mp); // perception cost
    double c_cost = computeCollisionCost(mp); // collision cost
    double g_cost = computeGoalProxCost(mp); // goal proximity cost
    double f_cost = computeEndpointCost(mp); //end point myopicity cost
    return k_per * p_cost + k_prox * g_cost + k_col * c_cost + k_end * f_cost;
}

double MpEvaluator::computePerceptionCost(MinJerkPolyTraj* mp){
    //To be implemented
    return 0.0;
}

double MpEvaluator::computeCollisionCost(MinJerkPolyTraj* mp){
    static constexpr double dt = 0.02;
    double T1, T2;
    mp->getTimeSpan(T1, T2);
    auto p0 = (*mp)(T1).p; auto pf = (*mp)(T2).p;
    Eigen::Vector3d p1 = p0;
    Eigen::Vector3d p2 = pf;
    double smallest_safe_dist = 1e10;
    while(T2 > T1){
        double safe_dist1 = ot_handle_->getSafeDistanceAtPosition(p1, dist_th);
        double safe_dist2 = ot_handle_->getSafeDistanceAtPosition(p2, dist_th);
        smallest_safe_dist = (smallest_safe_dist < safe_dist1)? smallest_safe_dist : safe_dist1;
        smallest_safe_dist = (smallest_safe_dist < safe_dist2)? smallest_safe_dist : safe_dist2;
        if(safe_dist1 < 0.0 || safe_dist2 < 0.0) return 1.0; 
        while(true){
            T1 += dt;
            p1 = (*mp)(T1).p;
            if((p1 - p0).norm() >= safe_dist1) break;
        }
        while(T2 > T1){
            T2 -= dt;
            p2 = (*mp)(T2).p;
            if((p2 - pf).norm() >= safe_dist2) break;
        }
    }
    if(smallest_safe_dist > safe_dist) return 0.0;
    else return pow((safe_dist - smallest_safe_dist)/safe_dist, power_dist);
}

//modified to goal heading & proximity cost
double MpEvaluator::computeGoalProxCost(MinJerkPolyTraj* mp){
    double min_dist_to_goal = 10000.0;
    double dt = 0.1;
    double t0, tf;
    mp->getTimeSpan(t0, tf);
    for(double t = t0; t < tf; t += dt){
        Eigen::Vector3d point = (*mp)(t).p;
        double dist_to_goal = (goal - point).norm() ;
        if(dist_to_goal < min_dist_to_goal){
            min_dist_to_goal = dist_to_goal;
        }
    }
    //find the nearest point to the goal point along the trajectory
    if(min_dist_to_goal < d_critic) return min_dist_to_goal/d_critic; //1.0 if d_critic
    else{
        Eigen::Vector3d endpoint = (*mp)(tf).p;
        Eigen::Vector3d endvel = (*mp)(tf, 1).p;
        Eigen::Vector3d goal_dir = (goal - endpoint).normalized();
        Eigen::Vector3d heading = endvel.normalized();
        return (1.0 + acos(heading.dot(goal_dir)));
    }
}

double MpEvaluator::computeEndpointCost(MinJerkPolyTraj* mp){
    double t0, tf;
    mp->getTimeSpan(t0, tf);
    traj_lib::FlatState x_end = mp_planner_->getState(mp, tf);
    auto lookahead_vec = mp_planner_->computeMpEndpoints(x_end);
    double L = mp_planner_->L;
    double mean_ray = 0.0;
    size_t n = lookahead_vec.size();
    for(auto iter = lookahead_vec.begin(); iter!=lookahead_vec.end(); ++iter){
        Eigen::Vector3d lookahead_endpoint = *iter;
        Eigen::Vector3d dir = (lookahead_endpoint - x_end.states[0].p).normalized();
        double ray_dist = ot_handle_->castRay(x_end.states[0].p, dir, L, true);
        mean_ray += ray_dist / n;
    }
    return (L - mean_ray) / L;
}
}//online_planner