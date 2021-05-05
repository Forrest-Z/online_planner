#include <planner/local_planner/motion_primitives_planner.h>
#include <assert.h>
#include <cmath>

namespace online_planner{
    MpPlanner::MpPlanner(MpPlanner::Param param): L(param.L), h_ang_max(param.h_ang_max), k_th(param.k_th),
                                              v_ang(param.v_ang), h_res(param.h_res), v_res(param.v_res), v_des(param.v_des)
    {
        mp_eval_ = std::unique_ptr<MpEvaluator>(new MpEvaluator(param.eval_param));
        mp_eval_->mp_planner_ = this;
        num_mps = h_res * v_res;
        mp_lib = new MotionPrimitive[num_mps];
        reset();
    }

    MpPlanner::~MpPlanner(){
        delete[] mp_lib;
    }

    void MpPlanner::reset(){
        for(int idx = 0; idx < num_mps; ++idx){
            mp_lib[idx].coeffs = Eigen::Matrix<double, DIM_PLANNING, MP_ORDER+1>::Zero();
        }
        best_idx = 0;
        found_best = false;
    }

    void MpPlanner::setInitState(pvaState x_init){
        x_init_ = x_init;
        for(int idx = 0; idx < num_mps; ++idx){
            mp_lib[idx].yaw_init = x_init.yaw;
        }
        mp_eval_->setInitState(x_init);
    }

    void MpPlanner::planTrajectory(std::vector<globalPlan>){
        generateMps();
    }

    void MpPlanner::getNextSetpoints(double t_query, std::vector<SetPoint>& sp_vec, int m){
        auto best_mp = findBestMp();
        for(int i=1; i <= m; ++i){
            SetPoint sp;
            pvaState x = best_mp.state(t_query * i);
            sp.position = x.position;
            sp.velocity = x.velocity;
            sp.accel = x.accel;
            sp.yaw = x.yaw;
            sp_vec.push_back(sp);
        }
        return;
    }

    //used for visualization
    void MpPlanner::getBestTrajectory(double t_query, std::vector<Eigen::Vector3d>& pos_vec){
        auto best_mp = findBestMp();
        for(int i=1; (i*t_query) <= best_mp.Tf; ++i){
            SetPoint sp;
            Eigen::Vector3d pos = best_mp(t_query * i);
            pos_vec.push_back(pos);
        }
        return;
    }

    std::vector<Eigen::Vector3d> MpPlanner::computeMpEndpoints(pvaState x_init){
        std::vector<Eigen::Vector3d> Endpoints;
        double speed_init = x_init.velocity.norm();
        double yaw = x_init.yaw;
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
                Eigen::Vector3d p_f = x_init.position + displacement;
                Endpoints.push_back(p_f);
            }
        }
        return Endpoints;
    }

    void MpPlanner::generateMps(){
        double speed_init = x_init_.velocity.norm();
        double yaw = x_init_.yaw;
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
        for(int h_idx = 0; h_idx < h_res; ++h_idx){
            for(int v_idx = 0; v_idx < v_res; ++v_idx, ++idx){
                //define terminal point
                mp_lib[idx].Tf = Tf;
                Eigen::Vector3d displacement = Eigen::Vector3d(L * sin(M_PI_2 - v) *cos(h), L * sin(M_PI_2 - v) * sin(h), L * cos(M_PI_2 - v)); 
                mp_lib[idx].p_f = x_init_.position + displacement;
                mp_lib[idx].computeCoeffs(x_init_);
                v = v + v_ivl;
            }
            h = h + h_ivl;
        }
    }

    MotionPrimitive MpPlanner::findBestMp(){
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

    void MpEvaluator::setInitState(pvaState x_init){
        x_init_ = x_init;
    }

    double MpEvaluator::evaluateMp(MotionPrimitive* mp){
        double p_cost = computePerceptionCost(mp); // perception cost
        double c_cost = computeCollisionCost(mp); // collision cost
        double g_cost = computeGoalProxCost(mp); // goal proximity cost
        double f_cost = computeEndpointCost(mp); //end point myopicity cost
        return k_per * p_cost + k_prox * g_cost + k_col * c_cost + k_end * f_cost;
    }

    double MpEvaluator::computePerceptionCost(MotionPrimitive* mp){
        //To be implemented
        return 0.0;
    }

    double MpEvaluator::computeCollisionCost(MotionPrimitive* mp){
        static constexpr double dt = 0.02;
        double T1 = 0.0; double T2 = 0.0; double Tf = mp->Tf;
        auto p0 = (*mp)(T1); auto pf = (*mp)(Tf - T2);
        Eigen::Vector3d p1 = p0;
        Eigen::Vector3d p2 = pf;
        double smallest_safe_dist = 1e10;
        while(T1 + T2 < Tf){
            double safe_dist1 = ot_handle_->getSafeDistanceAtPosition(p1, dist_th);
            double safe_dist2 = ot_handle_->getSafeDistanceAtPosition(p2, dist_th);
            smallest_safe_dist = (smallest_safe_dist < safe_dist1)? smallest_safe_dist : safe_dist1;
            smallest_safe_dist = (smallest_safe_dist < safe_dist2)? smallest_safe_dist : safe_dist2;
            if(safe_dist1 < 0.0 || safe_dist2 < 0.0) return 1.0; 
            while(true){
                T1 += dt;
                p1 = (*mp)(T1);
                if((p1 - p0).norm() >= safe_dist1) break;
            }
            while(Tf - T2 > T1){
                T2 += dt;
                p2 = (*mp)(Tf - T2);
                if((p2 - pf).norm() >= safe_dist2) break;
            }
        }
        if(smallest_safe_dist > safe_dist) return 0.0;
        else return pow((safe_dist - smallest_safe_dist)/safe_dist, power_dist);
    }

    //modified to goal heading & proximity cost
    double MpEvaluator::computeGoalProxCost(MotionPrimitive* mp){
        double min_dist_to_goal = 10000.0;
        for(double dt = 0.1; dt < mp->Tf; ++dt){
            Eigen::Vector3d point = (*mp)(dt);
            double dist_to_goal = (goal - point).norm() ;
            if(dist_to_goal < min_dist_to_goal){
                min_dist_to_goal = dist_to_goal;
            }
        }
        //find the nearest point to the goal point along the trajectory
        if(min_dist_to_goal < d_critic) return min_dist_to_goal/d_critic; //1.0 if d_critic
        else{
            Eigen::Vector3d endpoint = (*mp)(mp->Tf);
            Eigen::Vector3d endvel = (*mp)(mp->Tf, 1);
            Eigen::Vector3d goal_dir = (goal - endpoint).normalized();
            Eigen::Vector3d heading = endvel.normalized();
            return (1.0 + acos(heading.dot(goal_dir)));
        }
    }

    double MpEvaluator::computeEndpointCost(MotionPrimitive* mp){
        pvaState x_end = mp->state(mp->Tf);
        auto lookahead_vec = mp_planner_->computeMpEndpoints(x_end);
        double L = mp_planner_->L;
        double mean_ray = 0.0;
        size_t n = lookahead_vec.size();
        for(auto iter = lookahead_vec.begin(); iter!=lookahead_vec.end(); ++iter){
            Eigen::Vector3d lookahead_endpoint = *iter;
            Eigen::Vector3d dir = (lookahead_endpoint - x_end.position).normalized();
            double ray_dist = ot_handle_->castRay(x_end.position, dir, L, true);
            mean_ray += ray_dist / n;
        }
        return (L - mean_ray) / L;
    }
}//online_planner