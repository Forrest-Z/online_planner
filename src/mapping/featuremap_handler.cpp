#include <mapping/featuremap_handler.h>
#include <algorithm>

using namespace std;

namespace online_planner{
FeatureMapHandler::FeatureMapHandler(FeatureMapHandler::Param p):
d_margin_(p.d_margin), d_min_(p.d_min), voxel_size_(p.v_size), max_features_per_voxel_(p.mf), save_kf_for_(p.save_kf_for),
update_timer_(string("- Feature Map Update timer(per keyframe)")), 
query_timer_(string("- Visibility Query timer(per pose)")){}

void FeatureMapHandler::setOctHandle(std::shared_ptr<OctomapHandler> oct_handle){
    oct_handle_ = oct_handle;
}

void FeatureMapHandler::printTimers(bool total, bool avg){
    cout<<"***FeatureMapHandler Timer Reports***"<<endl;
    update_timer_.print(total, avg);
    query_timer_.print(total, avg);
}

//Assume ascending order of new features detected
void FeatureMapHandler::updateKfInfo(KFInfo& kf_info){
    unique_lock<mutex> lock(featuremap_mtx_);
    update_timer_.tic();
    removeOld();
    sort(kf_info.features.begin(), kf_info.features.end(), compareFeatureId);
    //compare kf info with last one and find new kf_info
    int idx_prev = 0;
    int last_kf_info_size = last_kf_info.features.size();
    bool insert_new_features = false;
    for(auto f_it = kf_info.features.begin();  f_it != kf_info.features.end(); ++f_it){
        while(last_kf_info.features[idx_prev].first != f_it->first){
            ++idx_prev;
            if(idx_prev >= last_kf_info_size){
                insert_new_features = true;
                break;
            }
        }
        if(!insert_new_features){ //feature id matches old one
            IdxType prev_voxel_idx, new_voxel_idx;
            getVoxelIdFromPos(f_it->second, new_voxel_idx);
            getVoxelIdFromPos(last_kf_info.features[idx_prev].second, prev_voxel_idx);
            if(new_voxel_idx == prev_voxel_idx){
                for(auto vmap_it = feature_voxel_map[prev_voxel_idx].features.begin(); 
                    vmap_it != feature_voxel_map[prev_voxel_idx].features.end(); ++vmap_it){
                    if(vmap_it->id == f_it->first){ //found the id match
                        vmap_it->pos = f_it->second;
                        vmap_it->last_kf_id = kf_info.kf_id;
                    }
                    else if((vmap_it->pos - f_it->second).norm() < 0.05){ // too close : abandone
                        vmap_it = feature_voxel_map[prev_voxel_idx].features.erase(vmap_it);
                    }
                }
            }
            else{ //moved to other voxel
                for(auto vmap_it = feature_voxel_map[prev_voxel_idx].features.begin(); 
                    vmap_it != feature_voxel_map[prev_voxel_idx].features.end(); ++vmap_it){
                    if(vmap_it->id == f_it->first){
                        vmap_it = feature_voxel_map[prev_voxel_idx].features.erase(vmap_it);
                        break;
                    }
                }
                for(auto vmap_it = feature_voxel_map[new_voxel_idx].features.begin(); 
                    vmap_it != feature_voxel_map[new_voxel_idx].features.end(); ++vmap_it){
                    if((vmap_it->pos - f_it->second).norm() < 0.05){ // too close : abandone
                        vmap_it = feature_voxel_map[new_voxel_idx].features.erase(vmap_it);
                    }
                }
                feature_voxel_map[new_voxel_idx].features.emplace_back(f_it->first, f_it->second, kf_info.kf_id);
            }
        }
        else{ //new features
            IdxType voxel_idx;
            getVoxelIdFromPos(f_it->second, voxel_idx);
            for(auto vmap_it = feature_voxel_map[voxel_idx].features.begin(); 
                    vmap_it != feature_voxel_map[voxel_idx].features.end(); ++vmap_it){
                if((vmap_it->pos - f_it->second).norm() < 0.05){ // too close : abandone
                        vmap_it = feature_voxel_map[voxel_idx].features.erase(vmap_it);
                }
            }
            feature_voxel_map[voxel_idx].features.emplace_back(f_it->first, f_it->second, kf_info.kf_id);
        }
    }
    last_kf_info = kf_info;
    update_timer_.toc();
    lock.unlock();
}

// note: only call this inside updateKFInfo, since this function does not acquire additional lock
void FeatureMapHandler::removeOld(){
    int old_kf_id = last_kf_info.kf_id - save_kf_for_;
    for(auto vox_it = feature_voxel_map.begin(); vox_it != feature_voxel_map.end(); ++vox_it){
        for(auto f_it = vox_it->second.features.begin(); f_it != vox_it->second.features.end(); ++f_it){
            if(f_it->last_kf_id <= old_kf_id || vox_it->second.features.size() >= max_features_per_voxel_)
                f_it = vox_it->second.features.erase(f_it);
           //The way of inserting features into each voxel assures that the features at the front are older
           else break;
       }
    }
}

//optimistic : whether to ignore unknown in raycasting
void FeatureMapHandler::queryVisible(transform_utils::SE3 T_wc, vector<FeatureInfo>& vec, pair<double, double> fov_rads, bool optimistic){
    //First search for points in fov
    vector<FeatureInfo> in_fov;
    vector<Eigen::Vector3d> in_fov_pos;
    double h_fov_rad = fov_rads.first;
    double v_fov_rad = fov_rads.second;
    Eigen::Matrix3d R_cw = T_wc.rotation().transpose();
    Eigen::Vector3d t_wc = T_wc.translation();
    Eigen::Vector3d t_cw = -R_cw*t_wc;
    unique_lock<mutex> lock(featuremap_mtx_);
    query_timer_.tic();
    for(auto vox_it = feature_voxel_map.begin(); vox_it != feature_voxel_map.end(); ++vox_it){
        for(auto f_it = vox_it->second.features.begin(); f_it != vox_it->second.features.end(); ++f_it){
            Eigen::Vector3d p_w = f_it->pos;
            Eigen::Vector3d p_c = R_cw*p_w + t_cw;
            if(p_c.z() > d_min_ && fabs(atan2(p_c.x(), p_c.z())) < h_fov_rad && fabs(atan2(p_c.y(), p_c.z())) < v_fov_rad){
                in_fov.push_back((*f_it));
                in_fov_pos.emplace_back(f_it->pos);
            }
        }
    }
    vector<bool> is_occluded;
    oct_handle_->castRayGroup(t_wc, in_fov_pos, is_occluded, d_margin_, optimistic);
    for(int idx = 0; idx < in_fov_pos.size();++idx){
        if(!is_occluded[idx]) vec.push_back(in_fov[idx]);
    }
    query_timer_.toc();
    lock.unlock();
    return;
}

}