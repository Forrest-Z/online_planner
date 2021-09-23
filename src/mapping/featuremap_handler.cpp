#include <mapping/featuremap_handler.h>
#include <algorithm>

using namespace std;

namespace online_planner{
FeatureMapHandler::FeatureMapHandler(FeatureMapHandler::Param p):
d_margin_(p.d_margin), d_min_(p.d_min), voxel_size_(p.v_size), cut_ray_at_(p.cut_ray_at), max_features_per_voxel_(p.mf), 
save_kf_for_(p.save_kf_for), n_features(0), update_timer_(string("- Feature Map Update timer(per keyframe)")), 
query_timer_(string("- Visibility Query timer(per pose)")){
}

void FeatureMapHandler::setOctHandle(std::shared_ptr<OctomapHandler> oct_handle){
    oct_handle_ = oct_handle;
}

void FeatureMapHandler::printTimers(bool total, bool avg){
    unique_lock<mutex> f_lock(featuremap_mtx_);
    cout<<"***FeatureMapHandler Timer Reports***"<<endl;
    cout<<"# of features in current feature map : "<< n_features<<endl;
    update_timer_.print(total, avg);
    query_timer_.print(total, avg);
    f_lock.unlock();
}

void FeatureMapHandler::getFeatureMap(FeatureVoxelMap::type* f){
    unique_lock<mutex> f_lock(featuremap_mtx_);
    f = &feature_voxel_map;
    return;
}

int FeatureMapHandler::getNfeatures() const{
    unique_lock<mutex> f_lock(featuremap_mtx_);
    return n_features;
}

//Assume ascending order of new features detected
void FeatureMapHandler::updateKfInfo(KFInfo& kf_info){
    unique_lock<mutex> f_lock(featuremap_mtx_);
    update_timer_.tic();
    removeOld();
    sort(kf_info.features.begin(), kf_info.features.end(), compareFpair);
    //compare kf info with last one and find new kf_info
    int idx_prev = 0;
    int last_kf_info_size = last_kf_info.features.size();
    for(auto f_it = kf_info.features.begin();  f_it != kf_info.features.end(); ++f_it){
        bool insert_new_features = true;
        for(; idx_prev < last_kf_info_size; ++idx_prev){
            if(last_kf_info.features[idx_prev].first == f_it->first){ //id matches
                insert_new_features = false;
                break;
            }
            else if(last_kf_info.features[idx_prev].first > f_it->first){ //id exceeds
                break;
            }
        }
        if(!insert_new_features){ //feature id matches old one.
            IdxType prev_voxel_idx, new_voxel_idx;
            getVoxelIdFromPos(f_it->second, new_voxel_idx);
            getVoxelIdFromPos(last_kf_info.features[idx_prev].second, prev_voxel_idx);
            if(new_voxel_idx == prev_voxel_idx){ //only update pos
                //iterate over features in voxel
                for(auto vmap_it = feature_voxel_map[prev_voxel_idx].features.begin(); 
                    vmap_it != feature_voxel_map[prev_voxel_idx].features.end();){
                    if(vmap_it->id == f_it->first){ //found the id match
                        vmap_it->pos = f_it->second;
                        vmap_it->last_kf_id = kf_info.kf_id;
                        ++vmap_it;
                    }
                    else if((vmap_it->pos - f_it->second).norm() < 0.05){ // too close : abandone
                        vmap_it = feature_voxel_map[prev_voxel_idx].features.erase(vmap_it);
                        --n_features;
                    }
                    else ++vmap_it;
                }
            }
            else{ //moved to other voxel
                //remove from past voxel
                for(auto vmap_it = feature_voxel_map[prev_voxel_idx].features.begin(); 
                    vmap_it != feature_voxel_map[prev_voxel_idx].features.end();++vmap_it){
                    if(vmap_it->id == f_it->first){
                        feature_voxel_map[prev_voxel_idx].features.erase(vmap_it);
                        break;
                    }
                }
                for(auto vmap_it = feature_voxel_map[new_voxel_idx].features.begin(); 
                    vmap_it != feature_voxel_map[new_voxel_idx].features.end();){
                    if((vmap_it->pos - f_it->second).norm() < 0.05){ // too close : abandone
                        vmap_it = feature_voxel_map[new_voxel_idx].features.erase(vmap_it);
                        --n_features;
                        continue;
                    }
                    ++vmap_it;
                }
                feature_voxel_map[new_voxel_idx].features.emplace_back(f_it->first, f_it->second, kf_info.kf_id);
            }
        }
        else{ //new features
            IdxType voxel_idx;
            getVoxelIdFromPos(f_it->second, voxel_idx);
            for(auto vmap_it = feature_voxel_map[voxel_idx].features.begin(); 
                    vmap_it != feature_voxel_map[voxel_idx].features.end();){
                if((vmap_it->pos - f_it->second).norm() < 0.05){ // too close : abandone
                        vmap_it = feature_voxel_map[voxel_idx].features.erase(vmap_it);
                        --n_features;
                        continue;
                }
                ++vmap_it;
            }
            feature_voxel_map[voxel_idx].features.emplace_back(f_it->first, f_it->second, kf_info.kf_id);
            ++n_features;
        }
    }
    last_kf_info = kf_info;
    update_timer_.toc();
    f_lock.unlock();
}

// note: only call this inside updateKFInfo, since this function does not acquire additional lock
void FeatureMapHandler::removeOld(){
    int old_kf_id = last_kf_info.kf_id - save_kf_for_;
    for(auto vox_it = feature_voxel_map.begin(); vox_it != feature_voxel_map.end();){
        for(auto f_it = vox_it->second.features.end(); f_it != vox_it->second.features.begin();){
            --f_it;
            if(f_it->last_kf_id <= old_kf_id){
                f_it = vox_it->second.features.erase(f_it);
                --n_features;
            }
        }
        if(vox_it->second.features.size() > max_features_per_voxel_){ //too large
            std::sort(vox_it->second.features.begin(), vox_it->second.features.end(), RcompareFeatureKfId);
            vox_it->second.features.resize(max_features_per_voxel_); // only the recently observed ones remains
        }
        if(vox_it->second.features.empty()){
           vox_it = feature_voxel_map.erase(vox_it);
           continue;
        }
        ++vox_it;
    }
}

//optimistic : whether to ignore unknown in raycasting
//vec : where visible vector is obtained
void FeatureMapHandler::queryVisible(transform_utils::SE3 T_wc, vector<FeatureInfo>& vec, pair<double, double> fov_rads, bool optimistic){
    //First search for points in fov
    vector<FeatureInfo> in_fov;
    vector<Eigen::Vector3d> in_fov_pos;
    double h_fov_rad = fov_rads.first;
    double v_fov_rad = fov_rads.second;
    Eigen::Matrix3d R_cw = T_wc.rotation().transpose();
    Eigen::Vector3d t_wc = T_wc.translation();
    Eigen::Vector3d t_cw = -R_cw*t_wc;
    unique_lock<mutex> f_lock(featuremap_mtx_);
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
    oct_handle_->castRayGroup(t_wc, in_fov_pos, is_occluded, d_margin_, cut_ray_at_, optimistic);
    for(int idx = 0; idx < in_fov_pos.size();++idx){
        if(!is_occluded[idx]) vec.push_back(in_fov[idx]);
    }
    query_timer_.toc();
    f_lock.unlock();
    return;
}

}