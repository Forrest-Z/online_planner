#ifndef ONLINE_FEATUREMAP_HANDLER_H_
#define ONLINE_FEATUREMAP_HANDLER_H_

#include <memory>
#include <Eigen/Dense>
#include <mutex>
#include <utils/timer.h>
#include <utils/featuremap_types.h>
#include <mapping/octomap_handler.h>
#include <transform_utils/transform_utils.h>

namespace online_planner{

class FeatureMapHandler{
public:
    struct Param{
        double d_margin, d_min, v_size;
        int mf, save_kf_for;
    };
    FeatureMapHandler(Param p);
    ~FeatureMapHandler();
    void setOctHandle(std::shared_ptr<OctomapHandler> oct_handle);
    void updateKfInfo(KFInfo& kf_info);
    void removeOld();
    //follows optical frame convention for T(z axis = optical frame axis)
    void queryVisible(transform_utils::SE3 T, std::vector<FeatureInfo>& vec, std::pair<double, double>, bool optimistic);
    void printTimers(bool, bool);
private:
    inline void getVoxelIdFromPos(Eigen::Vector3d pos, IdxType& idx){
        constexpr double keps = 0.00001;
        idx.x() = std::floor(pos.x() / voxel_size_ + keps);
        idx.y() = std::floor(pos.y() / voxel_size_ + keps);
        idx.z() = std::floor(pos.z() / voxel_size_ + keps);
    }

    inline bool getVoxelIdFromFeatureId(int f_id, IdxType& idx){
        auto iter_ = id_map.find(f_id);
        if(iter_ != id_map.end()){
            idx = iter_->second;
            return true;
        }
        return false;
    }

    FeatureVoxelMap::type feature_voxel_map;
    std::unordered_map<int, IdxType> id_map; // id - id_map finder
    std::shared_ptr<OctomapHandler> oct_handle_; //octomap handler for querying occlusion
    KFInfo last_kf_info; //last kf info. features upsorted by feature id

    //parameters
    double d_margin_; // amount of margin for checking raycast operation
    double d_min_; //minimum distance for the feature to be "visible"
    double voxel_size_; //voxel size
    int max_features_per_voxel_; //maximum number of features per voxel to maintain.
    int save_kf_for_;

    //timers
    Timer update_timer_;
    Timer query_timer_;

    //mutex
    std::mutex featuremap_mtx_;
};






}

#endif