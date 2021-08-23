#ifndef FEATUREMAP_UTILS_H_
#define FEATUREMAP_UTILS_H_

#include <vector>
#include <unordered_map>
#include <Eigen/Dense>

namespace online_planner{

using IdxType = Eigen::Matrix<int, 3, 1>;

//might consider "image coordinate from last observed kf"
struct FeatureInfo{ 
    int id; //feature id
    Eigen::Vector3d pos; //current estimation of position in odom frame
    int last_kf_id; //last keyframe id observed
    FeatureInfo(int i, Eigen::Vector3d p, int kfid):id(i), pos(p), last_kf_id(kfid){}
};

bool compareFeatureKfId(FeatureInfo a, FeatureInfo b){
    return a.last_kf_id < b.last_kf_id;
}

bool compareFeatureId(FeatureInfo a, FeatureInfo b){//used to sort
    return a.id < b.id; //upsort by id
}

struct KFInfo{
    int kf_id;
    std::vector<std::pair<int, Eigen::Vector3d>> features;
};

class FeatureVoxel{
public:
    FeatureVoxel(Eigen::Vector3d c):center_position(c){}
    ~FeatureVoxel(){}
    Eigen::Vector3d center_position;
    std::vector<FeatureInfo> features; //might require deletion and insertion into other voxel while updating
};

struct VoxelHash{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t sl = 524287;
    static constexpr size_t sl2 = sl * sl;

    std::size_t operator()(const IdxType& index) const {
    return static_cast<unsigned int>(index.x() + index.y() * sl +
                                     index.z() * sl2);
    }
};

struct FeatureVoxelMap{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::unordered_map<IdxType, FeatureVoxel, VoxelHash, std::equal_to<IdxType>, 
    Eigen::aligned_allocator<std::pair<const IdxType, FeatureVoxel>>> type;
};
}
#endif