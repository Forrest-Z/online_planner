#ifndef FEATUREMAP_TYPES_H_
#define FEATUREMAP_TYPES_H_

#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>


namespace online_planner{

using IdxType = Eigen::Matrix<int, 3, 1>;

//might consider "image coordinate from last observed kf"
struct FeatureInfo{ 
    int id; //feature id
    Eigen::Vector3d pos; //current estimation of position in odom frame
    int last_kf_id; //last keyframe id observed
    FeatureInfo(int i, Eigen::Vector3d p, int kfid):id(i), pos(p), last_kf_id(kfid){}
};

inline bool compareFeatureKfId(const FeatureInfo& a, const FeatureInfo& b){
    return a.last_kf_id < b.last_kf_id;
}

inline bool compareFpair(const std::pair<int, Eigen::Vector3d>& a, const std::pair<int, Eigen::Vector3d>& b){//used to sort
    return a.first < b.first; //upsort by id
}

struct KFInfo{
    int kf_id;
    std::vector<std::pair<int, Eigen::Vector3d>> features;
};

class FeatureVoxel{
public:
    FeatureVoxel(){}
    ~FeatureVoxel(){}
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

//modifying content of the header is up to the user
inline void FeatureMapToPointCloud2(const FeatureVoxelMap::type& fm_map, sensor_msgs::PointCloud2& cloud, const int n){
    constexpr int num_channels = 3;
    const std::string channel_id[] = { "x", "y", "z"};

    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.fields.resize(num_channels);
    for (int i = 0; i<num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }
    float data_array[num_channels];
    cloud.data.resize(n * cloud.point_step);
    unsigned char* data_ptr = &(cloud.data[0]);
    int idx = 0;
    for (auto it : fm_map){
        FeatureVoxel voxel = it.second;
        for (auto feat : voxel.features){
            Eigen::Vector3f pos = feat.pos.cast<float>();
            data_array[0] = pos.x();
            data_array[1] = pos.y();
            data_array[2] = pos.z();
            if(idx >= n){
                std::cerr<<"total number of features exceeded input n"<<std::endl;
                cloud.data.resize((++idx)*cloud.point_step);
            }
            else{
                ++idx;
            }
            memcpy(data_ptr, data_array, cloud.point_step);
            data_ptr += cloud.point_step;
            ++cloud.width;
        }
    }
    cloud.row_step = cloud.point_step * cloud.width;
    if(idx > n){
        std::cerr<<"total number of features : "<<idx<<std::endl;
    }
}

}
#endif