#ifndef FEATUREMAP_TYPES_H_
#define FEATUREMAP_TYPES_H_

#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <vins_vio_mod/KeyframeInfo.h>
#include <transform_utils/transform_utils.h>
#include <iostream>

namespace online_planner{

using IdxType = Eigen::Matrix<int, 3, 1>;

//might consider "image coordinate from last observed kf"
struct FeatureInfo{ 
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FeatureInfo(int i, Eigen::Vector3d p, int kfid):id(i), pos(p), last_kf_id(kfid){}
    FeatureInfo():id(-1), pos(Eigen::Vector3d::Zero()), last_kf_id(-1){}
    int id; //feature id
    Eigen::Vector3d pos; //current estimation of position in odom frame
    int last_kf_id; //last keyframe id observed
};

// down sort
inline bool RcompareFeatureKfId(const FeatureInfo& a, const FeatureInfo& b){
    return a.last_kf_id > b.last_kf_id;
}

inline bool compareFpair(const std::pair<int, Eigen::Vector3d>& a, const std::pair<int, Eigen::Vector3d>& b){//used to sort
    return a.first < b.first; //upsort by id
}

struct KFInfo{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int kf_id;
    transform_utils::SE3 kf_pose;
    std::vector<std::pair<int, Eigen::Vector3d>> features;
};

class FeatureVoxel{
public:
    FeatureVoxel(){
        features.clear();
    }
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

inline void kfMsgToKfInfo(vins_vio_mod::KeyframeInfo& kf_msg, KFInfo& kf_info, double cutting_thr){
    constexpr size_t int_size = sizeof(int);
    kf_info.kf_id = kf_msg.kf_id;
    kf_info.kf_pose = transform_utils::utils::from_vec3_quat(kf_msg.kf_pose.position, kf_msg.kf_pose.orientation);
    unsigned char* data_ptr = &(kf_msg.features.data[0]);
    size_t point_step = kf_msg.features.point_step;
    size_t n_features = kf_msg.features.data.size() / point_step;
    size_t p_size = point_step - int_size;
    auto kf_pos = kf_msg.kf_pose.position;
    //kf_info.features.resize(n_features);
    int* id_ptr = new int();
    double cthr_sq = cutting_thr * cutting_thr;
    float xyz[3];
    for(int i = 0; i < n_features; ++i){
        Eigen::Vector3d f_pos; int id;
        try{ //only for debugging?
            std::copy(data_ptr, data_ptr + int_size, reinterpret_cast<char*>(id_ptr));
            std::copy(data_ptr + int_size, data_ptr + point_step, reinterpret_cast<char*>(&xyz[0]));
            data_ptr += point_step;
        }
        catch(const std::runtime_error& re){
            std::cerr<<"KfMsgToKfInfo : Runtime error occurred."<<std::endl;
            break;
        }
        catch(const std::exception& ex){
            std::cerr<<"KfMsgToKfInfo : exception occurred."<<std::endl;
            break;
        }
        catch(...){
            std::cerr<<"KfMsgToKfInfo : Unknown occurred."<<std::endl;
            break;
        }
        if((xyz[0] - kf_pos.x)*(xyz[0] - kf_pos.x)+(xyz[1] - kf_pos.y)*(xyz[1] - kf_pos.y)+(xyz[2] - kf_pos.z)*(xyz[2] - kf_pos.z) >= cthr_sq)
            continue;
        id = *id_ptr;
        f_pos.x() = static_cast<double>(xyz[0]); f_pos.y() = static_cast<double>(xyz[1]); f_pos.z() = static_cast<double>(xyz[2]);
        kf_info.features.emplace_back(id, f_pos);
    }
    return;
}
}
#endif