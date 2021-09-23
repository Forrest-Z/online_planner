#include <mapping/featuremap_handler.h>
#include <ros/ros.h>
#include <transform_utils/transform_utils.h>
#include <time.h>

using namespace online_planner;
using namespace transform_utils;

static int feature_id_count = 0; //update this value whenever new feature create
static int kf_count = 0;
static double r = 10.0;

Eigen::Vector3d move_per_kf = Eigen::Vector3d(0.25, 0.25, 0.0);

inline void modifyKfInfo(KFInfo& kf_info, double replace_rate, double perturb=0.1){
    Eigen::Vector3d Xmin(-r, -r, -r);
    Eigen::Vector3d Xmax(r, r, r);
    Eigen::Vector3d pmin(-perturb,-perturb,-perturb);
    Eigen::Vector3d pmax(perturb, perturb, perturb);
    int replace_features = 0;
    kf_info.kf_id = ++kf_count;
    Xmin = Xmin + move_per_kf * kf_count;
    Xmax = Xmax + move_per_kf * kf_count;
    for(auto iter_ = kf_info.features.begin(); iter_ != kf_info.features.end();){
        double p = static_cast<double>(std::rand())/(RAND_MAX);
        if(p < replace_rate){
            iter_ = kf_info.features.erase(iter_);
            ++replace_features;
        }
        else{
            iter_->second += sample_random_point<double>(pmin, pmax);
            ++iter_;
        }
    }
    std::cout<<"ModifyKFInfo : Add new "<<replace_features<<" features"<<std::endl;
    //add new features
    for(int i=0; i < replace_features; ++i){
        Eigen::Vector3d pos = sample_random_point<double>(Xmin, Xmax);
        kf_info.features.emplace_back(feature_id_count++, pos);
    }
    std::cout<<"Feature ID count : "<<feature_id_count<<std::endl;
}

inline void generateRandomKfInfo(KFInfo& kf_info, int n, double r_=10.0){
    kf_info.kf_id = 0;
    kf_info.kf_pose = from_Matrix4d(Eigen::Matrix4d::Identity());
    r = r_;
    Eigen::Vector3d Xmin(-r, -r, -r);
    Eigen::Vector3d Xmax(r, r, r);
    for(int i=0; i<n; ){
        Eigen::Vector3d p = sample_random_point<double>(Xmin, Xmax);
        kf_info.features.emplace_back(i, p);
        feature_id_count = ++i;
    }
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "fm_test");
    srand(time(NULL));
    
    ros::NodeHandle nh_private("~");

    FeatureMapHandler::Param p;
    nh_private.param("FeatureMap/d_margin", p.d_margin, 0.5);
    nh_private.param("FeatureMap/d_min", p.d_min, 0.1);
    nh_private.param("FeatureMap/voxel_size", p.v_size, 0.3);
    nh_private.param("FeatureMap/max_features_per_voxel", p.mf, 10);
    nh_private.param("FeatureMap/save_kf_for", p.save_kf_for, 5);
    nh_private.param("FeatureMap/cut_ray_at", p.cut_ray_at, 10.0);

    FeatureMapHandler fm_handle_(p);

    KFInfo kf_info;
    generateRandomKfInfo(kf_info, 200, 10.0);

    for(int i=0; i<20; ++i){
        fm_handle_.updateKfInfo(kf_info);
        fm_handle_.printTimers(true, true);
        modifyKfInfo(kf_info, 0.35, 0.2);
    }
    return 1;
}
