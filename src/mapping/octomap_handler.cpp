#include <mapping/octomap_handler.h>
#include <chrono>

namespace online_planner{

OctomapHandler::OctomapHandler(OctomapHandler::Param param): oct_res(param.oct_res), max_range(param.max_range){
    bbxMax = param.bbxMax; bbxMin = param.bbxMin;
    ot_ = std::unique_ptr<octomap::OcTree>(new octomap::OcTree(oct_res));
    ot_->setBBXMax(bbxMax);
    ot_->setBBXMin(bbxMin);
    ot_->enableChangeDetection(true);
    edt_ = std::unique_ptr<DynamicEDTOctomap>(new DynamicEDTOctomap(max_range, ot_.get(), bbxMin, bbxMax, false));
}
    

double OctomapHandler::getDistanceAtPosition(Eigen::Vector3d p) const{
    return getDistanceAtPosition(octomath::Vector3(p.x(), p.y(), p.z()));
}

double OctomapHandler::getDistanceAtPosition(octomath::Vector3 p) const{
    return edt_->getDistance(p);
}

double OctomapHandler::getSafeDistanceAtPosition(Eigen::Vector3d p, double coll_thr) const{
    return getDistanceAtPosition(p) - coll_thr;
}

void OctomapHandler::insertPointcloud(sensor_msgs::PointCloud2 pcd, geometry_msgs::Pose T_wc){
    auto start = std::chrono::system_clock::now();
    octomap::Pointcloud oct_cloud;
    pc2ToOctomap(pcd, oct_cloud);
    octomath::Pose6D oct_pose = geoPose2octPose(T_wc);
    ot_->insertPointCloud(oct_cloud, octomath::Vector3(0.0, 0.0, 0.0), oct_pose, max_range);
    edt_->update();
    auto end = std::chrono::system_clock::now();
    auto microsec = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
}

double OctomapHandler::castRay(Eigen::Vector3d origin, Eigen::Vector3d direction, double range, bool ignore_unknown){
    octomath::Vector3 endpoint;
    octomath::Vector3 origin_o = toOctVec(origin);
    octomath::Vector3 dir_o = toOctVec(direction);
    ot_->castRay(origin_o, dir_o, endpoint, ignore_unknown, range);
    return (endpoint - origin_o).norm();
}
}//namespace online_planner