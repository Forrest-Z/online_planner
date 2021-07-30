#include <mapping/octomap_handler.h>
#include <chrono>

namespace online_planner{

OctomapHandler::OctomapHandler(OctomapHandler::Param param): oct_res(param.oct_res), max_range(param.max_range), verbose(param.verbose){
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
    std::unique_lock<std::mutex> lock(octomap_mtx_);
    double dist = edt_->getDistance(p);
    return dist;
}

double OctomapHandler::getSafeDistanceAtPosition(Eigen::Vector3d p, double coll_thr) const{
    std::unique_lock<std::mutex> lock(octomap_mtx_);
    double safe_dist = getDistanceAtPosition(p) - coll_thr;
    return safe_dist;
}

void OctomapHandler::insertUpdate(pcl::PointCloud<pcl::PointXYZI> changed_set){
    auto start = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lock(octomap_mtx_);
    for(size_t i = 0; i < changed_set.points.size(); ++i){
        pcl::PointXYZI& pnt = changed_set.points[i];
        ot_->updateNode(ot_->coordToKey(pnt.x, pnt.y, pnt.z), pnt.intensity, false);
    }
    ot_->updateInnerOccupancy();
    edt_->update();
    auto end = std::chrono::system_clock::now();
    auto microsec = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
}

double OctomapHandler::castRay(Eigen::Vector3d origin, Eigen::Vector3d direction, double range, bool ignore_unknown){
    octomath::Vector3 endpoint;
    octomath::Vector3 origin_o = toOctVec(origin);
    octomath::Vector3 dir_o = toOctVec(direction);
    {
        std::unique_lock<std::mutex> lock(octomap_mtx_);
        ot_->castRay(origin_o, dir_o, endpoint, ignore_unknown, range);
    }
    return (endpoint - origin_o).norm();
}
}//namespace online_planner