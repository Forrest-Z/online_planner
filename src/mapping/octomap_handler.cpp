#include <mapping/octomap_handler.h>
#include <chrono>
#include <octomap_msgs/conversions.h>

namespace online_planner{

OctomapHandler::OctomapHandler(OctomapHandler::Param param): oct_res(param.oct_res), max_range(param.max_range), 
verbose(param.verbose), undersample_rate(param.undersample_rate), is_perspective(param.is_perspective){
    bbxMax = param.bbxMax; bbxMin = param.bbxMin;
    sub_ot_ = std::unique_ptr<octomap::OcTree>(new octomap::OcTree(oct_res));
    sub_ot_->setBBXMax(bbxMax);
    sub_ot_->setBBXMin(bbxMin);
    sub_ot_->enableChangeDetection(true);
    pub_ot_ = std::unique_ptr<octomap::OcTree>(new octomap::OcTree(oct_res));
    pub_ot_->setBBXMax(bbxMax);
    pub_ot_->setBBXMin(bbxMin);
    pub_ot_->enableChangeDetection(true);
    edt_ = std::unique_ptr<DynamicEDTOctomap>(new DynamicEDTOctomap(max_range, pub_ot_.get(), bbxMin, bbxMax, false));
}
    

double OctomapHandler::getDistanceAtPosition(Eigen::Vector3d p) const{
    return getDistanceAtPosition(octomath::Vector3(p.x(), p.y(), p.z()));
}

double OctomapHandler::getDistanceAtPosition(octomath::Vector3 p) const{
    std::unique_lock<std::mutex> lock(pub_ot_mtx_);
    double dist = edt_->getDistance(p);
    return dist;
}

double OctomapHandler::getSafeDistanceAtPosition(Eigen::Vector3d p, double coll_thr) const{
    std::unique_lock<std::mutex> lock(pub_ot_mtx_);
    double safe_dist = getDistanceAtPosition(p) - coll_thr;
    return safe_dist;
}

void OctomapHandler::insertPointcloud(const sensor_msgs::ImageConstPtr img_msg, octomath::Pose6D T_wc_oct){
    std::unique_lock<std::mutex> lock(sub_ot_mtx_, std::defer_lock);
    sensor_msgs::PointCloud2::Ptr pcd_ptr(new sensor_msgs::PointCloud2);
    if(img_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        if(is_perspective) depth_to_pc_perspective<uint16_t>(img_msg, pcd_ptr, depthcam_model, max_range, undersample_rate, false); 
        else depth_to_pc<uint16_t>(img_msg, pcd_ptr, depthcam_model, max_range, undersample_rate, false); 
    }
    else{
        if(is_perspective) depth_to_pc_perspective<float>(img_msg, pcd_ptr, depthcam_model, max_range, undersample_rate, false); 
        else depth_to_pc<float>(img_msg, pcd_ptr, depthcam_model, max_range, undersample_rate, false);
    }
    octomap::Pointcloud oct_cloud;
    pc2ToOctomap(*pcd_ptr, oct_cloud);
    lock.lock();
    sub_ot_->insertPointCloud(oct_cloud, octomath::Vector3(0.0, 0.0, 0.0), T_wc_oct, max_range);
    lock.unlock();
}

pcl::PointCloud<pcl::PointXYZI> OctomapHandler::trackChanges(){
    std::unique_lock<std::mutex> lock(sub_ot_mtx_, std::defer_lock);
    octomap::KeyBoolMap::const_iterator startPnt = sub_ot_->changedKeysBegin();
    octomap::KeyBoolMap::const_iterator endPnt = sub_ot_->changedKeysEnd();
    pcl::PointCloud<pcl::PointXYZI> changedCells = pcl::PointCloud<pcl::PointXYZI>();
    int c = 0;
    for (octomap::KeyBoolMap::const_iterator iter = startPnt; iter != endPnt; ++iter) {
        ++c;
        octomap::OcTreeNode* node = sub_ot_->search(iter->first); 
        bool occupied = sub_ot_->isNodeOccupied(node); 
        octomap::point3d center = sub_ot_->keyToCoord(iter->first); 
        pcl::PointXYZI pnt;
        pnt.x = center(0);
        pnt.y = center(1);
        pnt.z = center(2);

        if (occupied) {
            pnt.intensity = 1000;
        }
        else {
        pnt.intensity = -1000;
        }

    changedCells.push_back(pnt);
    }

    if (c > 0){
        //ROS_DEBUG("[server] sending %d changed entries", (int)changedCells.size());
        sub_ot_->resetChangeDetection();
        //ROS_DEBUG("[server] octomap size after updating: %d", (int)sub_ot_->calcNumNodes());
    }
    return changedCells;
}

void OctomapHandler::insertUpdate(pcl::PointCloud<pcl::PointXYZI> changed_set){
    std::unique_lock<std::mutex> lock(pub_ot_mtx_);
    for(size_t i = 0; i < changed_set.points.size(); ++i){
        pcl::PointXYZI& pnt = changed_set.points[i];
        pub_ot_->updateNode(pub_ot_->coordToKey(pnt.x, pnt.y, pnt.z), pnt.intensity, false);
    }
    pub_ot_->updateInnerOccupancy();
    edt_->update();
    lock.unlock();
}

double OctomapHandler::castRay(Eigen::Vector3d origin, Eigen::Vector3d direction, double range, bool ignore_unknown){
    octomath::Vector3 endpoint;
    octomath::Vector3 origin_o = toOctVec(origin);
    octomath::Vector3 dir_o = toOctVec(direction);
    {
        std::unique_lock<std::mutex> lock(pub_ot_mtx_);
        pub_ot_->castRay(origin_o, dir_o, endpoint, ignore_unknown, range);
        lock.unlock();
    }
    return (endpoint - origin_o).norm();
}

void OctomapHandler::getOctomapMsg(octomap_msgs::Octomap& oct_msg){
    std::unique_lock<std::mutex> lock(pub_ot_mtx_);
    octomap_msgs::fullMapToMsg<octomap::OcTree>(*pub_ot_, oct_msg);
    lock.unlock();
}
}//namespace online_planner