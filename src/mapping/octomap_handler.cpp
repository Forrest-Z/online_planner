#include <mapping/octomap_handler.h>
#include <chrono>
#include <octomap_msgs/conversions.h>
#include <iostream>

using namespace std;

namespace online_planner{

OctomapHandler::OctomapHandler(OctomapHandler::Param param): oct_res(param.oct_res), max_range(param.max_range), 
 undersample_rate(param.undersample_rate), is_perspective(param.is_perspective), castray_timer(string("- raycasting timer : ")),
 insert_timer(string("- Pointcloud Insertion timer : ")), update_timer(string("- ChangeUpdate timer : ")){
    bbxMax = param.bbxMax; bbxMin = param.bbxMin;
    sub_ot_ = unique_ptr<octomap::OcTree>(new octomap::OcTree(oct_res));
    sub_ot_->setBBXMax(bbxMax);
    sub_ot_->setBBXMin(bbxMin);
    sub_ot_->enableChangeDetection(true);
    pub_ot_ = unique_ptr<octomap::OcTree>(new octomap::OcTree(oct_res));
    pub_ot_->setBBXMax(bbxMax);
    pub_ot_->setBBXMin(bbxMin);
    pub_ot_->enableChangeDetection(true);
    edt_ = unique_ptr<DynamicEDTOctomap>(new DynamicEDTOctomap(max_range, pub_ot_.get(), bbxMin, bbxMax, false)); //treat unknown as free
}
    
void OctomapHandler::printTimers(bool total, bool avg){
    cout<<"***OctomapHandler Timer Reports***"<<endl;
    unique_lock<mutex> lock_sub(sub_ot_mtx_);
    insert_timer.print(total, avg);
    lock_sub.unlock();

    unique_lock<mutex> lock_pub(pub_ot_mtx_);
    update_timer.print(total, avg);
    castray_timer.print(true, avg);
    lock_pub.unlock();
}

double OctomapHandler::getDistanceAtPosition(Eigen::Vector3d p) const{
    return getDistanceAtPosition(octomath::Vector3(p.x(), p.y(), p.z()));
}

double OctomapHandler::getDistanceAtPosition(octomath::Vector3 p) const{
    unique_lock<mutex> lock(pub_ot_mtx_);
    double dist = edt_->getDistance(p);
    lock.unlock();
    return dist;
}

double OctomapHandler::getSafeDistanceAtPosition(Eigen::Vector3d p, double coll_thr) const{
    unique_lock<mutex> lock(pub_ot_mtx_);
    double safe_dist = getDistanceAtPosition(p) - coll_thr;
    return safe_dist;
}

void OctomapHandler::insertPointcloud(const sensor_msgs::ImageConstPtr img_msg, octomath::Pose6D T_wc_oct){
    unique_lock<mutex> lock(sub_ot_mtx_, defer_lock);
    sensor_msgs::PointCloud2::Ptr pcd_ptr(new sensor_msgs::PointCloud2);
    if(img_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        if(is_perspective) depth_to_pc_perspective<uint16_t>(img_msg, pcd_ptr, depthcam_model, max_range+oct_res, undersample_rate, true); 
        else depth_to_pc<uint16_t>(img_msg, pcd_ptr, depthcam_model, max_range+oct_res, undersample_rate, true); 
    }
    else{
        if(is_perspective) depth_to_pc_perspective<float>(img_msg, pcd_ptr, depthcam_model, max_range+oct_res, undersample_rate, true); 
        else depth_to_pc<float>(img_msg, pcd_ptr, depthcam_model, max_range+oct_res, undersample_rate, true);
    }
    octomap::Pointcloud oct_cloud;
    pc2ToOctomap(*pcd_ptr, oct_cloud);
    lock.lock();
    insert_timer.tic();
    sub_ot_->insertPointCloud(oct_cloud, octomath::Vector3(0.0, 0.0, 0.0), T_wc_oct, max_range);
    insert_timer.toc();
    lock.unlock();
}

pcl::PointCloud<pcl::PointXYZI> OctomapHandler::trackChanges(){
    unique_lock<mutex> lock(sub_ot_mtx_);
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
    lock.unlock();
    return changedCells;
}

void OctomapHandler::insertUpdate(pcl::PointCloud<pcl::PointXYZI> changed_set){
    unique_lock<mutex> lock(pub_ot_mtx_);
    update_timer.tic();
    for(size_t i = 0; i < changed_set.points.size(); ++i){
        pcl::PointXYZI& pnt = changed_set.points[i];
        pub_ot_->updateNode(pub_ot_->coordToKey(pnt.x, pnt.y, pnt.z), pnt.intensity, false);
    }
    pub_ot_->updateInnerOccupancy();
    edt_->update();
    update_timer.toc();
    lock.unlock();
}

double OctomapHandler::castRay(Eigen::Vector3d origin, Eigen::Vector3d direction, double range, bool ignore_unknown){
    unique_lock<mutex> lock(pub_ot_mtx_, defer_lock);
    castray_timer.tic();
    octomath::Vector3 endpoint;
    octomath::Vector3 origin_o = toOctVec(origin);
    octomath::Vector3 dir_o = toOctVec(direction);
    lock.lock();
    pub_ot_->castRay(origin_o, dir_o, endpoint, ignore_unknown, range);
    castray_timer.toc();
    lock.unlock();
    return (endpoint - origin_o).norm();
}

// pair.first contain points to be casted. pair.second will be the placeholder for whether the point is occluded or not
void OctomapHandler::castRayGroup(Eigen::Vector3d p_query, const vector<Eigen::Vector3d>& points, vector<bool>& is_occluded, double d_margin, double d_max, bool ignore_unknown){
    unique_lock<mutex> lock(pub_ot_mtx_, defer_lock); //following getDistanceAtPosition will be blocked otherwise
    octomap::point3d p_ = toOctVec(p_query);
    double d_free = getDistanceAtPosition(p_query); //minimum free radius around p_query, which we don't want cast ray anymore
    d_free = 0.0 < d_free? d_free : 0.0;
    is_occluded.resize(points.size());
    lock.lock();
    for(int idx = 0; idx < points.size() ;++idx){
        Eigen::Vector3d point = points[idx];
        castray_timer.tic();
        double dist = (point - p_query).norm();
        Eigen::Vector3d dir = (point - p_query)/dist;
        if(dist > d_max){
            point = p_query + d_max*dir;
        }
        double range = dist - d_margin - d_free;
        octomap::point3d dir_oct = toOctVec(dir);
        octomap::point3d o = toOctVec(point - dir*d_margin);
        octomap::point3d endpoint;
        is_occluded[idx] = pub_ot_->castRay(o, dir_oct, endpoint, ignore_unknown, range); //true if occluded, false if free cell
        castray_timer.toc();
    }
    lock.unlock();
}

void OctomapHandler::getOctomapMsg(octomap_msgs::Octomap& oct_msg){
    unique_lock<mutex> lock(pub_ot_mtx_);
    octomap_msgs::fullMapToMsg<octomap::OcTree>(*pub_ot_, oct_msg);
    lock.unlock();
}
}//namespace online_planner