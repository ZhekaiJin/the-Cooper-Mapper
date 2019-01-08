
#include "graph.h"
#include <pcl/filters/voxel_grid.h>

namespace pose_graph {

Graph::Graph()
    : solver_g2o(new SolverG2O()), loop_detector(new LoopDetector()),
      keyframe_updater(new KeyframeUpdater()),
      max_keyframes_per_update(1), _laserCloudCornerLast2(new CloudI()),
      _laserCloudSurfLast2(new CloudI()), _laserCloudFullRes2(new CloudI()),
      _laserCloudFullResStack(new CloudI()), _newLaserCloudCornerLast2(false),
      _newLaserCloudSurfLast2(false), _newLaserCloudFullRes2(false),
      _newLaserOdometry2(false) {

  tf_odom2graph.setIdentity();
}

Graph::~Graph() {}

bool Graph::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode) {
  _odomAftGraph.header.frame_id = "/lidar_init";
  _odomAftGraph.child_frame_id = "/aft_graph";

  _subLaserCloudCornerLast2 = node.subscribe<sensor_msgs::PointCloud2>(
      "/laser_cloud_corner_last2", 2, &Graph::laserCloudCornerLastHandler,
      this);

  _subLaserCloudSurfLast2 = node.subscribe<sensor_msgs::PointCloud2>(
      "/laser_cloud_surf_last2", 2, &Graph::laserCloudSurfLastHandler, this);

  _subLaserOdometry2 = node.subscribe<nav_msgs::Odometry>(
      "/aft_mapped_to_init", 5, &Graph::laserOdometryHandler, this);

  _subLaserCloudFullRes2 = node.subscribe<sensor_msgs::PointCloud2>(
      "/velodyne_cloud_4", 2, &Graph::laserCloudFullResHandler, this);

  _pubOdomAftGraph =
      node.advertise<nav_msgs::Odometry>("/aft_graph_to_init", 1);

  _saveSrv = privateNode.advertiseService("saveGraph", &Graph::save, this);


  if (privateNode.getParam("filesDirectory", _filesDirectory)) {
    ROS_INFO("Set filesDirectory: %s", _filesDirectory.c_str());
  }

  optimize_thread = std::thread(&Graph::optimize, this);

  return true;
}

void Graph::laserCloudCornerLastHandler(
    const sensor_msgs::PointCloud2ConstPtr &cornerPointsLastMsg) {
  _timeLaserCloudCornerLast2 = cornerPointsLastMsg->header.stamp;

  _laserCloudCornerLast2->clear();
  pcl::fromROSMsg(*cornerPointsLastMsg, *_laserCloudCornerLast2);

  _newLaserCloudCornerLast2 = true;
}

void Graph::laserCloudSurfLastHandler(
    const sensor_msgs::PointCloud2ConstPtr &surfacePointsLastMsg) {
  _timeLaserCloudSurfLast2 = surfacePointsLastMsg->header.stamp;

  _laserCloudSurfLast2->clear();
  pcl::fromROSMsg(*surfacePointsLastMsg, *_laserCloudSurfLast2);

  _newLaserCloudSurfLast2 = true;
}

void Graph::laserCloudFullResHandler(
    const sensor_msgs::PointCloud2ConstPtr &laserCloudFullResMsg) {
  _timeLaserCloudFullRes2 = laserCloudFullResMsg->header.stamp;

  _laserCloudFullRes2->clear();
  pcl::fromROSMsg(*laserCloudFullResMsg, *_laserCloudFullRes2);

  _newLaserCloudFullRes2 = true;
}

void Graph::laserOdometryHandler(
    const nav_msgs::Odometry::ConstPtr &laserOdometry) {
  _timeLaserOdometry2 = laserOdometry->header.stamp;

  geometry_msgs::Quaternion q = laserOdometry->pose.pose.orientation;
  Eigen::Quaterniond equat(q.w, q.x, q.y, q.z);
  // std::cout<<"equat:"<<equat.coeffs()<<std::endl;

  geometry_msgs::Point v = laserOdometry->pose.pose.position;
  Eigen::Matrix3d rotation_matrix = equat.toRotationMatrix();
  Eigen::Vector3d v3d(v.x, v.y, v.z);
  tf_odomd.setIdentity();
  tf_odomd.translate(v3d);
  tf_odomd.rotate(rotation_matrix);

  _newLaserOdometry2 = true;

  // std::cout<<"tf_odomd:"<<tf_odomd.matrix()<<std::endl;
  // Eigen::Quaterniond quat(tf_odomd.rotation());
  // std::cout<<"tf_odomd quat:"<<quat.coeffs()<<"\n vec:"<<quat.vec()<<"
  // "<<quat.w()<<std::endl;
}

bool Graph::save(std_srvs::Empty::Request &req,
                 std_srvs::Empty::Response &resp) {
  ROS_INFO("Receive save request.");
  std::cout << "frame_count:" << keyframe_updater->get_frame_count()
            << std::endl;
  std::cout << "loop_count:" << loop_detector->get_loop_count() << std::endl;

  solver_g2o->save("/home/hr/lidar_slam/graph_before.g2o");
  solver_g2o->optimize();
  solver_g2o->save("/home/hr/lidar_slam/graph_end.g2o");

  const auto &keyframe = keyframes.back();
  Eigen::Isometry3d trans =
      keyframe->node->estimate() * keyframe->odom.inverse();
  tf_odom2graph_mutex.lock();
  tf_odom2graph = trans;
  tf_odom2graph_mutex.unlock();
  lidar_slam::FeatureMap<PointI> feature_map(121, 111, 121);
  std::string path = "/home/hr/lidar_slam/graph";
  feature_map.setupFilesDirectory(path);

  for (int i = 0; i < keyframes.size(); i++) {
    Eigen::Isometry3f estimate = keyframes[i]->node->estimate().cast<float>();
    PointI pos;
    pos.getVector3fMap() = estimate.translation();
    feature_map.update(pos);
    feature_map.addFeatureCloud(*keyframes[i]->cornerCloud,
                                *keyframes[i]->surfCloud, estimate);
  }
  feature_map.saveCloudToFiles();

  CloudIN cloud;
  generateGraphTrajectoryCloud(keyframes, cloud);
  saveTrajectoryCloud(cloud, "/home/hr/lidar_slam", "traj_graph");

  generateOdomTrajectoryCloud(keyframes, cloud);
  saveTrajectoryCloud(cloud, "/home/hr/lidar_slam", "traj_odom");

  getFinalFeatureMap();

  return true;
}

void Graph::getFinalFeatureMap() {
  lidar_slam::FeatureMap<PointI> feature_map2(121, 111, 121);
  feature_map2.setupFilterSize(0.2, 0.2, 0.4);

  std::string path2 = "/home/hr/lidar_slam/graph2";
  feature_map2.setupFilesDirectory(path2);

  CloudI::Ptr laserCloudCornerFromMap(new CloudI());
  CloudI::Ptr laserCloudSurfFromMap(new CloudI());

  CloudI::Ptr cloudCorner(new CloudI());
  CloudI::Ptr cloudSurf(new CloudI());
  lidar_slam::ScanMatch scan_match;

  pcl::VoxelGrid<PointI> downSizeFilterCorner;
  pcl::VoxelGrid<PointI> downSizeFilterSurf;
  downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
  downSizeFilterSurf.setLeafSize(0.3, 0.3, 0.3);

  int i = 0;
  int j = keyframes.size() / 40;

  for (const auto &frame : keyframes) {
    Eigen::Isometry3f estimate = frame->node->estimate().cast<float>();
    PointI pos;
    pos.getVector3fMap() = estimate.translation();
    feature_map2.update(pos);
    feature_map2.getSurroundFeature(*laserCloudCornerFromMap,
                                    *laserCloudSurfFromMap);

    downSizeFilterCorner.setInputCloud(frame->cornerCloud);
    downSizeFilterCorner.filter(*cloudCorner);

    downSizeFilterSurf.setInputCloud(frame->surfCloud);
    downSizeFilterSurf.filter(*cloudSurf);

    bool matched =
        scan_match.scanMatchScan(laserCloudCornerFromMap, laserCloudSurfFromMap,
                                 cloudCorner, cloudSurf, estimate);
    if (matched)
      feature_map2.addFeatureCloud(*frame->cornerCloud, *frame->surfCloud,
                                   estimate);
    i++;
    if (i >= j) {
      std::cout << "-";

      i = 0;
    }
  }
  feature_map2.saveCloudToFiles();
}

bool Graph::hasNewData() {
  return _newLaserCloudCornerLast2 && _newLaserCloudSurfLast2 &&
         _newLaserCloudFullRes2 && _newLaserOdometry2 &&
         fabs((_timeLaserCloudCornerLast2 - _timeLaserOdometry2).toSec()) <
             0.005 &&
         fabs((_timeLaserCloudSurfLast2 - _timeLaserOdometry2).toSec()) <
             0.005 &&
         fabs((_timeLaserCloudFullRes2 - _timeLaserOdometry2).toSec()) < 0.005;
}

void Graph::reset() {
  _newLaserCloudCornerLast2 = false;
  _newLaserCloudSurfLast2 = false;
  _newLaserCloudFullRes2 = false;
  _newLaserOdometry2 = false;
}

void Graph::spin() {
  ros::Rate rate(100);
  bool status = ros::ok();

  while (status) {
    ros::spinOnce();
    process();
    status = ros::ok();
    rate.sleep();
  }
}

void Graph::add_frame(KeyFrame::Ptr &keyframe) {
  if (!keyframe_updater->update(keyframe->odom)) {
    return;
  }
  double accum_d = keyframe_updater->get_accum_distance();
  keyframe->setAccumDistance(keyframe_updater->get_accum_distance());
  int frame_id = keyframe_updater->get_unique_id();
  keyframe->setFrameID(frame_id);
  // std::string name =
  //    std::to_string(frame_id) + "-" +
  //    std::to_string(keyframe->stamp.toSec());
  // saveTrajectoryCloud(*_laserCloudFullResStack, "/home/hr/.lidar/full",
  // name);
  std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
  keyframe_queue.push_back(keyframe);
}

bool Graph::flush_keyframe_queue() {
  std::lock_guard<std::mutex> lock(keyframe_queue_mutex);

  if (keyframe_queue.empty()) {
    return false;
  }

  tf_odom2graph_mutex.lock();
  Eigen::Isometry3d odom2map(tf_odom2graph);
  tf_odom2graph_mutex.unlock();

  int num_processed = 0;
  for (int i = 0;
       i < std::min<int>(keyframe_queue.size(), max_keyframes_per_update);
       i++) {
    num_processed = i;

    const auto &keyframe = keyframe_queue[i];
    new_keyframes.push_back(keyframe);

    Eigen::Isometry3d odom = odom2map * keyframe->odom;
    // Eigen::Isometry3d odom = keyframe->odom;
    keyframe->node = solver_g2o->add_se3_node(odom);

    if (i == 0 && keyframes.empty()) {
      continue;
    }

    // add edge between keyframes
    const auto &prev_keyframe =
        (i == 0 ? keyframes.back() : keyframe_queue[i - 1]);

    Eigen::Isometry3d relative_pose =
        prev_keyframe->odom.inverse() * keyframe->odom;
    Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6);

    for (int i = 0; i < 3; i++) {
      information(i, i) = 0.8;
      information(3 + i, 3 + i) = 1.0;
    }
    information(1, 1) = 0.4;
    information(4, 4) = 2;

    solver_g2o->add_se3_edge(prev_keyframe->node, keyframe->node, relative_pose,
                             information);

  }

  keyframe_queue.erase(keyframe_queue.begin(),
                       keyframe_queue.begin() + num_processed + 1);

  return true;
}

void Graph::process() {
  if (!hasNewData()) {
    return;
  }
  reset();

  KeyFrame::Ptr newkeyframe(new KeyFrame(_timeLaserOdometry2, tf_odomd,
                                         _laserCloudCornerLast2,
                                         _laserCloudSurfLast2));
  _laserCloudFullResStack.swap(_laserCloudFullRes2);
  add_frame(newkeyframe);
}

void Graph::optimize() {

  ros::Rate rate(100);
  bool status = ros::ok();

  while (status) {

    // add keyframes and floor coeffs in the queues to the pose graph
    if (!flush_keyframe_queue()) {
      status = ros::ok();
      rate.sleep();
      continue;
    }

    // loop detection
    std::vector<Loop::Ptr> loops;
    bool loop_found =
        loop_detector->detect_nearest(keyframes, new_keyframes, loops);
    for (const auto &loop : loops) {
      Eigen::Isometry3d relpose(loop->relative_pose.matrix().cast<double>());
      Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(6, 6);

      for (int i = 0; i < 3; i++) {
        information_matrix(i, i) = 2;
        information_matrix(3 + i, 3 + i) = 2;
      }
      solver_g2o->add_se3_edge(loop->key1->node, loop->key2->node, relpose,
                               information_matrix);
    }

    std::copy(new_keyframes.begin(), new_keyframes.end(),
              std::back_inserter(keyframes));
    new_keyframes.clear();

    // optimize the pose graph
    if (loop_found) {
      solver_g2o->optimize();
      int index = keyframes.size() - 1;
      Eigen::Isometry3d estimate = keyframes[index]->node->estimate();
      Eigen::Quaterniond q(estimate.rotation());
      Eigen::Vector3d v(estimate.translation());
      _odomAftGraph.header.stamp = keyframes[index]->stamp;
      _odomAftGraph.pose.pose.orientation.x = q.x();
      _odomAftGraph.pose.pose.orientation.y = q.y();
      _odomAftGraph.pose.pose.orientation.z = q.z();
      _odomAftGraph.pose.pose.orientation.w = q.w();
      _odomAftGraph.pose.pose.position.x = v(0);
      _odomAftGraph.pose.pose.position.y = v(1);
      _odomAftGraph.pose.pose.position.z = v(2);

      _pubOdomAftGraph.publish(_odomAftGraph);
    }

    // publish tf
    const auto &keyframe = keyframes.back();
    Eigen::Isometry3d trans =
        keyframe->node->estimate() * keyframe->odom.inverse();
    tf_odom2graph_mutex.lock();
    tf_odom2graph = trans;
    tf_odom2graph_mutex.unlock();

    status = ros::ok();
    rate.sleep();
  }
}
} // end namespace graph
