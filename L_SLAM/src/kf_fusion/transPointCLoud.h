#ifndef LOAM_VELODYNE_TRANSPOINTCLOUD_H
#define LOAM_VELODYNE_TRANSPOINTCLOUD_H

#endif //LOAM_VELODYNE_TRANSPOINTCLOUD_H

#include <iostream>
#include <fstream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace Eigen;
using namespace pcl;


void transMatrixContruct(Eigen::VectorXd pos,
                         Matrix4Xd &MTrans, Matrix4Xd &MTrans_inv);
void transMatrixContruct_novatel(std::vector<double> pos, Matrix4Xd &MTrans, Matrix4Xd &MTrans_inv);
void transMatrixContruct(std::vector<double> pos,
                         Matrix4Xd &MTrans, Matrix4Xd &MTrans_inv);
pcl::PointXYZI transPointwithMatrix(pcl::PointXYZI &ptIn, Matrix4Xd Trans);
void transPointwithMatrix(double &ptIn_x, double &ptIn_y, double &ptIn_z, Matrix4Xd Trans);
pcl::PointCloud<pcl::PointXYZI>::Ptr transCloudwithMatrix(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, Matrix4Xd Trans);
