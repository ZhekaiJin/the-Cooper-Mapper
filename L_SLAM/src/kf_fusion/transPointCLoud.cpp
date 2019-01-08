#include "transPointCLoud.h"

//DOF6 POSE FREOM IMU
void transMatrixContruct(Eigen::VectorXd pos,
                         Matrix4Xd &MTrans, Matrix4Xd &MTrans_inv)
{
    //imput pose
    //longitude,latitude,height,roll,pitch,yaw
    //output matrix * pt
    //pt.x = latitude
    //pt.y = longitude
    //pt.z = height

    Matrix4Xd trans = Matrix4Xd::Identity(4,4);
    Matrix4Xd trans_inv = Matrix4Xd::Identity(4,4);
    Matrix4Xd Rx = Matrix4Xd::Identity(4,4);
    Matrix4Xd Ry = Matrix4Xd::Identity(4,4);
    Matrix4Xd Rz = Matrix4Xd::Identity(4,4);

    //rotation
    double roll = pos[3] / 180.0 * M_PI, pitch = pos[4] / 180.0 * M_PI, yaw =  -pos[5]/ 180.0 * M_PI;
    Rx(1,1) =  cos(pitch); Rx(1,2) = -sin(pitch);
    Rx(2,1) =  sin(pitch); Rx(2,2) =  cos(pitch);
    Ry(0,0) =  cos(roll);                         Ry(0,2) =  sin(roll);
    Ry(2,0) = -sin(roll);                         Ry(2,2) =  cos(roll);
    Rz(0,0) =  cos(yaw); Rz(0,1) = -sin(yaw);
    Rz(1,0) =  sin(yaw); Rz(1,1) =  cos(yaw);

    trans = Rz * Ry * Rx;

    //transition
    trans(0,3) = pos[1];
    trans(1,3) = pos[0];
    trans(2,3) = pos[2];
    trans(3,3) = 1;

    MTrans = trans;
    MTrans_inv = trans.inverse();

    return;
}

void transMatrixContruct(std::vector<double> pos,
                         Matrix4Xd &MTrans, Matrix4Xd &MTrans_inv)
{
    Matrix4Xd trans = Matrix4Xd::Identity(4,4);
    Matrix4Xd trans_inv = Matrix4Xd::Identity(4,4);
    Matrix4Xd Rx = Matrix4Xd::Identity(4,4);
    Matrix4Xd Ry = Matrix4Xd::Identity(4,4);
    Matrix4Xd Rz = Matrix4Xd::Identity(4,4);

    //rotation
    double roll = pos[3] / 180.0 * M_PI, pitch = pos[4] / 180.0 * M_PI, yaw =  -pos[5]/ 180.0 * M_PI;
    Rx(1,1) =  cos(pitch); Rx(1,2) = -sin(pitch);
    Rx(2,1) =  sin(pitch); Rx(2,2) =  cos(pitch);
    Ry(0,0) =  cos(roll);                         Ry(0,2) =  sin(roll);
    Ry(2,0) = -sin(roll);                         Ry(2,2) =  cos(roll);
    Rz(0,0) =  cos(yaw); Rz(0,1) = -sin(yaw);
    Rz(1,0) =  sin(yaw); Rz(1,1) =  cos(yaw);

    trans = Rz * Ry * Rx;

    //transition
    trans(0,3) = pos[1];
    trans(1,3) = pos[0];
    trans(2,3) = pos[2];
    trans(3,3) = 1;

    MTrans = trans;
    MTrans_inv = trans.inverse();

    return;
}

void transMatrixContruct_novatel(std::vector<double> pos,
                                 Matrix4Xd &MTrans, Matrix4Xd &MTrans_inv)
{

    Matrix4Xd trans = Matrix4Xd::Identity(4,4);
    Matrix4Xd trans_inv = Matrix4Xd::Identity(4,4);

    trans(0,3) = pos[0];
    trans(1,3) = pos[1];
    trans(2,3) = pos[2];
    trans(3,3) = 1;

    double phi = pos[3] / 180.0 * M_PI, theta = pos[4] / 180.0 * M_PI, pusai =  pos[5]/ 180.0 * M_PI;
    trans(0,0) = cos(pusai)*cos(phi)-sin(pusai)*sin(theta)*sin(phi);	trans(0,1) = -sin(pusai)*cos(theta);	trans(0,2) = cos(pusai)*sin(phi)+sin(pusai)*sin(theta)*cos(phi);
    trans(1,0) = sin(pusai)*cos(phi)+cos(pusai)*sin(theta)*sin(phi);	trans(1,1) =  cos(pusai)*cos(theta);	trans(1,2) = sin(pusai)*sin(phi)-cos(pusai)*sin(theta)*cos(phi);;
    trans(2,0) = -cos(theta)*sin(phi);	                                trans(2,1) = sin(theta);	            trans(2,2) = cos(theta)*cos(phi);

    MTrans = trans;
    MTrans_inv = trans.inverse();

    return;
}

void transPointwithMatrix(double &ptIn_x, double &ptIn_y, double &ptIn_z, Matrix4Xd Trans)
{
    pcl::PointXYZI ptOut;
    //----------Matrix Multiply----------//
    /*MatrixXd Mpt_in(4,1), Mpt_out(4,1);
    Mpt_in(0,0) = ptIn.x;
    Mpt_in(1,0) = ptIn.y;
    Mpt_in(2,0) = ptIn.z;
    Mpt_in(3,0) = 1;

    Mpt_out = Trans * Mpt_in;

    ptOut.x = Mpt_out(0,0) / Mpt_out(3,0);
    ptOut.y = Mpt_out(1,0) / Mpt_out(3,0);
    ptOut.z = Mpt_out(2,0) / Mpt_out(3,0);*/

    //------------Point------------//速度更快！！！
    ptOut.x = ptIn_x * Trans(0,0) + ptIn_y * Trans(0,1) + ptIn_z * Trans(0,2) + Trans(0,3);
    ptOut.y = ptIn_x * Trans(1,0) + ptIn_y * Trans(1,1) + ptIn_z * Trans(1,2) + Trans(1,3);
    ptOut.z = ptIn_x * Trans(2,0) + ptIn_y * Trans(2,1) + ptIn_z * Trans(2,2) + Trans(2,3);

    ptIn_x = ptOut.x;
    ptIn_y = ptOut.y;
    ptIn_z = ptOut.z;

    return;
}


pcl::PointXYZI transPointwithMatrix(pcl::PointXYZI &ptIn, Matrix4Xd Trans)
{
    pcl::PointXYZI ptOut;
    //----------Matrix Multiply----------//
    /*MatrixXd Mpt_in(4,1), Mpt_out(4,1);
    Mpt_in(0,0) = ptIn.x;
    Mpt_in(1,0) = ptIn.y;
    Mpt_in(2,0) = ptIn.z;
    Mpt_in(3,0) = 1;

    Mpt_out = Trans * Mpt_in;

    ptOut.x = Mpt_out(0,0) / Mpt_out(3,0);
    ptOut.y = Mpt_out(1,0) / Mpt_out(3,0);
    ptOut.z = Mpt_out(2,0) / Mpt_out(3,0);*/

    ptOut.x = ptIn.x * Trans(0,0) + ptIn.y * Trans(0,1) + ptIn.z * Trans(0,2) + Trans(0,3);
    ptOut.y = ptIn.x * Trans(1,0) + ptIn.y * Trans(1,1) + ptIn.z * Trans(1,2) + Trans(1,3);
    ptOut.z = ptIn.x * Trans(2,0) + ptIn.y * Trans(2,1) + ptIn.z * Trans(2,2) + Trans(2,3);

    ptOut.intensity = ptIn.intensity;

    return ptOut;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr transCloudwithMatrix(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, Matrix4Xd Trans)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>);
    int numPts = cloudIn->points.size();
    if(numPts == 0)
        return cloudOut;
    for(unsigned i = 0 ; i < numPts ; i++)
    {
        cloudOut->points.push_back(transPointwithMatrix(cloudIn->points[i], Trans));
    }
    return cloudOut;
}
