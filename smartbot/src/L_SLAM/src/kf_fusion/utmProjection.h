//
// Created by tong on 17-11-14.
//

#ifndef HDL_GRABBER_UTMPROJECTION_H
#define HDL_GRABBER_UTMPROJECTION_H
#include <Eigen/Core>

#endif //HDL_GRABBER_UTMPROJECTION_H
void wgs2utm(double longitude, double latitude,double &x, double &y);
Eigen::Matrix<double, 3, 1> wgs2utm_proj4(Eigen::Matrix<double, 3, 1> wgs);
void wgs2utm_proj4(double longitude, double latitude,double attitude, double &x, double &y, double &z);
void wgs2utm_proj4(double longitude, double latitude,double &x, double &y);
void utm2wgs_proj4(double x, double y, double &longitude, double &latitude);

void test_HUACE();