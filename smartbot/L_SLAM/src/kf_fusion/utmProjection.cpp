#include "utmProjection.h"
#include <proj_api.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <strstream>
#include <Eigen/Dense>

void wgs2utm(double longitude, double latitude,double &x, double &y)
{

    double Lon0 = floor(longitude / 6) * 6 + 3; // reference longitude in degrees
    latitude = latitude * M_PI / 180;
    longitude = longitude * M_PI / 180;

    // WGS84 parameters
    double a = 6378137;           //semi - major axis
    // b = 6356752.314140;GRS80 value, originally used for WGS84 before refinements
    double b = 6356752.314245;    //semi - minor axis
    double e = sqrt(1 - pow((b / a), 2)); // eccentricity
    // UTM parameters
    double lon0 = Lon0*M_PI / 180;
    double k0 = 0.9996;  //scale on central meridian
    double FE = 500000; // false easting
    double FN = (latitude < 0) * 10000000; // false northing
    // Equations parameters
    double eps = pow(e, 2) / (1 - pow(e, 2));  // e prime square
    // N: radius of curvature of the earth perpendicular to meridian plane. Also, distance from point to polar axis
    double N = a / sqrt(1 - pow(e, 2) * pow(sin(latitude), 2));
    double T = pow(tan(latitude), 2);
    double C = (pow(e, 2) / (1 - pow(e, 2)))*pow(cos(latitude), 2);
    double A = ((longitude) - lon0)*cos(latitude);
    // M: true distance along the central meridian from the equator to lat
    double	M = a*((1 - pow(e, 2) / 4 - 3 * pow(e, 4) / 64 - 5 * pow(e, 6) / 256)*	(latitude)
                     - (3 * pow(e, 2) / 8 + 3 * pow(e, 4) / 32 + 45 * pow(e, 6) / 1024)*sin(2 * (latitude))
                     + (15 * pow(e, 4) / 256 + 45 * pow(e, 6) / 1024)*					sin(4 * (latitude))
                     - (35 * pow(e, 6) / 3072)*										sin(6 * (latitude)));
    // easting
    x = FE + k0*N*(A + (1 - T + C)*pow(A, 3) / 6 + (5 - 18 * T + pow(T, 2) + 72 * C - 58 * eps)*pow(A, 5) / 120);
    // northing
    y = FN + k0*M + k0*N*tan((latitude))*(pow(A, 2) / 2
                                          + (5 - T + 9 * C + 4 * pow(C, 2))*					pow(A, 4) / 24
                                          + (61 - 58 * T + pow(T, 2) + 600 * C - 330 * eps)*	pow(A, 6) / 720);
}

Eigen::Matrix<double, 3, 1> wgs2utm_proj4(Eigen::Matrix<double, 3, 1> wgs) {

    projPJ pj_utm, pj_latlong;

    int zone = int(wgs(0) / 6 + 0.5) + 31;
    std::stringstream iniSs;
    iniSs<<"+proj=utm +zone="<<zone<<" +ellps=WGS84 +units=m +no_defs";
    std::string iniStr = iniSs.str();

    pj_utm = pj_init_plus(iniStr.c_str());

    pj_latlong = pj_init_plus("+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs");

    double x = wgs(0);

    double y = wgs(1);

    double z = wgs(2);

    x *= DEG_TO_RAD;

    y *= DEG_TO_RAD;

    pj_transform(pj_latlong, pj_utm, 1, 1, &x, &y, NULL);

    return Eigen::Matrix<double, 3, 1>(x, y, z);

}

Eigen::Matrix<double, 3, 1> utm2wgs_proj4(Eigen::Matrix<double, 3, 1>
                                          utm) {

    projPJ pj_utm, pj_latlong;

    pj_utm = pj_init_plus("+proj=utm +zone=50 +ellps=WGS84 +units=m +no_defs");

    pj_latlong = pj_init_plus("+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs");

    double x = utm(0);

    double y = utm(1);

    double z = utm(2);

    pj_transform(pj_utm, pj_latlong, 1, 1, &x, &y, NULL);

    x *= RAD_TO_DEG;

    y *= RAD_TO_DEG;

    return Eigen::Matrix<double, 3, 1>(x, y, z);//lon lat

}
//lon, lat, east, north
void wgs2utm_proj4(double longitude, double latitude, double attitude, double &x, double &y, double &z)
{
    Eigen::Matrix<double, 3, 1> wgs(longitude, latitude, attitude);
    Eigen::Matrix<double, 3, 1> utm = wgs2utm_proj4(wgs);
    x = utm(0);
    y = utm(1);
    z = utm(2);
}

//lon, lat, east, north
void wgs2utm_proj4(double longitude, double latitude, double &x, double &y)
{
    double attitude = 0.0;
    Eigen::Matrix<double, 3, 1> wgs(longitude, latitude, attitude);
    Eigen::Matrix<double, 3, 1> utm = wgs2utm_proj4(wgs);
    x = utm(0);
    y = utm(1);
    double z = utm(2);
}

//east north lon lat
void utm2wgs_proj4(double x, double y, double &longitude, double &latitude)
{
    double height = 0.0;
    Eigen::Matrix<double, 3, 1> utm(x, y, height);
    Eigen::Matrix<double, 3, 1> wgs = utm2wgs_proj4(utm);
    longitude = wgs(0);
    latitude = wgs(1);

}

double planeDis(double x0, double y0, double x1, double y1)
{
    double dis = pow(x0 - x1, 2) + pow(y0 - y1, 2);
    return sqrt(dis);
}

double _3dDis(double x0, double y0, double z0, double x1, double y1, double z1)
{
    double dis = pow(x0 - x1, 2) + pow(y0 - y1, 2) + pow(z0 - z1, 2);
    return sqrt(dis);
}

void test_HUACE()
{
    double B_deg, B_min, B_sec;
    double L_deg, L_min, L_sec;
    double B, L, H;
    double x, y, z;

    B_deg = 39.0;
    B_min = 52.0;
    B_sec = 11.6138772;

    L_deg = 116.0;
    L_min = 10.0;
    L_sec = 52.1279212;

    H = 40.41371;

    B = B_deg + B_min / 60.0 + B_sec / 3600.0;
    L = L_deg + L_min / 60.0 + L_sec / 3600.0;

//4413829.920,430227.487,60.774,39.87164699N,116.18413641E,60.7736
    //31.28592603056	121.1941178778	13.6052
    B = 31.28592603056;
    L = 121.1941178778;
    H = 13.6052;

    std::cout<<std::fixed<<"B: "<<B<<" L: "<<L<<std::endl;

    wgs2utm_proj4(L, B, H, x, y, z);

    std::cout<<std::fixed<<"x: "<<x<<" y: "<<y<<" z: "<<z<<std::endl;

    //328102.8794089          3462697.50943101                   13.6052

    double x0 = 328102.8794089;
    double y0 = 3462697.50943101;
    double z0 = 13.6052;

    double x1 = 4414087.7324;
    double y1 = 429905.0042;
    double z1 = 66.6700;

    std::cout<<std::fixed<<"plane: "<<planeDis(x0, y0, x1, y1)<<" 3d: "<<_3dDis(x0, y0, z0, x1, y1, z1)<<std::endl;


}
