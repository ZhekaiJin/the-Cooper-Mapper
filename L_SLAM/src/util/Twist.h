#ifndef LIDAR_TWIST_H
#define LIDAR_TWIST_H

#include "Angle.h"
#include "Vector3.h"
#include <iostream>

namespace lidar_slam {

/** \brief Twist composed by three angles and a three-dimensional position.
 *
 */
class Twist {
public:
  Twist() : rot_x(), rot_y(), rot_z(), pos(){};

  Angle rot_x;
  Angle rot_y;
  Angle rot_z;
  Vector3 pos;

  void print() {
    std::cout << "twist:" << rot_x.rad() << " " << rot_y.rad() << " "
              << rot_z.rad() << " " << pos(0) << " " << pos(1) << " " << pos(2)
              << std::endl;
  }

  Twist operator*(const float scale) {
    Twist t;
    t.pos = pos * scale;
    t.rot_x = rot_x * scale;
    t.rot_y = rot_y * scale;
    t.rot_z = rot_z * scale;
    return t;
  }
};

} // end namespace lidar_slam

#endif // LIDAR_TWIST_H
