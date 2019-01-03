#ifndef LOAD_EXTRINSIC_HPP_
#define LOAD_EXTRINSIC_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

inline bool loadExtrinsic(const std::string &file_path, Eigen::Isometry3d &extrinsic) {
  YAML::Node config = YAML::LoadFile(file_path);
  if (config["transform"]) {
    if (config["transform"]["matrix"]) {
      const YAML::Node &matrix = config["transform"]["matrix"];
      Eigen::Matrix4d m4d;
      if (matrix.size() != 16) {
        std::cout << "Transform::Matrix error" << matrix.size();
      }

      m4d << matrix[0].as<double>(), matrix[1].as<double>(),
          matrix[2].as<double>(), matrix[3].as<double>(),
          matrix[4].as<double>(), matrix[5].as<double>(),
          matrix[6].as<double>(), matrix[7].as<double>(),
          matrix[8].as<double>(), matrix[9].as<double>(),
          matrix[10].as<double>(), matrix[11].as<double>(),
          matrix[12].as<double>(), matrix[13].as<double>(),
          matrix[14].as<double>(), matrix[15].as<double>();

      extrinsic.matrix() = m4d;
      return true;
    }
  }
  return false;
}
#endif // LOAD_EXTRINSIC_HPP_
