#ifndef POSE_GRAPH_H__
#define POSE_GRAPH_H__

///////////////////
#include <algorithm>
#include <cassert>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <signal.h>
#include <sstream>
#include <string>

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

#include "g2o/config.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/estimate_propagator.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/optimization_algorithm_property.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace pose_graph {

class SolverG2O {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SolverG2O();
  ~SolverG2O();

  /**
   * @brief add a SE3 node to the graph
   * @param pose
   * @return registered node
   */
  g2o::VertexSE3 *add_se3_node(const Eigen::Isometry3d &pose);

  /**
   * @brief add a plane node to the graph
   * @param plane_coeffs
   * @return registered node
   */
  // g2o::VertexPlane* add_plane_node(const Eigen::Vector4d& plane_coeffs);

  /**
   * @brief add an edge between SE3 nodes
   * @param v1  node1
   * @param v2  node2
   * @param relative_pose  relative pose between node1 and node2
   * @param information_matrix  information matrix (it must be 6x6)
   * @return registered edge
   */
  g2o::EdgeSE3 *add_se3_edge(g2o::VertexSE3 *v1, g2o::VertexSE3 *v2,
                             const Eigen::Isometry3d &relative_pose,
                             const Eigen::MatrixXd &information_matrix);

  /**
 * @brief perform graph optimization
 */
  void optimize();

  /**
   * @brief save the pose graph
   * @param filename  output filename
   */
  void save(const std::string &filename);

public:
  std::unique_ptr<g2o::SparseOptimizer> graph; // g2o graph
  // g2o::VertexPlane* floor_plane_node;           // ground floor plane node

  bool is_first;
};
}

#endif // POSE_GRAPH_H__
