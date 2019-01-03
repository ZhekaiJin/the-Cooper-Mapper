#include "solver_g2o.h"
#include <ros/time.h>

// force linking to the g2o csparse solvers
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

namespace pose_graph {

/**
 * @brief constructor
 */
SolverG2O::SolverG2O() : is_first(true) {
  graph.reset(new g2o::SparseOptimizer());

  // choose solver
  std::string g2o_solver_name = "lm_var";
  std::cout << "construct solver... " << std::endl;
  g2o::OptimizationAlgorithmFactory *solver_factory =
      g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;

  graph->setVerbose(false);

  solver_factory->listSolvers(std::cout);
  g2o::Factory::instance()->printRegisteredTypes(std::cout, true);
  std::vector<std::string> kernels;
  g2o::RobustKernelFactory::instance()->fillKnownKernels(kernels);
  std::cout << "Robust Kernels:" << std::endl;
  for (size_t i = 0; i < kernels.size(); ++i) {
    std::cout << kernels[i] << std::endl;
  }

  g2o::OptimizationAlgorithm *solver =
      solver_factory->construct(g2o_solver_name, solver_property);
  graph->setAlgorithm(solver);
  if (!graph->solver()) {
    std::cerr << std::endl;
    std::cerr << "error : failed to allocate solver!!" << std::endl;
    solver_factory->listSolvers(std::cerr);
    std::cerr << "-------------" << std::endl;
    std::cin.ignore(1);
    return;
  }

  graph->solver()->printProperties(std::cout);
  std::cout << "done" << std::endl;
}

SolverG2O::~SolverG2O() { graph.reset(); }

g2o::VertexSE3 *SolverG2O::add_se3_node(const Eigen::Isometry3d &pose) {
  g2o::VertexSE3 *vertex(new g2o::VertexSE3());
  vertex->setId(graph->vertices().size());
  vertex->setEstimate(pose);
  if (is_first) {
    std::cout << "fixed pose:\n" << pose.matrix();
    vertex->setFixed(true);
    is_first = false;
  }
  graph->addVertex(vertex);

  return vertex;
}

g2o::EdgeSE3 *
SolverG2O::add_se3_edge(g2o::VertexSE3 *v1, g2o::VertexSE3 *v2,
                        const Eigen::Isometry3d &relative_pose,
                        const Eigen::MatrixXd &information_matrix) {
  g2o::EdgeSE3 *edge(new g2o::EdgeSE3());
  edge->setMeasurement(relative_pose);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  graph->addEdge(edge);

  return edge;
}

void SolverG2O::optimize() {

  std::cout << std::endl;
  std::cout << "--- g2o optimization ---" << std::endl;
  std::cout << "nodes: " << graph->vertices().size()
            << "   edges: " << graph->edges().size() << std::endl;

  graph->initializeOptimization();
  // graph->setVerbose(false);
  // double chi2 = graph->chi2();
  auto t1 = ros::Time::now();
  int iterations = graph->optimize(1000);
  auto t2 = ros::Time::now();
  std::cout << "iterations: " << iterations << std::endl;
  std::cout << "time: " << boost::format("%.3f") % (t2 - t1).toSec()
            << std::endl;
}

void SolverG2O::save(const std::string &filename) {
  std::ofstream ofs(filename);
  graph->save(ofs);
}

} // namespace
