/**
 * Example: Using TOPP-RA with Tesseract Motion Planning Framework
 *
 * This example demonstrates how to use TOPP-RA's Tesseract-based constraints
 * for time-optimal path parameterization within the Tesseract ecosystem.
 */

#include <toppra/algorithm.hpp>
#include <toppra/algorithm/toppra.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>

#ifdef BUILD_WITH_TESSERACT
#include <toppra/constraint/cartesian_velocity_norm/tesseract.hpp>
#include <toppra/constraint/joint_torque/tesseract.hpp>

#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_scene_graph/graph.h>

// KDL is a dependency of Tesseract, used for inverse dynamics
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#endif

#include <iostream>
#include <fstream>

int main(int argc, char** argv) {
#ifdef BUILD_WITH_TESSERACT

  // ============================================================================
  // Step 1: Load robot model using Tesseract
  // ============================================================================

  std::string urdf_file = "path/to/your/robot.urdf";
  std::string srdf_file = "path/to/your/robot.srdf"; // Optional

  // Create Tesseract environment
  auto env = std::make_shared<tesseract_environment::Environment>();

  // Parse URDF
  tesseract_scene_graph::SceneGraph::Ptr scene_graph;
  tesseract_common::fs::path urdf_path(urdf_file);

  std::ifstream urdf_stream(urdf_file);
  std::string urdf_string((std::istreambuf_iterator<char>(urdf_stream)),
                          std::istreambuf_iterator<char>());

  auto resource_locator = std::make_shared<tesseract_common::SimpleResourceLocator>(
      [](const std::string& url) {
        return tesseract_common::SimpleLocatedResource::simpleLocatedResource(
            url, url, std::make_shared<tesseract_common::BytesResource>());
      });

  scene_graph = tesseract_urdf::parseURDFString(urdf_string, resource_locator);

  if (!scene_graph) {
    std::cerr << "Failed to parse URDF" << std::endl;
    return -1;
  }

  // Initialize environment
  if (!env->init(scene_graph)) {
    std::cerr << "Failed to initialize Tesseract environment" << std::endl;
    return -1;
  }

  std::cout << "Loaded robot model with " << scene_graph->getJoints().size()
            << " joints" << std::endl;

  // ============================================================================
  // Step 2: Define the geometric path (trajectory waypoints)
  // ============================================================================

  // Example: 6-DOF robot path
  int dof = 6;
  toppra::Matrix coefficients(4, dof);  // Cubic polynomial

  // Define a simple path: q(s) = a0 + a1*s + a2*s^2 + a3*s^3
  // Rows: [a0, a1, a2, a3]
  // Columns: one for each joint
  coefficients << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,      // a0: initial position
                  1.0, 1.0, 1.0, 1.0, 1.0, 1.0,      // a1: initial velocity
                  -2.0, -2.0, -2.0, -2.0, -2.0, -2.0, // a2
                  1.0, 1.0, 1.0, 1.0, 1.0, 1.0;      // a3

  toppra::Matrices path_coeffs = {coefficients};
  std::vector<double> breakpoints = {0.0, 1.0};  // Path parameter from 0 to 1

  toppra::PiecewisePolyPath path(path_coeffs, breakpoints);

  std::cout << "Created geometric path with DOF: " << path.dof() << std::endl;

  // ============================================================================
  // Step 3: Define constraints
  // ============================================================================

  toppra::LinearConstraintPtrs constraints;

  // 3a. Joint velocity constraints
  toppra::Vector vel_lower(dof), vel_upper(dof);
  vel_lower.setConstant(-1.0);  // rad/s
  vel_upper.setConstant(1.0);   // rad/s

  auto vel_constraint = std::make_shared<toppra::constraint::LinearJointVelocity>(
      vel_lower, vel_upper);
  constraints.push_back(vel_constraint);
  std::cout << "Added joint velocity constraints" << std::endl;

  // 3b. Joint acceleration constraints
  toppra::Vector accel_lower(dof), accel_upper(dof);
  accel_lower.setConstant(-2.0);  // rad/s^2
  accel_upper.setConstant(2.0);   // rad/s^2

  auto accel_constraint = std::make_shared<toppra::constraint::LinearJointAcceleration>(
      accel_lower, accel_upper);
  constraints.push_back(accel_constraint);
  std::cout << "Added joint acceleration constraints" << std::endl;

  // 3c. Cartesian velocity constraint using Tesseract
  std::string manipulator_name = "manipulator";  // Your manipulator group name
  std::string end_effector_link = "tool0";       // Your end-effector link name

  // Selection matrix: constrain only linear velocities (first 3 components)
  toppra::Matrix S(6, 6);
  S.setZero();
  S.block<3,3>(0, 0) = toppra::Matrix::Identity(3, 3);  // Linear velocity only

  double cartesian_vel_limit = 0.5;  // m/s

  auto cart_vel_constraint =
      std::make_shared<toppra::constraint::cartesianVelocityNorm::Tesseract<>>(
          env, manipulator_name, end_effector_link, S, cartesian_vel_limit);

  constraints.push_back(cart_vel_constraint);
  std::cout << "Added Cartesian velocity constraint (Tesseract)" << std::endl;

  // 3d. Joint torque constraint using Tesseract + KDL
  // KDL is already a dependency of Tesseract
  // First, create KDL tree from URDF
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromString(urdf_string, kdl_tree)) {
    std::cerr << "Failed to construct KDL tree" << std::endl;
    return -1;
  }

  toppra::Vector friction_coeffs(dof);
  friction_coeffs.setConstant(0.01);  // Small friction

  auto torque_constraint =
      std::make_shared<toppra::constraint::jointTorque::Tesseract<>>(
          env, kdl_tree, manipulator_name, toppra::Vector(), friction_coeffs);

  constraints.push_back(torque_constraint);
  std::cout << "Added joint torque constraint (Tesseract + KDL)" << std::endl;

  // ============================================================================
  // Step 4: Set up and run TOPP-RA algorithm
  // ============================================================================

  toppra::algorithm::TOPPRA algo(constraints, path);

  // Set discretization
  int N_gridpoints = 100;
  toppra::Vector gridpoints = toppra::Vector::LinSpaced(
      N_gridpoints, path.pathInterval()[0], path.pathInterval()[1]);

  algo.setN(N_gridpoints);

  // Compute parameterization
  toppra::ReturnCode ret = algo.computePathParametrization(0.0, 0.0);

  if (ret == toppra::ReturnCode::OK) {
    std::cout << "\n=== TOPP-RA computation successful! ===" << std::endl;

    // Get the parameterization
    auto parametrizer = algo.getParametrization();

    // Sample the time-parameterized trajectory
    int n_samples = 50;
    for (int i = 0; i < n_samples; ++i) {
      double s = i * (path.pathInterval()[1] - path.pathInterval()[0]) /
                 (n_samples - 1);

      // Get position, velocity, acceleration at path parameter s
      auto q = path.eval_single(s);
      auto qd = parametrizer->eval_single(s, 1);  // velocity
      auto qdd = parametrizer->eval_single(s, 2); // acceleration

      if (i == 0 || i == n_samples - 1) {
        std::cout << "s=" << s << ": q=[" << q.transpose() << "]" << std::endl;
      }
    }

    // Compute total trajectory duration
    double duration = parametrizer->pathInterval()[1] - parametrizer->pathInterval()[0];
    std::cout << "\nTrajectory duration: " << duration << " seconds" << std::endl;

  } else {
    std::cerr << "\nTOPP-RA computation failed with code: "
              << static_cast<int>(ret) << std::endl;
    return -1;
  }

  std::cout << "\n=== Example completed successfully ===" << std::endl;

#else
  std::cerr << "This example requires BUILD_WITH_TESSERACT=ON" << std::endl;
  return -1;
#endif

  return 0;
}
