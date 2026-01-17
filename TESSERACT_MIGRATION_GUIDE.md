# Migrating from Pinocchio to Tesseract in TOPP-RA

This guide explains how to replace Pinocchio functionality with Tesseract in TOPP-RA's C++ implementation.

## Overview

TOPP-RA currently uses Pinocchio for:
1. **Inverse Dynamics** - Computing joint torques using RNEA algorithm
2. **Forward Kinematics & Velocities** - Computing Cartesian velocities for end-effector constraints

## Files to Modify/Create

### Files using Pinocchio:
- `cpp/src/toppra/constraint/joint_torque/pinocchio.hpp`
- `cpp/src/toppra/constraint/cartesian_velocity_norm/pinocchio.hpp`
- `cpp/CMakeLists.txt` (build configuration)
- `cpp/tests/test_constraints.cpp` (tests)

### New files to create:
- `cpp/src/toppra/constraint/joint_torque/tesseract.hpp`
- `cpp/src/toppra/constraint/cartesian_velocity_norm/tesseract.hpp`

## Detailed Migration Steps

### 1. Joint Torque Constraint (Inverse Dynamics)

**Pinocchio approach:**
```cpp
tau = pinocchio::rnea(model, data, q, v, a);
```

**Tesseract + KDL approach:**
Tesseract doesn't have built-in dynamics, but since KDL is already a dependency of Tesseract, we use it for inverse dynamics:
```cpp
// KDL is already available as a Tesseract dependency
KDL::TreeIdSolver_RNE id_solver(kdl_tree, gravity);
id_solver.CartToJnt(q_kdl, v_kdl, a_kdl, f_ext, tau_kdl);
```

This is a perfect fit - no additional dependencies needed!

### 2. Cartesian Velocity Constraint (Forward Kinematics)

**Pinocchio approach:**
```cpp
pinocchio::forwardKinematics(model, data, q, qdot);
v = pinocchio::getFrameVelocity(model, data, frame_id, reference_frame).toVector();
```

**Tesseract approach:**
```cpp
// Get kinematic group
auto kin_group = env->getKinematicGroup(manipulator_name);

// Compute Jacobian
Eigen::MatrixXd jacobian = kin_group->calcJacobian(q, link_name);

// Compute Cartesian velocity: v = J * qdot
Eigen::VectorXd v = jacobian * qdot;
```

## Implementation Structure

### Joint Torque Constraint with Tesseract + KDL

Since KDL is already a dependency of Tesseract, the implementation is straightforward:

```cpp
#include <tesseract_environment/environment.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <kdl/tree.hpp>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>

namespace toppra {
namespace constraint {
namespace jointTorque {

template<typename Environment = tesseract_environment::Environment>
class Tesseract : public JointTorque {
  public:
    void computeInverseDynamics(const Vector& q, const Vector& v, const Vector& a,
                                Vector& tau) override {
      // Convert Eigen to KDL
      KDL::JntArray q_kdl(q.size()), v_kdl(v.size()), a_kdl(a.size());
      for (int i = 0; i < q.size(); ++i) {
        q_kdl(i) = q[i];
        v_kdl(i) = v[i];
        a_kdl(i) = a[i];
      }

      // Compute inverse dynamics using KDL's RNEA
      KDL::JntArray tau_kdl(tau.size());
      KDL::Wrenches f_ext(kdl_tree_.getNrOfSegments(), KDL::Wrench::Zero());
      id_solver_->CartToJnt(q_kdl, v_kdl, a_kdl, f_ext, tau_kdl);

      // Convert back to Eigen
      for (int i = 0; i < tau.size(); ++i) {
        tau[i] = tau_kdl(i);
      }
    }

  private:
    std::shared_ptr<Environment> env_;
    KDL::Tree kdl_tree_;
    std::unique_ptr<KDL::TreeIdSolver_RNE> id_solver_;
};

} // namespace jointTorque
} // namespace constraint
} // namespace toppra
```

### For Cartesian Velocity Constraint with Tesseract:

```cpp
#include <tesseract_environment/environment.h>
#include <tesseract_kinematics/core/kinematic_group.h>

namespace toppra {
namespace constraint {
namespace cartesianVelocityNorm {

template<typename Environment = tesseract_environment::Environment>
class Tesseract : public CartesianVelocityNorm {
  public:
    void computeVelocity(const Vector& q, const Vector& qdot,
                        Vector& v) override {
      // Get Jacobian at current configuration
      Eigen::MatrixXd jacobian = kin_group_->calcJacobian(q, link_name_);

      // Compute Cartesian velocity
      v = jacobian * qdot;
    }

    Tesseract(std::shared_ptr<Environment> env,
              const std::string& manipulator_name,
              const std::string& link_name,
              const Matrix& S,
              const double& limit)
      : CartesianVelocityNorm(S, limit)
      , env_(env)
      , link_name_(link_name) {
      kin_group_ = env_->getKinematicGroup(manipulator_name);
    }

  private:
    std::shared_ptr<Environment> env_;
    tesseract_kinematics::KinematicGroup::UPtr kin_group_;
    std::string link_name_;
};

} // namespace cartesianVelocityNorm
} // namespace constraint
} // namespace toppra
```

## CMake Changes

Update `cpp/CMakeLists.txt`:

```cmake
# Replace:
option(BUILD_WITH_PINOCCHIO "Compile with Pinocchio library" OFF)

# With:
option(BUILD_WITH_TESSERACT "Compile with Tesseract library" OFF)

# Replace:
if(BUILD_WITH_PINOCCHIO)
    find_package(pinocchio REQUIRED)
    message(STATUS "Found pinocchio ${pinocchio_VERSION}")
endif()

# With:
if(BUILD_WITH_TESSERACT)
    find_package(tesseract_environment REQUIRED)
    find_package(tesseract_kinematics REQUIRED)
    find_package(tesseract_urdf REQUIRED)
    find_package(tesseract_scene_graph REQUIRED)
    # KDL is a dependency of Tesseract - just need kdl_parser for URDF parsing
    find_package(orocos_kdl REQUIRED)
    find_package(kdl_parser REQUIRED)
    message(STATUS "Found Tesseract with KDL")
endif()
```

## Recommendations

### Best Approach for TOPP-RA + Tesseract Integration:

Since **KDL is already a dependency of Tesseract**, the integration is seamless:

1. **For Cartesian Velocity Constraints**: Use Tesseract directly
   - Tesseract's Jacobian computation via `KinematicGroup::calcJacobian()`
   - No additional dependencies needed

2. **For Joint Torque Constraints**: Use Tesseract + KDL
   - KDL is already included with Tesseract
   - KDL's `TreeIdSolver_RNE` provides RNEA inverse dynamics
   - Just need to add `kdl_parser` for URDF to KDL tree conversion
   - Perfect fit - well-tested, ROS ecosystem compatible

### Recommended Approach:

- Use **Tesseract** for all kinematics (scene management, collision, path planning)
- Use **KDL** for inverse dynamics (already available as a Tesseract dependency)
- Single, cohesive framework - no mixed dependencies needed!

## Testing

Update test files to use Tesseract:
- Load URDF through Tesseract's URDF parser
- Create Environment and KinematicGroup objects
- Verify constraint computations match expected values

## Next Steps

1. The `tesseract.hpp` files are already implemented for both constraints
2. Update your CMake configuration to include Tesseract and kdl_parser
3. Build with `-DBUILD_WITH_TESSERACT=ON`
4. Test with your robot model
5. Enjoy seamless integration!

## Benefits of This Approach

1. **Single framework**: Tesseract + KDL work together seamlessly
2. **No additional dependencies**: KDL is already part of Tesseract
3. **ROS ecosystem compatible**: Both widely used in ROS
4. **Well-tested**: KDL's RNEA implementation is mature and reliable
5. **Clean integration**: Tesseract for planning, KDL for dynamics
