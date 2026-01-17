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

**Tesseract approach:**
Tesseract doesn't have a built-in RNEA implementation. You have two options:

#### Option A: Use Tesseract with external dynamics library
Combine Tesseract for kinematics with another library for dynamics:
- Use **KDL** (Kinematics and Dynamics Library) for dynamics
- Use **RBDL** (Rigid Body Dynamics Library)
- Use **Drake** for multibody dynamics

#### Option B: Implement custom inverse dynamics
If you have access to the mass matrix, Coriolis, and gravity terms:
```cpp
// Tesseract can provide kinematic information
// You'll need to compute: tau = M(q)*a + C(q,v)*v + g(q)
```

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

### For Joint Torque Constraint with Tesseract + KDL:

```cpp
#include <tesseract_environment/environment.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <kdl/treeinvsolver.hpp>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>

namespace toppra {
namespace constraint {
namespace jointTorque {

template<typename Environment = tesseract_environment::Environment>
class Tesseract : public JointTorque {
  public:
    void computeInverseDynamics(const Vector& q, const Vector& v, const Vector& a,
                                Vector& tau) override {
      // Convert to KDL types
      KDL::JntArray q_kdl, v_kdl, a_kdl, tau_kdl;
      // ... conversion code ...

      // Compute inverse dynamics using KDL
      id_solver_->CartToJnt(q_kdl, v_kdl, a_kdl,
                            KDL::Wrenches(), tau_kdl);

      // Convert back to Eigen
      // ... conversion code ...
    }

  private:
    std::shared_ptr<Environment> env_;
    std::shared_ptr<KDL::TreeIdSolver_RNE> id_solver_;
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
    # If using KDL for dynamics:
    find_package(orocos_kdl REQUIRED)
    message(STATUS "Found Tesseract")
endif()
```

## Recommendations

### Best Approach for TOPP-RA + Tesseract Integration:

1. **For Cartesian Velocity Constraints**: Use Tesseract directly (straightforward replacement)
   - Tesseract's Jacobian computation works well for this

2. **For Joint Torque Constraints**: Consider one of these options:
   - **Option A (Recommended)**: Tesseract + KDL for dynamics
     - Pros: KDL is well-tested, ROS ecosystem compatible
     - Cons: Additional dependency

   - **Option B**: Tesseract + Drake
     - Pros: Modern, well-maintained, comprehensive dynamics
     - Cons: Heavier dependency

   - **Option C**: Keep Pinocchio for dynamics only
     - Pros: Already working, efficient RNEA implementation
     - Cons: Mixed dependencies

### Suggested Hybrid Approach:

For minimal changes and maximum compatibility with Tesseract:
- Use **Tesseract** for all kinematics (scene management, collision, path planning)
- Use **KDL** for inverse dynamics (it's already a dependency in many ROS systems)
- Create Tesseract-based constraint classes that leverage both

## Testing

Update test files to use Tesseract:
- Load URDF through Tesseract's URDF parser
- Create Environment and KinematicGroup objects
- Verify constraint computations match expected values

## Next Steps

1. Choose your dynamics library (KDL recommended for ROS ecosystem)
2. Implement `tesseract.hpp` files for both constraints
3. Update CMake configuration
4. Create/update tests
5. Update documentation

## Questions to Consider

1. **Do you need full inverse dynamics?** Or just gravity compensation?
2. **Is KDL acceptable as a dependency?** It's common in ROS environments
3. **Do you need real-time performance?** This may influence library choice
4. **Will you use Tesseract's collision checking?** This could influence integration strategy
