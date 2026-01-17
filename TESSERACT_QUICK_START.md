# Quick Start: Replacing Pinocchio with Tesseract in TOPP-RA

## Summary

This guide provides a practical workflow for replacing Pinocchio with Tesseract in TOPP-RA.

## What Has Been Provided

1. **Migration Guide** (`TESSERACT_MIGRATION_GUIDE.md`): Comprehensive explanation of the migration
2. **Implementation Files**:
   - `cpp/src/toppra/constraint/cartesian_velocity_norm/tesseract.hpp` - Cartesian velocity constraints
   - `cpp/src/toppra/constraint/joint_torque/tesseract.hpp` - Joint torque constraints (with KDL)
3. **Example Code** (`examples/tesseract_integration_example.cpp`): Complete working example
4. **CMake Module** (`cpp/cmake/FindTesseract.cmake`): Helper for finding Tesseract packages

## Key Differences: Pinocchio vs Tesseract

| Feature | Pinocchio | Tesseract Equivalent |
|---------|-----------|---------------------|
| **Primary Focus** | Rigid body dynamics | Motion planning & kinematics |
| **Inverse Dynamics** | Built-in RNEA | Requires KDL or Drake |
| **Jacobian** | `pinocchio::computeJacobian()` | `KinematicGroup::calcJacobian()` |
| **Forward Kinematics** | `pinocchio::forwardKinematics()` | `KinematicGroup::calcFwdKin()` |
| **Model Loading** | `pinocchio::urdf::buildModel()` | `tesseract_urdf::parseURDFString()` |

## Recommended Approach

### Option 1: Tesseract + KDL (Recommended for ROS)

**Pros:**
- KDL is widely used in ROS ecosystem
- Lightweight dependency
- Well-tested inverse dynamics
- Compatible with MoveIt and Tesseract

**Cons:**
- Need to maintain both Tesseract and KDL trees

**Use case:** If you're already using ROS or need ROS compatibility

### Option 2: Tesseract Only (Kinematics Only)

**Pros:**
- Single dependency
- Clean integration with Tesseract ecosystem

**Cons:**
- No inverse dynamics (can't use joint torque constraints)
- Limited to kinematic constraints

**Use case:** If you only need Cartesian velocity constraints

### Option 3: Hybrid (Tesseract + Pinocchio)

**Pros:**
- Use Tesseract for planning, Pinocchio for dynamics
- Minimal code changes
- Best of both worlds

**Cons:**
- Multiple dependencies
- Need to keep models synchronized

**Use case:** If you need both Tesseract's planning features and efficient dynamics

## Build Instructions

### Install Dependencies

#### For Tesseract:
```bash
# ROS 2 users (recommended):
sudo apt install ros-${ROS_DISTRO}-tesseract-environment \
                 ros-${ROS_DISTRO}-tesseract-kinematics \
                 ros-${ROS_DISTRO}-tesseract-urdf \
                 ros-${ROS_DISTRO}-tesseract-scene-graph

# Or from source:
git clone https://github.com/tesseract-robotics/tesseract.git
cd tesseract
colcon build
```

#### For KDL (if using joint torque constraints):
```bash
# ROS 2 users:
sudo apt install ros-${ROS_DISTRO}-orocos-kdl \
                 ros-${ROS_DISTRO}-kdl-parser

# Or:
sudo apt install liborocos-kdl-dev
```

### Build TOPP-RA with Tesseract

```bash
cd toppra/cpp
mkdir -p build && cd build

# Build with Tesseract support:
cmake .. \
  -DBUILD_WITH_TESSERACT=ON \
  -DBUILD_WITH_KDL=ON \
  -DCMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO}

make -j$(nproc)
```

### CMakeLists.txt Changes Needed

Add to your `cpp/CMakeLists.txt`:

```cmake
# Add Tesseract option
option(BUILD_WITH_TESSERACT "Compile with Tesseract library" OFF)
option(BUILD_WITH_KDL "Compile with KDL for dynamics" OFF)

# Find packages
if(BUILD_WITH_TESSERACT)
  find_package(tesseract_environment REQUIRED)
  find_package(tesseract_kinematics REQUIRED)
  find_package(tesseract_urdf REQUIRED)
  find_package(tesseract_scene_graph REQUIRED)
  message(STATUS "Found Tesseract")

  if(BUILD_WITH_KDL)
    find_package(orocos_kdl REQUIRED)
    find_package(kdl_parser REQUIRED)
    message(STATUS "Found KDL for dynamics")
  endif()
endif()
```

Add to `cpp/src/CMakeLists.txt`:

```cmake
if(BUILD_WITH_TESSERACT)
  target_link_libraries(toppra PUBLIC
    tesseract::tesseract_environment
    tesseract::tesseract_kinematics
    tesseract::tesseract_scene_graph
    tesseract::tesseract_urdf
  )

  if(BUILD_WITH_KDL)
    target_link_libraries(toppra PUBLIC
      orocos-kdl
      kdl_parser
    )
    target_compile_definitions(toppra PUBLIC BUILD_WITH_KDL)
  endif()

  target_compile_definitions(toppra PUBLIC BUILD_WITH_TESSERACT)
endif()
```

## Usage Example

```cpp
#include <tesseract_environment/environment.h>
#include <toppra/constraint/cartesian_velocity_norm/tesseract.hpp>

// Create Tesseract environment
auto env = std::make_shared<tesseract_environment::Environment>();

// Load robot
auto scene_graph = tesseract_urdf::parseURDFFile("robot.urdf");
env->init(scene_graph);

// Create constraint
toppra::Matrix S = toppra::Matrix::Identity(6, 6);
auto constraint =
    std::make_shared<toppra::constraint::cartesianVelocityNorm::Tesseract<>>(
        env, "manipulator", "end_effector", S, 0.5);

// Use with TOPP-RA
toppra::algorithm::TOPPRA algo({constraint}, path);
```

## Testing Your Implementation

Run the example:
```bash
cd build
./examples/tesseract_integration_example /path/to/robot.urdf
```

Run existing tests with Tesseract:
```bash
cd build
./tests/all_tests --gtest_filter="*Tesseract*"
```

## Troubleshooting

### Issue: "tesseract_environment not found"
**Solution:** Make sure Tesseract is installed and CMAKE_PREFIX_PATH is set:
```bash
export CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO}:$CMAKE_PREFIX_PATH
```

### Issue: "No kinematic group found"
**Solution:** Check your SRDF file defines the manipulator group, or create it programmatically:
```cpp
env->addKinematicGroup("manipulator", joint_names, "base_link", "tool0");
```

### Issue: Link errors with KDL
**Solution:** Make sure to link both KDL libraries:
```cmake
target_link_libraries(your_target orocos-kdl kdl_parser)
```

## Next Steps

1. Review the migration guide for detailed explanations
2. Examine the example code in `examples/tesseract_integration_example.cpp`
3. Modify CMakeLists.txt as shown above
4. Test with your robot model
5. Consider contributing back improvements!

## Comparison: Code Changes

### Before (Pinocchio):
```cpp
#include <toppra/constraint/joint_torque/pinocchio.hpp>

pinocchio::Model model;
pinocchio::urdf::buildModel(urdf_file, model);

auto constraint =
    std::make_shared<toppra::constraint::jointTorque::Pinocchio<>>(
        model, friction_coeffs);
```

### After (Tesseract + KDL):
```cpp
#include <toppra/constraint/joint_torque/tesseract.hpp>

auto env = std::make_shared<tesseract_environment::Environment>();
auto scene_graph = tesseract_urdf::parseURDFFile(urdf_file);
env->init(scene_graph);

KDL::Tree kdl_tree;
kdl_parser::treeFromFile(urdf_file, kdl_tree);

auto constraint =
    std::make_shared<toppra::constraint::jointTorque::Tesseract<>>(
        env, kdl_tree, "manipulator", torque_limits, friction_coeffs);
```

## Support

- Check `TESSERACT_MIGRATION_GUIDE.md` for detailed technical information
- Review Tesseract documentation: https://tesseract-docs.readthedocs.io/
- TOPP-RA issues: https://github.com/hungpham2511/toppra/issues
