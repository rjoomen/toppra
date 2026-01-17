#ifndef TOPPRA_CONSTRAINT_JOINT_TORQUE_TESSERACT_HPP
#define TOPPRA_CONSTRAINT_JOINT_TORQUE_TESSERACT_HPP

#include <tesseract_environment/environment.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_scene_graph/graph.h>

#include <toppra/constraint/joint_torque.hpp>

// KDL (Kinematics and Dynamics Library) is a dependency of Tesseract
// and provides inverse dynamics computation via RNEA algorithm
#include <kdl/tree.hpp>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>
#include <kdl/jntarray.hpp>

namespace toppra {
namespace constraint {
namespace jointTorque {

/** Implementation of JointTorque using Tesseract + KDL.
 * \extends JointTorque
 *
 * This class uses Tesseract for robot model management and KDL
 * (Kinematics and Dynamics Library) for inverse dynamics computation.
 *
 * KDL is already a dependency of Tesseract, making this a perfect fit
 * for dynamics calculations without additional dependencies.
 */
template<typename Environment = tesseract_environment::Environment>
class Tesseract;

template<typename _Environment>
class Tesseract : public JointTorque {
  public:
    typedef _Environment Environment;
    typedef std::shared_ptr<Environment> EnvironmentPtr;

    std::ostream& print(std::ostream& os) const
    {
      return JointTorque::print(os << "Tesseract-KDL - ");
    }

    /**
     * Computes inverse dynamics using KDL's RNEA algorithm.
     * tau = M(q)*a + C(q,v)*v + g(q)
     */
    void computeInverseDynamics (const Vector& q, const Vector& v, const Vector& a,
        Vector& tau)
    {
      if (!id_solver_) {
        throw std::runtime_error("Inverse dynamics solver not initialized");
      }

      // Convert Eigen vectors to KDL JntArray
      KDL::JntArray q_kdl(q.size());
      KDL::JntArray v_kdl(v.size());
      KDL::JntArray a_kdl(a.size());
      KDL::JntArray tau_kdl(tau.size());

      for (int i = 0; i < q.size(); ++i) {
        q_kdl(i) = q[i];
        v_kdl(i) = v[i];
        a_kdl(i) = a[i];
      }

      // Compute inverse dynamics
      // External wrenches (empty for now - can be extended)
      KDL::Wrenches f_ext(kdl_tree_.getNrOfSegments(), KDL::Wrench::Zero());

      int ret = id_solver_->CartToJnt(q_kdl, v_kdl, a_kdl, f_ext, tau_kdl);
      if (ret < 0) {
        throw std::runtime_error("KDL inverse dynamics computation failed");
      }

      // Convert back to Eigen
      for (int i = 0; i < tau.size(); ++i) {
        tau[i] = tau_kdl(i);
      }
    }

    /**
     * Constructor from Tesseract environment and KDL tree.
     *
     * \param env Tesseract environment
     * \param kdl_tree KDL tree structure of the robot
     * \param manipulator_name Name of the kinematic group
     * \param torque_limits Joint torque limits (optional, extracted from URDF if empty)
     * \param frictionCoeffs Friction coefficients
     */
    Tesseract (EnvironmentPtr env,
               const KDL::Tree& kdl_tree,
               const std::string& manipulator_name,
               const Vector& torque_limits = Vector(),
               const Vector& frictionCoeffs = Vector())
      : JointTorque (computeTorqueLimits(env, manipulator_name, torque_limits, true),
                     computeTorqueLimits(env, manipulator_name, torque_limits, false),
                     frictionCoeffs)
      , env_ (env)
      , kdl_tree_ (kdl_tree)
      , manipulator_name_ (manipulator_name)
    {
      initialize();
    }

    /// Move-assignment operator
    Tesseract (Tesseract&& other)
      : JointTorque(other)
      , env_ (std::move(other.env_))
      , kdl_tree_ (other.kdl_tree_)
      , id_solver_ (std::move(other.id_solver_))
      , manipulator_name_ (std::move(other.manipulator_name_))
    {}

    const EnvironmentPtr& environment() const { return env_; }

  private:
    void initialize()
    {
      // Create KDL inverse dynamics solver
      // Use default gravity vector (0, 0, -9.81)
      KDL::Vector gravity(0.0, 0.0, -9.81);
      id_solver_ = std::make_unique<KDL::TreeIdSolver_RNE>(kdl_tree_, gravity);
    }

    /**
     * Helper to extract torque limits from Tesseract environment
     */
    static Vector computeTorqueLimits(EnvironmentPtr env,
                                     const std::string& manipulator_name,
                                     const Vector& provided_limits,
                                     bool lower)
    {
      if (!provided_limits.empty()) {
        return lower ? -provided_limits : provided_limits;
      }

      // Extract from scene graph
      auto joint_names = env->getGroupJointNames(manipulator_name);
      Vector limits(joint_names.size());

      for (size_t i = 0; i < joint_names.size(); ++i) {
        auto joint = env->getSceneGraph()->getJoint(joint_names[i]);
        if (joint && joint->dynamics) {
          double effort_limit = joint->dynamics->effort;
          limits[i] = lower ? -effort_limit : effort_limit;
        } else {
          // Default if not specified
          limits[i] = lower ? -100.0 : 100.0;
        }
      }

      return limits;
    }

    EnvironmentPtr env_;
    KDL::Tree kdl_tree_;
    std::unique_ptr<KDL::TreeIdSolver_RNE> id_solver_;
    std::string manipulator_name_;
}; // class Tesseract

} // namespace jointTorque
} // namespace constraint
} // namespace toppra

#endif
