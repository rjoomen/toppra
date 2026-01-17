#ifndef TOPPRA_CONSTRAINT_CARTESIAN_VELOCITY_NORM_TESSERACT_HPP
#define TOPPRA_CONSTRAINT_CARTESIAN_VELOCITY_NORM_TESSERACT_HPP

#include <tesseract_environment/environment.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_scene_graph/graph.h>

#include <toppra/constraint/cartesian_velocity_norm.hpp>

namespace toppra {
namespace constraint {
namespace cartesianVelocityNorm {

/** Implementation of CartesianVelocityNorm using Tesseract framework.
 * \extends CartesianVelocityNorm
 *
 * This class uses Tesseract's kinematic group to compute Jacobians
 * and subsequently Cartesian velocities for end-effector constraints.
 */
template<typename Environment = tesseract_environment::Environment>
class Tesseract;

template<typename _Environment>
class Tesseract : public CartesianVelocityNorm {
  public:
    typedef _Environment Environment;
    typedef std::shared_ptr<Environment> EnvironmentPtr;

    std::ostream& print(std::ostream& os) const
    {
      return CartesianVelocityNorm::print(os << "Tesseract - ");
    }

    /**
     * Computes Cartesian velocity using Tesseract's Jacobian computation.
     * v = J(q) * qdot
     */
    void computeVelocity (const Vector& q, const Vector& qdot, Vector& v)
    {
      // Update environment state
      tesseract_scene_graph::SceneState state;

      // Set joint positions
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        state.joints[joint_names_[i]] = q[i];
      }
      env_->setState(state.joints);

      // Compute Jacobian at current configuration
      Eigen::MatrixXd jacobian;
      if (!kin_group_->calcJacobian(jacobian, q, link_name_)) {
        throw std::runtime_error("Failed to compute Jacobian for link: " + link_name_);
      }

      // Compute Cartesian velocity: v = J * qdot
      v = jacobian * qdot;
    }

    /**
     * Constructor for constant velocity limit.
     *
     * \param env Tesseract environment containing the robot model
     * \param manipulator_name Name of the kinematic group/manipulator
     * \param link_name Name of the link/frame to track
     * \param S Selection matrix for velocity components
     * \param limit Velocity limit
     */
    Tesseract (EnvironmentPtr env,
               const std::string& manipulator_name,
               const std::string& link_name,
               const Matrix& S,
               const double& limit)
      : CartesianVelocityNorm (S, limit)
      , env_ (env)
      , manipulator_name_ (manipulator_name)
      , link_name_ (link_name)
    {
      initialize();
    }

    /// Move-assignment operator
    Tesseract (Tesseract&& other)
      : CartesianVelocityNorm(other)
      , env_ (std::move(other.env_))
      , kin_group_ (std::move(other.kin_group_))
      , manipulator_name_ (std::move(other.manipulator_name_))
      , link_name_ (std::move(other.link_name_))
      , joint_names_ (std::move(other.joint_names_))
    {}

    const EnvironmentPtr& environment() const { return env_; }
    const std::string& manipulatorName() const { return manipulator_name_; }
    const std::string& linkName() const { return link_name_; }

  protected:
    /**
     * Constructor for varying velocity limit.
     */
    Tesseract (EnvironmentPtr env,
               const std::string& manipulator_name,
               const std::string& link_name)
      : CartesianVelocityNorm ()
      , env_ (env)
      , manipulator_name_ (manipulator_name)
      , link_name_ (link_name)
    {
      initialize();
    }

  private:
    void initialize()
    {
      // Get kinematic group from environment
      if (!env_->hasGroup(manipulator_name_)) {
        throw std::runtime_error("Manipulator group not found: " + manipulator_name_);
      }

      auto group_joint_names = env_->getGroupJointNames(manipulator_name_);
      if (group_joint_names.empty()) {
        throw std::runtime_error("No joints found for manipulator: " + manipulator_name_);
      }

      // Store joint names for state updates
      joint_names_ = group_joint_names;

      // Get kinematic solver for the group
      kin_group_ = env_->getKinematicGroup(manipulator_name_);
      if (!kin_group_) {
        throw std::runtime_error("Failed to get kinematic group: " + manipulator_name_);
      }

      // Verify that the link exists
      if (!env_->getSceneGraph()->getLink(link_name_)) {
        throw std::runtime_error("Link not found in scene graph: " + link_name_);
      }
    }

    EnvironmentPtr env_;
    tesseract_kinematics::KinematicGroup::UPtr kin_group_;
    std::string manipulator_name_;
    std::string link_name_;
    std::vector<std::string> joint_names_;
}; // class Tesseract

} // namespace cartesianVelocityNorm
} // namespace constraint
} // namespace toppra

#endif
