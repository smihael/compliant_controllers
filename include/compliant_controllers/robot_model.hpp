#pragma once

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace compliant_controllers {

// Forward-only interface (PIMPL) hiding Pinocchio to reduce rebuild times.
// All heavy allocations happen in init(). Methods are non-allocating after success.
class RobotModel {
public:
  RobotModel();
  ~RobotModel();
  RobotModel(RobotModel&&) noexcept;
  RobotModel& operator=(RobotModel&&) noexcept;
  RobotModel(const RobotModel&) = delete;
  RobotModel& operator=(const RobotModel&) = delete;

  // Build model from URDF XML string. Optionally specify preferred end-effector frame name (ee_hint)
  // and the actuated joints used by the controller.
  // If ee_hint empty or not found, heuristics are applied; returns false on failure.
  bool init(const std::string& urdf_xml, const std::string& ee_hint = "",
            const std::vector<std::string>& controlled_joints = {});

  // Update internal kinematics with controlled joint positions q.
  // Returns false if not initialized or size mismatch.
  bool update(const Eigen::Ref<const Eigen::VectorXd>& q);

  // Get end-effector pose. Returns false if not initialized. Outputs set only on success.
  bool getPose(Eigen::Vector3d& position, Eigen::Quaterniond& orientation) const;

  // Get end-effector Jacobian.  Returns false if not initialized.
  bool getJacobian(Eigen::Ref<Eigen::Matrix<double,6,Eigen::Dynamic>> J_out);

  // Get end-effector Jacobian and its Moore-Penrose pseudo-inverse J^+.
  bool getJacobianAndPseudoInverse(Eigen::Ref<Eigen::Matrix<double,6,Eigen::Dynamic>> J_out,
                                   Eigen::Ref<Eigen::Matrix<double,Eigen::Dynamic,6>> J_pinv_out,
                                   double rcond = 1e-6);

  // Get gravity torques. Returns false if not initialized.
  bool getGravity(Eigen::Ref<Eigen::VectorXd> g_out);

  // Get Coriolis/centrifugal torques for the provided joint velocities.
  // Returns false if not initialized or size mismatch.
  bool getCoriolis(const Eigen::Ref<const Eigen::VectorXd>& dq,
                   Eigen::Ref<Eigen::VectorXd> c_out);

  // Get joint-space mass matrix in controlled-joint coordinates.
  // Returns false if not initialized or matrix size mismatch.
  bool getMassMatrix(Eigen::Ref<Eigen::MatrixXd> m_out);

  // Get combined mass + coriolis torque term: M(q)*ddq + c(q,dq).
  // Computed as rnea(q,dq,ddq) - g(q) in controlled-joint coordinates.
  // Returns false if not initialized or size mismatch.
  bool getMassCoriolisTorque(const Eigen::Ref<const Eigen::VectorXd>& dq,
                             const Eigen::Ref<const Eigen::VectorXd>& ddq,
                             Eigen::Ref<Eigen::VectorXd> mc_out);

  int nj;

  // Velocity DOF count (nv). Returns 0 if not initialized.
  inline int dofs() const noexcept { return nj; }
  std::string endEffectorFrame() const; // empty if not initialized

  bool valid() const; // initialized and last update succeeded

private:
  struct Impl; // defined in .cpp
  std::unique_ptr<Impl> impl_;
};

} // namespace compliant_controllers
