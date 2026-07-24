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

  // Build model from URDF XML string. Optionally specify preferred end-effector frame name (ee_hint).
  // If ee_hint empty or not found, heuristics are applied; returns false on failure.
  bool init(const std::string& urdf_xml, const std::string& ee_hint = "");

  // Update internal kinematics with joint positions q (size must be >= dofs()).
  // Returns false if not initialized or size mismatch.
  bool update(const Eigen::Ref<const Eigen::VectorXd>& q);

  // Get end-effector pose. Returns false if not initialized. Outputs set only on success.
  bool getPose(Eigen::Vector3d& position, Eigen::Quaterniond& orientation) const;

  // Get end-effector Jacobian.  Returns false if not initialized.
  bool getJacobian(Eigen::Ref<Eigen::Matrix<double,6,Eigen::Dynamic>> J_out);

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
