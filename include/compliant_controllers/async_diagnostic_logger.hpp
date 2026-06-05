#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <iosfwd>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Eigen>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace compliant_controllers {

class AsyncDiagnosticLogger {
public:
  AsyncDiagnosticLogger() = default;
  ~AsyncDiagnosticLogger();

  bool configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                 int num_joints);
  void start();
  void stop();
  void setMode(int mode) { mode_.store(mode, std::memory_order_relaxed); }

  void record(double stamp_s,
              const Eigen::VectorXd& q,
              const Eigen::VectorXd& dq,
              const Eigen::VectorXd& tau_measured,
              const Eigen::VectorXd& tau_commanded,
              const Eigen::VectorXd& gravity,
              const Eigen::VectorXd& coriolis,
              const Eigen::MatrixXd& inertia);

  bool enabled() const { return enabled_; }
  const std::string& outputPath() const { return output_path_; }
  double duration() const { return duration_s_; }
  int mode() const { return mode_.load(std::memory_order_relaxed); }

private:
  struct Sample {
    double stamp_s{0.0};
    int mode{0};
    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    Eigen::VectorXd tau_measured;
    Eigen::VectorXd tau_commanded;
    Eigen::VectorXd gravity;
    Eigen::VectorXd coriolis;
    Eigen::MatrixXd inertia;
  };

  static std::string timestampedPath(const std::string& requested_path);

  void writerLoop();
  void writeHeader(std::ofstream& out) const;
  void writeSample(std::ofstream& out, const Sample& sample) const;

  bool enabled_{false};
  bool running_{false};
  bool stop_requested_{false};
  int num_joints_{0};
  double duration_s_{0.0};
  double start_stamp_s_{0.0};
  std::string output_path_;
  std::size_t max_queue_size_{4096};
  std::atomic<int> mode_{0};

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Sample> queue_;
  std::thread writer_thread_;
};

}  // namespace compliant_controllers
