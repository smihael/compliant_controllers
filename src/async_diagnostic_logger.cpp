#include <compliant_controllers/async_diagnostic_logger.hpp>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace compliant_controllers {

AsyncDiagnosticLogger::~AsyncDiagnosticLogger() {
  stop();
}

bool AsyncDiagnosticLogger::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                      int num_joints) {
  stop();
  num_joints_ = num_joints;
  enabled_ = false;
  node->get_parameter("diagnostic_logger.duration", duration_s_);
  std::string requested_path;
  node->get_parameter("diagnostic_logger.log_file", requested_path);
  double log_filter_tag_value{0.0};
  node->get_parameter("diagnostic_logger.log_filter_tag", log_filter_tag_value);
  log_filter_tag_.store(static_cast<int>(log_filter_tag_value), std::memory_order_relaxed);

  if (requested_path.empty() || duration_s_ <= 0.0) {
    output_path_.clear();
    return true;
  }

  output_path_ = timestampedPath(requested_path);
  enabled_ = true;
  RCLCPP_INFO(node->get_logger(),
              "Diagnostic logger configured: file='%s', duration=%.3fs, log_filter_tag=%d",
              output_path_.c_str(), duration_s_, logFilterTag());
  return true;
}

void AsyncDiagnosticLogger::start() {
  if (!enabled_ || running_) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    stop_requested_ = false;
    start_stamp_s_ = 0.0;
  }
  running_ = true;
  writer_thread_ = std::thread(&AsyncDiagnosticLogger::writerLoop, this);
}

void AsyncDiagnosticLogger::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_requested_ = true;
  }
  condition_.notify_all();
  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }
  running_ = false;
}

void AsyncDiagnosticLogger::record(double stamp_s,
                                   const Eigen::VectorXd& q,
                                   const Eigen::VectorXd& dq,
                                   const Eigen::VectorXd& tau_measured,
                                   const Eigen::VectorXd& tau_commanded,
                                   const Eigen::VectorXd& gravity,
                                   const Eigen::VectorXd& coriolis,
                                   const Eigen::MatrixXd& inertia) {
  if (!enabled_ || !running_) {
    return;
  }
  if (q.size() != num_joints_ || dq.size() != num_joints_ ||
      tau_measured.size() != num_joints_ || tau_commanded.size() != num_joints_ ||
      gravity.size() != num_joints_ || coriolis.size() != num_joints_ ||
      inertia.rows() != num_joints_ || inertia.cols() != num_joints_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (start_stamp_s_ <= 0.0) {
    start_stamp_s_ = stamp_s;
  }
  if (stamp_s - start_stamp_s_ > duration_s_) {
    stop_requested_ = true;
    condition_.notify_one();
    return;
  }
  if (queue_.size() >= max_queue_size_) {
    queue_.pop_front();
  }
  queue_.push_back(Sample{
    stamp_s,
    logFilterTag(),
    q,
    dq,
    tau_measured,
    tau_commanded,
    gravity,
    coriolis,
    inertia,
  });
  condition_.notify_one();
}

std::string AsyncDiagnosticLogger::timestampedPath(const std::string& requested_path) {
  const auto now = std::chrono::system_clock::now();
  const auto now_time = std::chrono::system_clock::to_time_t(now);
  std::tm now_tm{};
  localtime_r(&now_time, &now_tm);

  std::ostringstream stamp;
  stamp << std::put_time(&now_tm, "%Y%m%d_%H%M%S");

  std::filesystem::path path(requested_path);
  const auto parent = path.parent_path();
  const auto stem = path.stem().string();
  const auto ext = path.extension().string();
  const auto filename = stem + "_" + stamp.str() + (ext.empty() ? ".csv" : ext);
  return (parent.empty() ? std::filesystem::path(filename) : parent / filename).string();
}

void AsyncDiagnosticLogger::writerLoop() {
  std::filesystem::path path(output_path_);
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }

  std::ofstream out(output_path_);
  if (!out) {
    return;
  }
  writeHeader(out);

  while (true) {
    Sample sample;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] { return stop_requested_ || !queue_.empty(); });
      if (queue_.empty() && stop_requested_) {
        break;
      }
      sample = std::move(queue_.front());
      queue_.pop_front();
    }
    writeSample(out, sample);
  }
}

void AsyncDiagnosticLogger::writeHeader(std::ofstream& out) const {
  out << "stamp_s,log_filter_tag";
  for (int i = 0; i < num_joints_; ++i) {
    out << ",q" << i;
  }
  for (int i = 0; i < num_joints_; ++i) {
    out << ",dq" << i;
  }
  for (int i = 0; i < num_joints_; ++i) {
    out << ",tau_measured" << i;
  }
  for (int i = 0; i < num_joints_; ++i) {
    out << ",tau_commanded" << i;
  }
  for (int i = 0; i < num_joints_; ++i) {
    out << ",gravity" << i;
  }
  for (int i = 0; i < num_joints_; ++i) {
    out << ",coriolis" << i;
  }
  for (int r = 0; r < num_joints_; ++r) {
    for (int c = 0; c < num_joints_; ++c) {
      out << ",inertia_" << r << "_" << c;
    }
  }
  out << "\n";
}

void AsyncDiagnosticLogger::writeSample(std::ofstream& out, const Sample& sample) const {
  out << std::setprecision(12) << sample.stamp_s << "," << sample.log_filter_tag;
  for (int i = 0; i < num_joints_; ++i) {
    out << "," << sample.q(i);
  }
  for (int i = 0; i < num_joints_; ++i) {
    out << "," << sample.dq(i);
  }
  for (int i = 0; i < num_joints_; ++i) {
    out << "," << sample.tau_measured(i);
  }
  for (int i = 0; i < num_joints_; ++i) {
    out << "," << sample.tau_commanded(i);
  }
  for (int i = 0; i < num_joints_; ++i) {
    out << "," << sample.gravity(i);
  }
  for (int i = 0; i < num_joints_; ++i) {
    out << "," << sample.coriolis(i);
  }
  for (int r = 0; r < num_joints_; ++r) {
    for (int c = 0; c < num_joints_; ++c) {
      out << "," << sample.inertia(r, c);
    }
  }
  out << "\n";
}

}  // namespace compliant_controllers
