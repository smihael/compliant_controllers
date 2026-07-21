#include <compliant_controllers/joint_impedance_impl.hpp>
#include <control/ControlCommand.hpp>
#include <control/ControlStates.hpp>
#include <ros2_control_robot_dynamics/robot_model.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

namespace {

struct Options {
  std::string urdf_xml_path;
  int iterations{10000};
  int warmup{1000};
  int joints{7};
};

void usage(const char* argv0) {
  std::cerr
    << "Usage: " << argv0 << " [--urdf-xml FILE] [--iterations N] [--warmup N] [--joints N]\n"
    << "\n"
    << "Measures model update + JointImpedanceImpl::step() without ros2_control/libfranka communication.\n"
    << "If --urdf-xml is omitted, the implementation runs without RobotModel coriolis.\n";
}

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto require_value = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << name << "\n";
        usage(argv[0]);
        std::exit(2);
      }
      return argv[++i];
    };

    if (arg == "--urdf-xml") {
      options.urdf_xml_path = require_value("--urdf-xml");
    } else if (arg == "--iterations") {
      options.iterations = std::stoi(require_value("--iterations"));
    } else if (arg == "--warmup") {
      options.warmup = std::stoi(require_value("--warmup"));
    } else if (arg == "--joints") {
      options.joints = std::stoi(require_value("--joints"));
    } else if (arg == "--help" || arg == "-h") {
      usage(argv[0]);
      std::exit(0);
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      usage(argv[0]);
      std::exit(2);
    }
  }
  if (options.iterations <= 0 || options.warmup < 0 || options.joints <= 0) {
    std::cerr << "Invalid non-positive iteration/joint count.\n";
    usage(argv[0]);
    std::exit(2);
  }
  return options;
}

std::string read_file(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open " + path);
  }
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

std::vector<std::string> franka_joint_names(const std::string& arm_id, int joints) {
  std::vector<std::string> names;
  names.reserve(static_cast<size_t>(joints));
  for (int i = 1; i <= joints; ++i) {
    names.push_back(arm_id + "_joint" + std::to_string(i));
  }
  return names;
}

double percentile_us(const std::vector<double>& sorted, double p) {
  if (sorted.empty()) {
    return 0.0;
  }
  const double idx = (p / 100.0) * static_cast<double>(sorted.size() - 1);
  const auto lo = static_cast<size_t>(idx);
  const auto hi = std::min(lo + 1, sorted.size() - 1);
  const double frac = idx - static_cast<double>(lo);
  return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
}

void print_stats(const std::string& label, std::vector<double> samples_us) {
  std::sort(samples_us.begin(), samples_us.end());
  const double sum = std::accumulate(samples_us.begin(), samples_us.end(), 0.0);
  const double mean = sum / static_cast<double>(samples_us.size());
  std::cout << "  " << label << "_min_us: " << samples_us.front() << "\n"
            << "  " << label << "_mean_us: " << mean << "\n"
            << "  " << label << "_p50_us: " << percentile_us(samples_us, 50.0) << "\n"
            << "  " << label << "_p90_us: " << percentile_us(samples_us, 90.0) << "\n"
            << "  " << label << "_p99_us: " << percentile_us(samples_us, 99.0) << "\n"
            << "  " << label << "_max_us: " << samples_us.back() << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  const auto options = parse_options(argc, argv);

  control::ControllerState state(options.joints);
  state.q.setZero();
  state.dq.setZero();
  state.tau.setZero();

  control::ControlCommand command(static_cast<size_t>(options.joints));
  command.joint_position = state.q;
  command.joint_velocity.setZero();
  command.joint_stiffness.setZero();
  command.joint_damping.setZero();
  command.joint_torque_ff.setZero();

  if (options.joints == 7) {
    command.joint_stiffness << 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0;
    command.joint_damping << 30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0;
    state.q << 0.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079633, 0.785398163;
    command.joint_position = state.q;
  }

  compliant_controllers::JointImpedanceImpl impl(options.joints);
  compliant_controllers::RobotModel robot_model;
  bool model_enabled = false;

  if (!options.urdf_xml_path.empty()) {
    const auto urdf_xml = read_file(options.urdf_xml_path);
    const auto joint_names = franka_joint_names("fr3", options.joints);
    if (!robot_model.init(urdf_xml, "fr3_link8", joint_names)) {
      std::cerr << "Failed to initialize RobotModel from " << options.urdf_xml_path << "\n";
      return 1;
    }
    impl.setRobotModel(static_cast<void*>(&robot_model));
    model_enabled = true;
  }

  Eigen::VectorXd tau_out = Eigen::VectorXd::Zero(options.joints);
  constexpr double dt = 0.001;
  const int total_iterations = options.warmup + options.iterations;
  std::vector<double> model_update_samples_us;
  std::vector<double> impl_step_samples_us;
  std::vector<double> total_samples_us;
  model_update_samples_us.reserve(static_cast<size_t>(options.iterations));
  impl_step_samples_us.reserve(static_cast<size_t>(options.iterations));
  total_samples_us.reserve(static_cast<size_t>(options.iterations));

  for (int i = 0; i < total_iterations; ++i) {
    state.q(0) = 0.05 * std::sin(static_cast<double>(i) * 0.001);
    state.dq(0) = 0.05 * std::cos(static_cast<double>(i) * 0.001);
    command.joint_position(0) = state.q(0);
    command.joint_velocity(0) = 0.0;

    const auto t0 = std::chrono::steady_clock::now();
    const auto t_model0 = std::chrono::steady_clock::now();
    if (model_enabled && !robot_model.update(state.q)) {
      std::cerr << "RobotModel update failed at iteration " << i << "\n";
      return 1;
    }
    const auto t_model1 = std::chrono::steady_clock::now();
    if (!impl.step(command, state, tau_out, dt)) {
      std::cerr << "JointImpedanceImpl step failed at iteration " << i << "\n";
      return 1;
    }
    const auto t1 = std::chrono::steady_clock::now();

    if (i >= options.warmup) {
      model_update_samples_us.push_back(
        std::chrono::duration<double, std::micro>(t_model1 - t_model0).count());
      impl_step_samples_us.push_back(
        std::chrono::duration<double, std::micro>(t1 - t_model1).count());
      total_samples_us.push_back(
        std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
  }

  std::vector<double> sorted_total = total_samples_us;
  std::sort(sorted_total.begin(), sorted_total.end());
  const double total_p99 = percentile_us(sorted_total, 99.0);

  std::cout << "joint_update_timing_test\n"
            << "  iterations: " << options.iterations << "\n"
            << "  warmup: " << options.warmup << "\n"
            << "  joints: " << options.joints << "\n"
            << "  robot_model: " << (model_enabled ? "enabled" : "disabled") << "\n";
  print_stats("model_update", model_update_samples_us);
  print_stats("impl_step", impl_step_samples_us);
  print_stats("total", total_samples_us);
  std::cout << "  total_p99_under_500us: " << (total_p99 < 500.0 ? "true" : "false") << "\n";

  return total_p99 < 500.0 ? 0 : 3;
}
