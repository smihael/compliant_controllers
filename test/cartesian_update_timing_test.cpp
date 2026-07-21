#include <control/AbstractController.hpp>
#include <control/ControlCommand.hpp>
#include <control/ControlStates.hpp>
#include <ros2_control_robot_dynamics/robot_model.hpp>

#include <dlfcn.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

namespace {

struct Options {
  std::string urdf_xml_path;
  std::string impl_library{"libcartesian_impedance_impl.so"};
  int iterations{10000};
  int warmup{1000};
  int joints{7};
};

void usage(const char* argv0) {
  std::cerr
    << "Usage: " << argv0
    << " --urdf-xml FILE [--impl-library FILE] [--iterations N] [--warmup N] [--joints N]\n"
    << "\n"
    << "Measures dynamically loaded plugin step() and the wrapper compute path without "
       "ros2_control/libfranka communication.\n";
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
    } else if (arg == "--impl-library") {
      options.impl_library = require_value("--impl-library");
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
  if (options.urdf_xml_path.empty() || options.iterations <= 0 ||
      options.warmup < 0 || options.joints <= 0) {
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
  const double sum = std::accumulate(samples_us.begin(), samples_us.end(), 0.0);
  const double mean = sum / static_cast<double>(samples_us.size());
  const double squared_deviation_sum = std::accumulate(
    samples_us.begin(), samples_us.end(), 0.0,
    [mean](double sum_so_far, double sample) {
      const double deviation = sample - mean;
      return sum_so_far + deviation * deviation;
    });
  const double stddev = std::sqrt(
    squared_deviation_sum / static_cast<double>(samples_us.size()));
  std::sort(samples_us.begin(), samples_us.end());
  std::cout << "  " << label << "_min_us: " << samples_us.front() << "\n"
            << "  " << label << "_mean_us: " << mean << "\n"
            << "  " << label << "_stddev_us: " << stddev << "\n"
            << "  " << label << "_p50_us: " << percentile_us(samples_us, 50.0) << "\n"
            << "  " << label << "_p90_us: " << percentile_us(samples_us, 90.0) << "\n"
            << "  " << label << "_p99_us: " << percentile_us(samples_us, 99.0) << "\n"
            << "  " << label << "_max_us: " << samples_us.back() << "\n";
}

using CreateControllerFn = control::AbstractController* (*)(int);
using DestroyControllerFn = void (*)(control::AbstractController*);
using ControllerNameFn = const char* (*)();

struct DlCloser {
  void operator()(void* handle) const noexcept {
    if (handle != nullptr) {
      dlclose(handle);
    }
  }
};

struct ControllerDeleter {
  DestroyControllerFn destroy{};
  void operator()(control::AbstractController* controller) const noexcept {
    if (controller != nullptr && destroy != nullptr) {
      destroy(controller);
    }
  }
};

}  // namespace

int main(int argc, char** argv) {
  const auto options = parse_options(argc, argv);
  const auto urdf_xml = read_file(options.urdf_xml_path);
  const auto joint_names = franka_joint_names("fr3", options.joints);

  compliant_controllers::RobotModel robot_model;
  if (!robot_model.init(urdf_xml, "fr3_link8", joint_names)) {
    std::cerr << "Failed to initialize RobotModel from " << options.urdf_xml_path << "\n";
    return 1;
  }

  control::ControllerState state(options.joints);
  state.q.setZero();
  state.dq.setZero();
  state.tau.setZero();
  if (options.joints == 7) {
    state.q << 0.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079633, 0.785398163;
  }

  if (!robot_model.update(state.q) ||
      !robot_model.getPose(state.position, state.orientation)) {
    std::cerr << "Failed to seed RobotModel pose.\n";
    return 1;
  }

  control::ControlCommand command(static_cast<size_t>(options.joints));
  command.position = state.position;
  command.orientation = state.orientation;
  command.velocity.setZero();
  command.wrench.setZero();
  command.stiffness.setZero();
  command.damping.setZero();
  command.stiffness.topLeftCorner(3, 3) = 200.0 * Eigen::Matrix3d::Identity();
  command.stiffness.bottomRightCorner(3, 3) = 10.0 * Eigen::Matrix3d::Identity();
  command.damping.topLeftCorner(3, 3) = 2.0 * std::sqrt(200.0) * Eigen::Matrix3d::Identity();
  command.damping.bottomRightCorner(3, 3) = 2.0 * std::sqrt(10.0) * Eigen::Matrix3d::Identity();
  command.q_ns_des = state.q;
  command.k_ns = 5.0 * Eigen::VectorXd::Ones(options.joints);
  command.d_ns = 2.0 * std::sqrt(5.0) * Eigen::VectorXd::Ones(options.joints);
  command.tau_ff.setZero();

  std::unique_ptr<void, DlCloser> library(dlopen(options.impl_library.c_str(), RTLD_NOW | RTLD_LOCAL));
  if (!library) {
    std::cerr << "Failed to load implementation library '" << options.impl_library
              << "': " << dlerror() << "\n";
    return 1;
  }
  const auto create_controller =
    reinterpret_cast<CreateControllerFn>(dlsym(library.get(), "create_controller"));
  const auto destroy_controller =
    reinterpret_cast<DestroyControllerFn>(dlsym(library.get(), "destroy_controller"));
  const auto controller_name =
    reinterpret_cast<ControllerNameFn>(dlsym(library.get(), "controller_name"));
  if (!create_controller || !destroy_controller || !controller_name) {
    std::cerr << "Implementation library is missing controller factory symbols.\n";
    return 1;
  }
  std::unique_ptr<control::AbstractController, ControllerDeleter> impl(
    create_controller(options.joints), ControllerDeleter{destroy_controller});
  if (!impl) {
    std::cerr << "Implementation factory returned null.\n";
    return 1;
  }
  impl->setRobotModel(static_cast<void*>(&robot_model));

  Eigen::VectorXd tau_out = Eigen::VectorXd::Zero(options.joints);
  constexpr double dt = 0.001;
  const int total_iterations = options.warmup + options.iterations;
  std::vector<double> model_update_samples_us;
  std::vector<double> pose_samples_us;
  std::vector<double> impl_step_samples_us;
  std::vector<double> total_samples_us;
  model_update_samples_us.reserve(static_cast<size_t>(options.iterations));
  pose_samples_us.reserve(static_cast<size_t>(options.iterations));
  impl_step_samples_us.reserve(static_cast<size_t>(options.iterations));
  total_samples_us.reserve(static_cast<size_t>(options.iterations));

  for (int i = 0; i < total_iterations; ++i) {
    state.q(0) = 0.05 * std::sin(static_cast<double>(i) * 0.001);
    state.dq(0) = 0.05 * std::cos(static_cast<double>(i) * 0.001);

    const auto t0 = std::chrono::steady_clock::now();
    if (!robot_model.update(state.q)) {
      std::cerr << "RobotModel update failed at iteration " << i << "\n";
      return 1;
    }
    const auto t_model1 = std::chrono::steady_clock::now();
    if (!robot_model.getPose(state.position, state.orientation)) {
      std::cerr << "RobotModel pose failed at iteration " << i << "\n";
      return 1;
    }
    const auto t_pose1 = std::chrono::steady_clock::now();
    if (!impl->step(command, state, tau_out, dt)) {
      std::cerr << "CartesianImpedanceImpl step failed at iteration " << i << "\n";
      return 1;
    }
    const auto t1 = std::chrono::steady_clock::now();

    if (i >= options.warmup) {
      model_update_samples_us.push_back(
        std::chrono::duration<double, std::micro>(t_model1 - t0).count());
      pose_samples_us.push_back(
        std::chrono::duration<double, std::micro>(t_pose1 - t_model1).count());
      impl_step_samples_us.push_back(
        std::chrono::duration<double, std::micro>(t1 - t_pose1).count());
      total_samples_us.push_back(
        std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
  }

  std::vector<double> sorted_total = total_samples_us;
  std::sort(sorted_total.begin(), sorted_total.end());
  const double total_p99 = percentile_us(sorted_total, 99.0);

  std::cout << "cartesian_update_timing_test\n"
            << "  controller: " << controller_name() << "\n"
            << "  implementation_library: " << options.impl_library << "\n"
            << "  iterations: " << options.iterations << "\n"
            << "  warmup: " << options.warmup << "\n"
            << "  joints: " << options.joints << "\n"
            << "  robot_model: enabled\n";
  print_stats("model_update", model_update_samples_us);
  print_stats("pose", pose_samples_us);
  print_stats("plugin_step", impl_step_samples_us);
  print_stats("wrapper_compute", total_samples_us);
  std::cout << "  total_p99_under_500us: " << (total_p99 < 500.0 ? "true" : "false") << "\n";

  return total_p99 < 500.0 ? 0 : 3;
}
