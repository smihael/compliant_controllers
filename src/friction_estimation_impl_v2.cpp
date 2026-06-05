#include <compliant_controllers/friction_estimation_impl_v2.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <toml.hpp>

namespace {
constexpr double kPi = 3.14159265358979323846;

bool allFinite(const Eigen::VectorXd& v) {
	return v.array().isFinite().all();
}

double parameterToDouble(const control::AbstractController::ParameterValue& value, double fallback) {
	if (const auto* v = std::get_if<double>(&value)) {
		return *v;
	}
	if (const auto* v = std::get_if<int64_t>(&value)) {
		return static_cast<double>(*v);
	}
	if (const auto* v = std::get_if<bool>(&value)) {
		return *v ? 1.0 : 0.0;
	}
	return fallback;
}

std::vector<double> parameterToDoubleVector(const control::AbstractController::ParameterValue& value) {
	if (const auto* v = std::get_if<std::vector<double>>(&value)) {
		return *v;
	}
	if (const auto* v = std::get_if<std::vector<int64_t>>(&value)) {
		std::vector<double> out;
		for (const auto x : *v) {
			out.push_back(static_cast<double>(x));
		}
		return out;
	}
	if (const auto* v = std::get_if<double>(&value)) {
		return {*v};
	}
	if (const auto* v = std::get_if<int64_t>(&value)) {
		return {static_cast<double>(*v)};
	}
	if (const auto* v = std::get_if<bool>(&value)) {
		return {*v ? 1.0 : 0.0};
	}
	return {};
}

std::vector<int> parameterToIntVector(const control::AbstractController::ParameterValue& value) {
	std::vector<int> out;
	if (const auto* v = std::get_if<std::vector<int64_t>>(&value)) {
		for (const auto x : *v) {
			out.push_back(static_cast<int>(x));
		}
		return out;
	}
	if (const auto* v = std::get_if<std::vector<double>>(&value)) {
		for (const auto x : *v) {
			out.push_back(static_cast<int>(x));
		}
		return out;
	}
	if (const auto* v = std::get_if<int64_t>(&value)) {
		return {static_cast<int>(*v)};
	}
	if (const auto* v = std::get_if<double>(&value)) {
		return {static_cast<int>(*v)};
	}
	if (const auto* v = std::get_if<bool>(&value)) {
		return {*v ? 1 : 0};
	}

	const auto doubles = parameterToDoubleVector(value);
	for (const auto d : doubles) {
		out.push_back(static_cast<int>(d));
	}
	return out;
}

void assignVectorFromParameter(const std::map<std::string, control::AbstractController::ParameterValue>& params,
										 const std::string& name,
										 Eigen::VectorXd& target) {
	const auto it = params.find(name);
	if (it == params.end()) {
		return;
	}
	const auto values = parameterToDoubleVector(it->second);
	if (values.size() == 1) {
		target.setConstant(values.front());
		return;
	}
	const std::size_t count = std::min(values.size(), static_cast<std::size_t>(target.size()));
	for (std::size_t i = 0; i < count; ++i) {
		target(static_cast<int>(i)) = values[i];
	}
}

void loadTomlParams(const std::string& path,
							std::map<std::string, control::AbstractController::ParameterValue>& out) {
	const auto parsed = toml::parse_file(path);

	if (const auto v = parsed["csv_file"].value<std::string>()) {
		out["csv_file"] = *v;
	}
	if (const auto v = parsed["output_csv"].value<std::string>()) {
		out["output_csv"] = *v;
	}
	if (const auto v = parsed["amplitude_scaling"].value<double>()) {
		out["amplitude_scaling"] = *v;
	}
	if (const auto v = parsed["kp"].value<double>()) {
		out["kp"] = *v;
	}
	if (const auto v = parsed["kd"].value<double>()) {
		out["kd"] = *v;
	}

	if (const auto* arr = parsed["q_min"].as_array()) {
		std::vector<double> vals;
		for (const auto& item : *arr) {
			if (const auto d = item.value<double>()) {
				vals.push_back(*d);
			}
		}
		out["q_min"] = vals;
	}

	if (const auto* arr = parsed["q_max"].as_array()) {
		std::vector<double> vals;
		for (const auto& item : *arr) {
			if (const auto d = item.value<double>()) {
				vals.push_back(*d);
			}
		}
		out["q_max"] = vals;
	}

	if (const auto* arr = parsed["q_init"].as_array()) {
		std::vector<double> vals;
		for (const auto& item : *arr) {
			if (const auto d = item.value<double>()) {
				vals.push_back(*d);
			}
		}
		out["q_init"] = vals;
	}

	if (const auto* arr = parsed["test_joint_indices"].as_array()) {
		std::vector<int64_t> vals;
		for (const auto& item : *arr) {
			if (const auto i = item.value<int64_t>()) {
				vals.push_back(*i);
			}
		}
		out["test_joint_indices"] = vals;
	}

	if (const auto* arr = parsed["k_gains"].as_array()) {
		std::vector<double> vals;
		for (const auto& item : *arr) {
			if (const auto d = item.value<double>()) {
				vals.push_back(*d);
			}
		}
		out["k_gains"] = vals;
	}

	if (const auto* arr = parsed["d_gains"].as_array()) {
		std::vector<double> vals;
		for (const auto& item : *arr) {
			if (const auto d = item.value<double>()) {
				vals.push_back(*d);
			}
		}
		out["d_gains"] = vals;
	}
}
}  // namespace

namespace compliant_controllers {

FrictionEstimationImplV2::ModifiedFourierSeriesGenerator::ModifiedFourierSeriesGenerator(unsigned seed)
		: joints_{} {
	for (int j = 0; j < 7; ++j) {
		joints_[j] = makeJointGenerator(seed + static_cast<unsigned>(j) * 101U);
	}
}

auto FrictionEstimationImplV2::ModifiedFourierSeriesGenerator::makeJointGenerator(unsigned seed)
		-> JointGenerator {
	JointGenerator joint{};
	std::mt19937 rng(seed);
	std::uniform_real_distribution<double> dist(-0.35, 0.35);

	for (int k = 0; k < kHarmonics; ++k) {
		joint.amp[k] = dist(rng) / std::sqrt(static_cast<double>(k + 1));
	}
	joint.norm = computeNorm(joint.amp);
	return joint;
}

double FrictionEstimationImplV2::ModifiedFourierSeriesGenerator::computeNorm(
		const std::array<double, kHarmonics>& amp) {
	constexpr int kSamples = 5000;
	double peak = 0.0;
	for (int s = 0; s <= kSamples; ++s) {
		const double tau = static_cast<double>(s) / static_cast<double>(kSamples);
		const double phase_base = 2.0 * kPi * tau;
		double raw = 0.0;
		for (int k = 1; k <= kHarmonics; ++k) {
			const double phase = static_cast<double>(k) * phase_base;
			raw += amp[k - 1] * std::sin(phase) / static_cast<double>(k);
		}
		peak = std::max(peak, std::abs(raw));
	}

	return peak > 1e-12 ? peak : 1.0;
}

double FrictionEstimationImplV2::ModifiedFourierSeriesGenerator::normalizedSignal(
		double t,
		double duration_s,
		int joint_one_based) const {
	if (joint_one_based < 1 || joint_one_based > 7) {
		return 0.0;
	}
	const int j = joint_one_based - 1;
	const JointGenerator& joint = joints_[j];
	const double segment_duration = std::max(1e-3, duration_s);
	const double omega = 2.0 * kPi / segment_duration;

	double raw = 0.0;
	for (int k = 1; k <= kHarmonics; ++k) {
		const double phase = static_cast<double>(k) * omega * t;
		raw += joint.amp[k - 1] * std::sin(phase) / static_cast<double>(k);
	}
	const double normalized = raw / joint.norm;
	return std::clamp(normalized, -1.0, 1.0);
}

FrictionEstimationImplV2::FrictionEstimationImplV2(int num_joints)
		: num_joints_(num_joints),
			q_init_(Eigen::VectorXd::Zero(num_joints)),
			q_min_(Eigen::VectorXd::Constant(num_joints, -0.5)),
			q_max_(Eigen::VectorXd::Constant(num_joints, 0.5)),
			q_move_ref_(Eigen::VectorXd::Zero(num_joints)),
			tau_cmd_(Eigen::VectorXd::Zero(num_joints)),
			k_gains_(Eigen::VectorXd::Constant(num_joints, 80.0)),
			d_gains_(Eigen::VectorXd::Constant(num_joints, 15.0)),
			gravity_torque_(Eigen::VectorXd::Zero(num_joints)),
			coriolis_torque_(Eigen::VectorXd::Zero(num_joints_)),
			mass_matrix_(Eigen::MatrixXd::Zero(num_joints_, num_joints_)),
			prev_dq_des_(Eigen::VectorXd::Zero(num_joints_)) {
	test_joint_indices_.reserve(static_cast<std::size_t>(num_joints_));
	for (int i = 1; i <= num_joints_; ++i) {
		test_joint_indices_.push_back(i);
	}

	std::cout << "\033[32mUsing " << kName << " compiled at " << __DATE__ << ", " << __TIME__
						<< "\033[0m" << std::endl;
}

FrictionEstimationImplV2::~FrictionEstimationImplV2() {
	writeCsvIfNeeded();
}

void FrictionEstimationImplV2::setParameter(const std::string& name, const ParameterValue& value) {
	if (name == "plugin_params_file") {
		if (const auto* path = std::get_if<std::string>(&value); path != nullptr && !path->empty()) {
			loadTomlParams(*path, forwarded_params_);
		}
		parameters_applied_ = false;
		return;
	}
	forwarded_params_[name] = value;
	parameters_applied_ = false;
}

void FrictionEstimationImplV2::readForwardedParameters() {
	if (parameters_applied_) {
		return;
	}

	if (const auto it = forwarded_params_.find("csv_file"); it != forwarded_params_.end()) {
		if (const auto* p = std::get_if<std::string>(&it->second); p != nullptr && !p->empty()) {
			output_csv_path_ = *p;
		}
	} else if (const auto it = forwarded_params_.find("output_csv"); it != forwarded_params_.end()) {
		if (const auto* p = std::get_if<std::string>(&it->second); p != nullptr && !p->empty()) {
			output_csv_path_ = *p;
		}
	}

	if (output_csv_path_.empty()) {
		output_csv_path_ = buildDefaultOutputPath();
	}

	if (const auto it = forwarded_params_.find("amplitude_scaling"); it != forwarded_params_.end()) {
		amplitude_scaling_ = parameterToDouble(it->second, amplitude_scaling_);
	}
	if (const auto it = forwarded_params_.find("kp"); it != forwarded_params_.end()) {
		k_gains_.setConstant(parameterToDouble(it->second, k_gains_(0)));
	}
	if (const auto it = forwarded_params_.find("kd"); it != forwarded_params_.end()) {
		d_gains_.setConstant(parameterToDouble(it->second, d_gains_(0)));
	}

	assignVectorFromParameter(forwarded_params_, "q_min", q_min_);
	assignVectorFromParameter(forwarded_params_, "q_max", q_max_);
	assignVectorFromParameter(forwarded_params_, "k_gains", k_gains_);
	assignVectorFromParameter(forwarded_params_, "d_gains", d_gains_);

	if (forwarded_params_.find("q_init") == forwarded_params_.end()) {
		q_init_ = 0.5 * (q_min_ + q_max_);
	} else {
		assignVectorFromParameter(forwarded_params_, "q_init", q_init_);
	}

	if (const auto it = forwarded_params_.find("test_joint_indices"); it != forwarded_params_.end()) {
		auto values = parameterToIntVector(it->second);
		test_joint_indices_ = values;
	} else if (const auto it = forwarded_params_.find("test_joint_indeces"); it != forwarded_params_.end()) {
		auto values = parameterToIntVector(it->second);
		test_joint_indices_ = values;
	}

	samples_.clear();

	RCLCPP_INFO_STREAM(rclcpp::get_logger(kName),
						 "q_min=" << q_min_.transpose() << " q_max=" << q_max_.transpose()
						 << " q_init=" << q_init_.transpose()
						 << " k=" << k_gains_.transpose()
						 << " d=" << d_gains_.transpose());

	RCLCPP_INFO(rclcpp::get_logger(kName),
						"Configured v2: csv=%s, amp=%.3f, joints=%zu",
						output_csv_path_.c_str(),
						amplitude_scaling_,
						test_joint_indices_.size());
	parameters_applied_ = true;
}

std::string FrictionEstimationImplV2::buildDefaultOutputPath() const {
	const std::filesystem::path dir("/tmp");
	std::error_code ec;
	std::filesystem::create_directories(dir, ec);

	const auto now = std::chrono::system_clock::now();
	const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
	std::tm now_tm{};
	localtime_r(&now_c, &now_tm);

	std::ostringstream filename;
	filename << "friction_estimation_v2_" << std::put_time(&now_tm, "%Y%m%d_%H%M%S") << ".csv";
	return (dir / filename.str()).string();
}

bool FrictionEstimationImplV2::initializeRun(const control::ControllerState& state) {
	readForwardedParameters();

	if (state.q.size() != num_joints_ || state.dq.size() != num_joints_ || state.tau.size() != num_joints_) {
		RCLCPP_ERROR(rclcpp::get_logger(kName), "State size mismatch during init.");
		return false;
	}
	if (!allFinite(state.q) || !allFinite(state.dq) || !allFinite(state.tau)) {
		RCLCPP_ERROR(rclcpp::get_logger(kName), "Non-finite state during init.");
		return false;
	}

	q_move_ref_ = state.q;
	tau_cmd_.setZero();
	prev_dq_des_.setZero();
	sim_time_ = 0.0;
	phase_time_s_ = 0.0;
	active_joint_list_idx_ = 0;
	setPhase(Phase::MoveToQInit, "initialization");
	initialized_ = true;
	finished_ = false;
	csv_written_ = false;
	return true;
}

double FrictionEstimationImplV2::clamp(double x, double low, double high) const {
	if (x < low) {
		return low;
	}
	if (x > high) {
		return high;
	}
	return x;
}

const char* FrictionEstimationImplV2::phaseName(Phase phase) {
	switch (phase) {
		case Phase::MoveToQInit:
			return "MoveToQInit";
		case Phase::ExciteJoint:
			return "ExciteJoint";
		case Phase::ReturnToQInit:
			return "ReturnToQInit";
		case Phase::Done:
			return "Done";
		default:
			return "Unknown";
	}
}

void FrictionEstimationImplV2::setPhase(Phase phase, const char* reason) {
	phase_ = phase;
	phase_time_s_ = 0.0;
	if (reason != nullptr) {
		RCLCPP_INFO(rclcpp::get_logger(kName), "Phase begin: %s (%s)", phaseName(phase_), reason);
	} else {
		RCLCPP_INFO(rclcpp::get_logger(kName), "Phase begin: %s", phaseName(phase_));
	}
}

bool FrictionEstimationImplV2::isAtTarget(const control::ControllerState& state,
																					const Eigen::Ref<const Eigen::VectorXd>& q_target) const {
	if (q_target.size() != num_joints_) {
		return false;
	}
	const double q_err_max = (q_target - state.q).cwiseAbs().maxCoeff();
	const double dq_max = state.dq.cwiseAbs().maxCoeff();
	return q_err_max < q_init_tolerance_rad_ && dq_max < 0.1;
}

void FrictionEstimationImplV2::computeHoldTorques(const control::ControllerState& state,
																									const Eigen::Ref<const Eigen::VectorXd>& q_target,
																									Eigen::Ref<Eigen::VectorXd> tau_out,
																									const Eigen::VectorXd* dq_target) const {
	tau_out.setZero();
	for (int i = 0; i < num_joints_; ++i) {
		const double dq_des = (dq_target != nullptr && dq_target->size() == num_joints_) ? (*dq_target)(i) : 0.0;
		const double tau = k_gains_(i) * (q_target(i) - state.q(i)) + d_gains_(i) * (dq_des - state.dq(i));
		tau_out(i) = clamp(tau, -tau_abs_limit_nm_, tau_abs_limit_nm_);
	}
}

void FrictionEstimationImplV2::computeMoveTorques(const control::ControllerState& state,
																									const Eigen::Ref<const Eigen::VectorXd>& q_target,
																									double dt,
																						Eigen::Ref<Eigen::VectorXd> tau_out,
																						Eigen::Ref<Eigen::VectorXd> dq_des_out) {
	tau_out.setZero();
	dq_des_out.setZero();
	if (dt <= 0.0 || !std::isfinite(dt)) {
		dt = 0.001;
	}

	for (int i = 0; i < num_joints_; ++i) {
		const double max_step = move_max_vel_rad_s_ * dt;
		const double delta = q_target(i) - q_move_ref_(i);
		const double step = clamp(delta, -max_step, max_step);
		q_move_ref_(i) += step;
		const double dq_des = step / dt;
		dq_des_out(i) = dq_des;
		const double tau = 0.45 * k_gains_(i) * (q_move_ref_(i) - state.q(i)) +
											 0.45 * d_gains_(i) * (dq_des - state.dq(i));
		tau_out(i) = clamp(tau, -move_tau_limit_nm_, move_tau_limit_nm_);
	}
}

void FrictionEstimationImplV2::writeCsvIfNeeded() {
	if (csv_written_) {
		return;
	}
	csv_written_ = true;

	std::filesystem::path out_path(output_csv_path_);
	std::error_code ec;
	if (out_path.has_parent_path()) {
		std::filesystem::create_directories(out_path.parent_path(), ec);
	}

	std::ofstream out(output_csv_path_);
	if (!out.is_open()) {
		RCLCPP_ERROR(rclcpp::get_logger(kName), "Failed to open csv output: %s", output_csv_path_.c_str());
		return;
	}

	out << "t,active_joint,excitation,q,dq,tau_measured,tau_task,tau_commanded,tau_coriolis,tau_inertia,tau_gravity,q_des,dq_des\n";
	for (const auto& s : samples_) {
		out << s.t << ',' << s.active_joint << ',' << (s.excitation ? 1 : 0) << ','
				<< s.q << ',' << s.dq << ',' << s.tau_measured << ',' << s.tau_task << ',' << s.tau_commanded << ','
				<< s.tau_coriolis << ',' << s.tau_inertia << ',' << s.tau_gravity << ','
				<< s.q_des << ',' << s.dq_des << '\n';
	}

	RCLCPP_INFO(rclcpp::get_logger(kName), "Wrote %zu samples to %s", samples_.size(), output_csv_path_.c_str());
}

void FrictionEstimationImplV2::requestShutdownIfNeeded() {
	if (shutdown_requested_) {
		return;
	}
	shutdown_requested_ = true;
	RCLCPP_INFO(rclcpp::get_logger(kName), "Friction estimation v2 finished. Requesting ROS shutdown.");
	rclcpp::shutdown();
}

bool FrictionEstimationImplV2::step(const control::ControlCommand& /*command*/,
																		const control::ControllerState& current_state,
																		Eigen::Ref<Eigen::VectorXd> control_output,
																		double dt) {
	if (control_output.size() != num_joints_) {
		RCLCPP_ERROR(rclcpp::get_logger(kName), "control_output size mismatch.");
		return false;
	}

	if (!initialized_) {
		if (!initializeRun(current_state)) {
			control_output.setZero();
			return false;
		}
	}

	if (!allFinite(current_state.q) || !allFinite(current_state.dq) || !allFinite(current_state.tau)) {
		RCLCPP_ERROR(rclcpp::get_logger(kName), "Non-finite state detected, outputting zero torque.");
		control_output.setZero();
		return false;
	}

	if (dt <= 0.0 || !std::isfinite(dt)) {
		dt = 0.001;
	}

	tau_cmd_.setZero();
	Eigen::VectorXd q_des = q_init_;
	Eigen::VectorXd dq_des = Eigen::VectorXd::Zero(num_joints_);
	Eigen::VectorXd ddq_des = Eigen::VectorXd::Zero(num_joints_);
	Eigen::VectorXd tau_task = Eigen::VectorXd::Zero(num_joints_);
	Eigen::VectorXd tau_inertia = Eigen::VectorXd::Zero(num_joints_);
	gravity_torque_.setZero();
	coriolis_torque_.setZero();
	mass_matrix_.setZero();

	const bool gravity_ok = robot_model_ != nullptr && robot_model_->getGravity(gravity_torque_);
	const bool coriolis_ok = robot_model_ != nullptr && robot_model_->getCoriolis(current_state.dq, coriolis_torque_);
	const bool mass_ok = robot_model_ != nullptr && robot_model_->getMassMatrix(mass_matrix_);

	const int active_joint = test_joint_indices_[active_joint_list_idx_] - 1;
	const Eigen::VectorXd amplitude_rad = amplitude_scaling_ * 0.5 * (q_max_ - q_min_).cwiseAbs();
	const Phase phase_before = phase_;

	switch (phase_) {
		case Phase::MoveToQInit: {
			computeMoveTorques(current_state, q_init_, dt, tau_cmd_, dq_des);
			tau_task = tau_cmd_;
			ddq_des = (dq_des - prev_dq_des_) / dt;
			if (isAtTarget(current_state, q_init_)) {
				setPhase(Phase::ExciteJoint, "q_init reached");
				q_move_ref_ = q_init_;
			}
			break;
		}
		case Phase::ExciteJoint: {
			const double t = phase_time_s_;
			const double q_offset = amplitude_rad(active_joint) * mfs_.normalizedSignal(t, measurement_time_s_, active_joint + 1);
			const double dt_fd = 1e-3;
			const double q_offset_next = amplitude_rad(active_joint) *
																	 mfs_.normalizedSignal(t + dt_fd, measurement_time_s_, active_joint + 1);
			const double q_offset_next2 = amplitude_rad(active_joint) *
															  mfs_.normalizedSignal(t + 2.0 * dt_fd, measurement_time_s_, active_joint + 1);
			q_des(active_joint) = clamp(q_init_(active_joint) + q_offset, q_min_(active_joint), q_max_(active_joint));
			dq_des(active_joint) = (q_offset_next - q_offset) / dt_fd;
			const double dq_des_next = (q_offset_next2 - q_offset_next) / dt_fd;
			ddq_des(active_joint) = (dq_des_next - dq_des(active_joint)) / dt_fd;

			computeHoldTorques(current_state, q_des, tau_cmd_, &dq_des);
			tau_task = tau_cmd_;
			tau_cmd_(active_joint) = clamp(tau_cmd_(active_joint), -tau_abs_limit_nm_, tau_abs_limit_nm_);

			const double missing = std::numeric_limits<double>::quiet_NaN();

			if (mass_ok) {
				tau_inertia.noalias() = mass_matrix_ * ddq_des;
			}

			tau_cmd_ = tau_task;
			if (coriolis_ok) {
				tau_cmd_ += coriolis_torque_;
			}
			if (mass_ok) {
				tau_cmd_ += tau_inertia;
			}

			for (int i = 0; i < num_joints_; ++i) {
				tau_cmd_(i) = clamp(tau_cmd_(i), -tau_abs_limit_nm_, tau_abs_limit_nm_);
			}

			samples_.push_back(Sample{
					sim_time_,
					active_joint + 1,
					true,
					current_state.q(active_joint),
					current_state.dq(active_joint),
					current_state.tau(active_joint),
					tau_task(active_joint),
					tau_cmd_(active_joint),
					mass_ok ? tau_inertia(active_joint) : missing,
					gravity_ok ? gravity_torque_(active_joint) : missing,
					coriolis_ok ? coriolis_torque_(active_joint) : missing,
					q_des(active_joint),
					dq_des(active_joint),
			});

			if (phase_time_s_ >= measurement_time_s_) {
				setPhase(Phase::ReturnToQInit, "joint excitation complete");
			}
			break;
		}
		case Phase::ReturnToQInit: {
			computeMoveTorques(current_state, q_init_, dt, tau_cmd_, dq_des);
			tau_task = tau_cmd_;
			ddq_des = (dq_des - prev_dq_des_) / dt;
			if (isAtTarget(current_state, q_init_)) {
				if (phase_time_s_ >= return_settle_time_s_) {
					++active_joint_list_idx_;
					if (active_joint_list_idx_ >= test_joint_indices_.size()) {
						setPhase(Phase::Done, "all selected joints measured");
						tau_cmd_.setZero();
						finished_ = true;
						writeCsvIfNeeded();
						requestShutdownIfNeeded();
					} else {
						setPhase(Phase::MoveToQInit, "next joint");
					}
				}
			}
			break;
		}
		case Phase::Done: {
			tau_cmd_.setZero();
			finished_ = true;
			writeCsvIfNeeded();
			requestShutdownIfNeeded();
			break;
		}
	}

	if (phase_before == Phase::MoveToQInit || phase_before == Phase::ReturnToQInit) {
		if (mass_ok) {
			tau_inertia.noalias() = mass_matrix_ * ddq_des;
		}
		tau_cmd_ = tau_task;
		if (coriolis_ok) {
			tau_cmd_ += coriolis_torque_;
		}
		if (mass_ok) {
			tau_cmd_ += tau_inertia;
		}
		for (int i = 0; i < num_joints_; ++i) {
			tau_cmd_(i) = clamp(tau_cmd_(i), -tau_abs_limit_nm_, tau_abs_limit_nm_);
		}
	}

	prev_dq_des_ = dq_des;

	if (!finished_) {
		phase_time_s_ += dt;
		if (phase_ == Phase::MoveToQInit && phase_time_s_ >= settle_time_s_) {
			phase_time_s_ = settle_time_s_;
		}
	}

	sim_time_ += dt;
	control_output = tau_cmd_;

	if (!allFinite(control_output)) {
		RCLCPP_ERROR(rclcpp::get_logger(kName), "Non-finite output detected.");
		control_output.setZero();
		return false;
	}

	return true;
}

}  // namespace compliant_controllers

FACTORY_EXPORT_CONTROLLER(compliant_controllers::FrictionEstimationImplV2)
