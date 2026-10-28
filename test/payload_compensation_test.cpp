#include <gtest/gtest.h>
#include <compliant_controllers/payload_compensation.hpp>
#include <limits>

using namespace compliant_controllers;
namespace {
geometry_msgs::msg::Inertia payload(double mass = 2) {
  geometry_msgs::msg::Inertia msg;
  msg.m = mass;
  msg.com.x = .3;
  msg.ixx = .02; msg.iyy = .03; msg.izz = .04;
  return msg;
}
TEST(Payload, GravityForceAndTorque) {
  auto msg = payload();
  const auto target = PayloadProperties::fromMessage(msg, Eigen::Isometry3d::Identity());
  PayloadCompensation compensation;
  auto wrench = compensation.step(target, 1, Eigen::Matrix3d::Identity());
  EXPECT_TRUE(wrench.head<3>().isApprox(Eigen::Vector3d(0, 0, 19.62)));
  EXPECT_TRUE(wrench.tail<3>().isApprox(Eigen::Vector3d(0, -5.886, 0)));
  const auto rotation = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  wrench = compensation.step(target, 1, rotation);
  EXPECT_TRUE(wrench.tail<3>().isApprox(Eigen::Vector3d(5.886, 0, 0), 1e-12));
  msg.com.x = 0;
  EXPECT_TRUE(compensation.step(PayloadProperties::fromMessage(msg, Eigen::Isometry3d::Identity()),
                               1, rotation).tail<3>().isZero());
}
TEST(Payload, RigidFrameConversionAndTensorRotation) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(.7, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  T.translation() << .1, -.2, .3;
  auto msg = payload();
  const auto transformed = PayloadProperties::fromMessage(msg, T);
  EXPECT_TRUE(transformed.first_moment.isApprox(msg.m * (T * Eigen::Vector3d(.3, 0, 0))));
  Eigen::Matrix3d tensor = Eigen::Vector3d(.02, .03, .04).asDiagonal();
  EXPECT_TRUE(transformed.inertia.isApprox(T.linear() * tensor * T.linear().transpose()));
  const auto com = transformed.first_moment / transformed.mass;
  msg.com.x = com.x(); msg.com.y = com.y(); msg.com.z = com.z();
  msg.ixx = transformed.inertia(0,0); msg.ixy = transformed.inertia(0,1);
  msg.ixz = transformed.inertia(0,2); msg.iyy = transformed.inertia(1,1);
  msg.iyz = transformed.inertia(1,2); msg.izz = transformed.inertia(2,2);
  const auto direct = PayloadProperties::fromMessage(msg, Eigen::Isometry3d::Identity());
  PayloadCompensation a, b;
  EXPECT_TRUE(a.step(transformed, 1, T.linear()).isApprox(b.step(direct, 1, T.linear())));
  EXPECT_TRUE(direct.inertia.isApprox(transformed.inertia));
  const auto base = direct.transformed(T);
  EXPECT_TRUE(base.first_moment.isApprox(T.linear() * direct.first_moment + direct.mass * T.translation()));
  EXPECT_TRUE(base.inertia.isApprox(T.linear() * direct.inertia * T.linear().transpose()));
}
TEST(Payload, AlphaReplacementClearAndReset) {
  const auto target = PayloadProperties::fromMessage(payload(), Eigen::Isometry3d::Identity());
  PayloadCompensation compensation;
  EXPECT_NEAR(compensation.step(target, .5, Eigen::Matrix3d::Identity())[2], 9.81, 1e-12);
  auto replacement = target;
  replacement.mass = 4; replacement.first_moment = Eigen::Vector3d(0, .8, 0);
  const auto wrench = compensation.step(replacement, .5, Eigen::Matrix3d::Identity());
  EXPECT_NEAR(wrench[2], 2.5 * 9.81, 1e-12);
  EXPECT_NEAR(wrench[3], .4 * 9.81, 1e-12);
  EXPECT_NEAR(wrench[4], -.15 * 9.81, 1e-12);
  EXPECT_TRUE(compensation.step({}, .5, Eigen::Matrix3d::Identity()).isApprox(.5 * wrench));
  compensation.reset();
  EXPECT_TRUE(compensation.step({}, .5, Eigen::Matrix3d::Identity()).isZero());
  for (int i = 0; i < 100; ++i) compensation.step(target, .5, Eigen::Matrix3d::Identity());
  EXPECT_NEAR(compensation.step(target, .5, Eigen::Matrix3d::Identity())[2], 19.62, 1e-12);
}
TEST(Payload, InvalidPropertiesAndTransforms) {
  const auto identity = Eigen::Isometry3d::Identity();
  auto msg = payload(); msg.m = -1;
  EXPECT_THROW(PayloadProperties::fromMessage(msg, identity), std::invalid_argument);
  msg = payload(); msg.m = std::numeric_limits<double>::infinity();
  EXPECT_THROW(PayloadProperties::fromMessage(msg, identity), std::invalid_argument);
  msg = payload(); msg.com.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(PayloadProperties::fromMessage(msg, identity), std::invalid_argument);
  msg = payload(); msg.ixy = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(PayloadProperties::fromMessage(msg, identity), std::invalid_argument);
  msg = payload(); msg.ixx = -1;
  EXPECT_THROW(PayloadProperties::fromMessage(msg, identity), std::invalid_argument);
  msg = payload(); msg.izz = 1;
  EXPECT_THROW(PayloadProperties::fromMessage(msg, identity), std::invalid_argument);
  msg = payload(0);
  EXPECT_THROW(PayloadProperties::fromMessage(msg, identity), std::invalid_argument);
  msg = geometry_msgs::msg::Inertia{}; msg.m = 1;
  EXPECT_NO_THROW(PayloadProperties::fromMessage(msg, identity));
  auto bad = identity; bad.linear()(0,0) = 2;
  EXPECT_THROW(PayloadProperties::fromMessage(msg, bad), std::invalid_argument);
  bad = identity; bad.translation().x() = std::numeric_limits<double>::infinity();
  EXPECT_THROW(PayloadProperties::fromMessage(msg, bad), std::invalid_argument);
}
}  // namespace
