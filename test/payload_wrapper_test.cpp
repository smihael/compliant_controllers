#include <gtest/gtest.h>
#include <compliant_controllers/generic_cartesian_controller_wrapper.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <thread>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace compliant_controllers;
namespace {
const char* xml = R"(<robot name="test"><link name="base"/><link name="ee"/><link name="remote"/>
<joint name="hinge" type="revolute"><parent link="base"/><child link="ee"/>
<axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="100" velocity="10"/></joint>
<joint name="offset" type="fixed"><parent link="ee"/><child link="remote"/>
<origin xyz="0.3 0 0" rpy="0 0 1.5707963267948966"/></joint></robot>)";

class WrapperTest : public testing::Test {
protected:
  GenericCartesianControllerWrapper wrapper;
  std::shared_ptr<rclcpp::Node> publisher;
  rclcpp::executors::SingleThreadedExecutor executor;
  double q=0, dq=0, effort=0, output=0;
  hardware_interface::StateInterface qi{"hinge","position",&q},
    dqi{"hinge","velocity",&dq}, ti{"hinge","effort",&effort};
  hardware_interface::CommandInterface ci{"hinge","effort",&output};
  static void SetUpTestSuite() { rclcpp::init(0,nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void setup(const std::string& frame) {
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({
      rclcpp::Parameter("joints",std::vector<std::string>{"hinge"}),
      rclcpp::Parameter("arm_id","test"), rclcpp::Parameter("control_frame","ee"),
      rclcpp::Parameter("compliance_frame",frame), rclcpp::Parameter("robot_description",xml),
      rclcpp::Parameter("robot_description_node","robot_state_publisher"),
      rclcpp::Parameter("robot_description_param","robot_description"),
      rclcpp::Parameter("init_k_pos",0.0),rclcpp::Parameter("init_k_ori",0.0),
      rclcpp::Parameter("impl_library",std::string(TEST_IMPL_LIBRARY)),
      rclcpp::Parameter("joint_limit_repulsion.enabled",false)});
    ASSERT_EQ(wrapper.init("test_compliance_wrapper","",options),controller_interface::return_type::OK);
  }
  void activate() {
    ASSERT_EQ(wrapper.on_configure(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(qi); states.emplace_back(dqi); states.emplace_back(ti);
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    commands.emplace_back(ci);
    wrapper.assign_interfaces(std::move(commands),std::move(states));
    ASSERT_EQ(wrapper.on_activate(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);
    executor.add_node(wrapper.get_node()->get_node_base_interface());
    publisher=std::make_shared<rclcpp::Node>("test_compliance_sender");
    executor.add_node(publisher);
  }
  void spin() {
    for (int i=0;i<20;++i) { executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
  }
  double update() {
    EXPECT_EQ(wrapper.update(wrapper.get_node()->now(),rclcpp::Duration::from_seconds(.001)),
              controller_interface::return_type::OK);
    return output;
  }
};
class PayloadWrapperTest : public WrapperTest {
protected:
  void configurePayload(bool enabled = true, double alpha = 1, const std::string& frame = "",
                        bool robot_gravity = false) {
    setup(frame);
    // A massive tool on a horizontal-axis hinge gives observable gravity torque.
    std::string description(xml);
    description.replace(description.find("0 0 1"), 5, "0 1 0");
    const auto pos = description.find("<link name=\"ee\"/>");
    description.replace(pos, std::string("<link name=\"ee\"/>").size(),
      "<link name=\"ee\"><inertial><origin xyz=\"0.1 0 0\"/><mass value=\"1\"/>"
      "<inertia ixx=\".01\" iyy=\".01\" izz=\".01\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/>"
      "</inertial></link>");
    wrapper.get_node()->set_parameter(rclcpp::Parameter("robot_description", description));
    wrapper.get_node()->set_parameter(rclcpp::Parameter("payload_compensation_enabled", enabled));
    wrapper.get_node()->set_parameter(rclcpp::Parameter("payload_compensation_alpha", alpha));
    wrapper.get_node()->set_parameter(rclcpp::Parameter("gravity_compensation_enabled", robot_gravity));
    activate();
  }
  bool sendPayload(const std::string& frame, double mass, double com_x = 0) {
    auto client = publisher->create_client<compliant_controllers_msgs::srv::SetPayload>(
      "/test_compliance_wrapper/set_payload");
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      ADD_FAILURE() << "Payload service unavailable"; return false;
    }
    auto request = std::make_shared<compliant_controllers_msgs::srv::SetPayload::Request>();
    request->frame_id = frame; request->inertia.m = mass; request->inertia.com.x = com_x;
    auto future = client->async_send_request(request);
    if (executor.spin_until_future_complete(future, std::chrono::seconds(2)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      ADD_FAILURE() << "Payload service timed out"; return false;
    }
    return future.get()->success;
  }
};
TEST_F(PayloadWrapperTest, AcceptReplaceRejectAndClear) {
  configurePayload();
  ASSERT_TRUE(sendPayload("remote", 2));
  EXPECT_NEAR(update(), -5.886, 1e-9);
  EXPECT_FALSE(sendPayload("base", 10));
  EXPECT_FALSE(sendPayload("missing", 10));
  EXPECT_FALSE(sendPayload("", 10));
  EXPECT_FALSE(sendPayload("ee", -1));
  EXPECT_NEAR(update(), -5.886, 1e-9);
  ASSERT_TRUE(sendPayload("ee", 1, .2));
  EXPECT_NEAR(update(), -1.962, 1e-9);
  ASSERT_TRUE(sendPayload("", 0));
  EXPECT_NEAR(update(), 0, 1e-9);
  wrapper.on_deactivate(rclcpp_lifecycle::State());
}
TEST_F(PayloadWrapperTest, DisabledRejectsUpdates) {
  configurePayload(false);
  EXPECT_FALSE(sendPayload("ee", 1, .3));
  EXPECT_NEAR(update(), 0, 1e-9);
  wrapper.on_deactivate(rclcpp_lifecycle::State());
}
TEST_F(PayloadWrapperTest, RobotGravityAndComplianceAreIndependent) {
  configurePayload(true, 1, "remote", true);
  const double robot = update();
  EXPECT_NEAR(robot, -.981, 1e-9);
  ASSERT_TRUE(sendPayload("remote", 2));
  EXPECT_NEAR(update(), robot - 5.886, 1e-9);
  wrapper.on_deactivate(rclcpp_lifecycle::State());
}
TEST_F(PayloadWrapperTest, AlphaAndActivationReset) {
  configurePayload(true, .5);
  ASSERT_TRUE(sendPayload("remote", 2));
  EXPECT_NEAR(update(), -2.943, 1e-9);
  EXPECT_NEAR(update(), -4.4145, 1e-9);
  ASSERT_TRUE(sendPayload("ee", 1, .2));
  EXPECT_NEAR(update(), (-4.4145 - 1.962) / 2, 1e-9);
  wrapper.on_deactivate(rclcpp_lifecycle::State());
  ASSERT_EQ(wrapper.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_NEAR(update(), -.981, 1e-9);
  ASSERT_TRUE(sendPayload("", 0));
  EXPECT_NEAR(update(), -.4905, 1e-9);
  wrapper.on_deactivate(rclcpp_lifecycle::State());
}
TEST_F(PayloadWrapperTest, ExternalRigidToolUsesTf) {
  configurePayload();
  tf2_ros::StaticTransformBroadcaster broadcaster(publisher);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = publisher->now(); tf.header.frame_id = "ee";
  tf.child_frame_id = "payload_tool"; tf.transform.translation.x = .4;
  tf.transform.rotation.w = 1;
  broadcaster.sendTransform(tf); spin();
  ASSERT_TRUE(sendPayload("payload_tool", 1));
  EXPECT_NEAR(update(), -3.924, 1e-9);
  wrapper.on_deactivate(rclcpp_lifecycle::State());
}
TEST_F(PayloadWrapperTest, InvalidAlphaFailsConfiguration) {
  setup("");
  wrapper.get_node()->set_parameter(rclcpp::Parameter("payload_compensation_alpha", 0.0));
  EXPECT_EQ(wrapper.on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}
TEST_F(PayloadWrapperTest, CurrentOrientationIsUsedEveryCycle) {
  q = .7;
  configurePayload();
  ASSERT_TRUE(sendPayload("remote", 2));
  EXPECT_NEAR(update(), -5.886 * std::cos(.7), 1e-9);
  q = .2;
  const double loaded = update();
  ASSERT_TRUE(sendPayload("", 0));
  const double unloaded = update();
  EXPECT_NEAR(loaded - unloaded, -5.886 * std::cos(.2), 1e-9);
  wrapper.on_deactivate(rclcpp_lifecycle::State());
}
TEST_F(PayloadWrapperTest, ReconfigureClearsTarget) {
  configurePayload();
  ASSERT_TRUE(sendPayload("remote", 2));
  EXPECT_NEAR(update(), -5.886, 1e-9);
  wrapper.on_deactivate(rclcpp_lifecycle::State());
  ASSERT_EQ(wrapper.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(wrapper.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_NEAR(update(), 0, 1e-9);
  wrapper.on_deactivate(rclcpp_lifecycle::State());
}
} // namespace
