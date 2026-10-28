# Generic Cartesian controller wrapper

`compliant_controllers/GenericCartesianControllerWrapper` reads position, velocity,
and effort state interfaces and writes effort commands in `joints` order.
Configuration is cached during configure; deactivate and reconfigure to apply
parameter changes. Use the command topics for live targets/gains.

## Parameters

**Required** means the wrapper reads the parameter without declaring a default;
supply it in controller YAML. ROS/controller-manager options such as
`use_sim_time` are separate.

| Parameter | Type / default | Meaning |
|---|---|---|
| `joints` | string array, required, nonempty | Controlled joints and ordering for interfaces and all joint vectors. |
| `arm_id` | string, required | Robot identifier for diagnostics; does not select joints or frames. |
| `impl_library` | string, required | Implementation shared library, e.g. `libcartesian_impedance_impl.so`. Relative names are first resolved under the package's installed `lib`, then passed to `dlopen`. Requires `create_controller` and `destroy_controller` exports. |
| `control_frame` | string, `""` (must set) | Robot frame used for pose and Jacobian calculations. Must name a URDF link or joint. Franka FR3 bringup defaults to `fr3_link8`; other robots set their own control frame. Formerly `ee_frame`. |
| `compliance_frame` | string, `""` | Rigid URDF frame defining the spring/damper origin and moving gain axes. Empty uses base-axis gains at E. Nonempty requires `CartesianImpedanceImpl`. Formerly `stiffness_frame`; migrate YAML and launch arguments. |
| `robot_description` | string, optional | Local URDF XML; preferred when nonempty. |
| `robot_description_node` | string, required | Remote URDF source node when local XML is absent. Service/response waits are each 2 seconds. Read even when local XML is supplied. |
| `robot_description_param` | string, required | Remote parameter name, normally `robot_description`. Read even when local XML is supplied. |
| `init_k_pos` | double, required | Initial isotropic position stiffness, N/m; use finite nonnegative values. |
| `init_k_ori` | double, required | Initial isotropic orientation stiffness, Nm/rad; use finite nonnegative values. |
| `max_step_guard_enabled` | bool, `true` | Limits target distance from the current measured pose, independently for position and orientation. |
| `max_position_command_step_m` | double, `0.01` | Maximum translation-error norm, metres. Excess replaces position target with measured position. |
| `max_orientation_command_step_rad` | double, `pi/180` | Maximum shortest-rotation error, radians. Excess replaces orientation target with measured orientation. |
| `gravity_compensation_enabled` | bool, `false` | Adds model gravity torque; choose according to hardware gravity compensation. |
| `dithering_enabled` | bool, `false` | Adds error-biased rotational dither about robot-base Y through E's Jacobian. Compiled constants: 50 Hz, base amplitude 1.5 Nm, alpha 50, beta 0.8. |
| `friction_compensation_enabled` | bool, `false` | Enables joint friction compensation. |
| `friction_compensation.model` | string, `""` | Required when enabled: `sigmoid` or `hysteresis`. |
| `friction_compensation.scale` | double, `1.0` | Finite nonnegative torque multiplier; zero suppresses friction contribution. |
| `friction_compensation.phi1` | double array, `[]` | Sigmoid torque amplitude, one finite coefficient per joint. |
| `friction_compensation.phi2` | double array, `[]` | Sigmoid velocity slope, one finite coefficient per joint. |
| `friction_compensation.phi3` | double array, `[]` | Sigmoid velocity offset, one finite coefficient per joint. |
| `friction_compensation.hyst` | double array, `[]` | Hysteresis velocity threshold, one finite nonnegative coefficient per joint. |
| `friction_compensation.fvp` | double array, `[]` | Positive-direction viscous coefficients, one finite value per joint. |
| `friction_compensation.fcp` | double array, `[]` | Positive-direction Coulomb torque coefficients, one finite value per joint. |
| `friction_compensation.fvn` | double array, `[]` | Negative-direction viscous coefficients, one finite value per joint. |
| `friction_compensation.fcn` | double array, `[]` | Negative-direction Coulomb torque coefficients, one finite value per joint. |
| `joint_limit_repulsion.enabled` | bool, absent means `false` | Adds spring/damper torque near URDF position limits; continuous joints are excluded. |
| `joint_limit_repulsion.stiffness` | double array, required when enabled | One broadcast value or one per joint; finite nonnegative stiffness (Nm/rad for revolute joints). |
| `joint_limit_repulsion.damping` | double array, required when enabled | Same sizing; finite nonnegative damping (Nms/rad for revolute joints). |
| `joint_limit_repulsion.margin` | double array, required when enabled | Same sizing; finite positive distance from limit where repulsion starts (rad for revolute joints). |
| `log_file` | string, `""` | Empty disables wrapper CSV logging. Otherwise inserts a timestamp before the extension; missing extension becomes `.csv`. Also forwarded to implementation. |
| `diagnostic_logger.log_filter_tag` | double, absent means `0.0` | Initial CSV sample tag, converted to integer. No live tag subscription is created by this Cartesian wrapper. |
| `plugin_params_file` | string, `""` | Opaque path forwarded to the implementation; interpretation depends on implementation. |

For velocity v, sigmoid torque is
`phi1/(1+exp(-phi2*(v+phi3))) - phi1/(1+exp(-phi2*phi3))`, with exponential
arguments clamped to [-60,60]. Hysteresis torque is `fvp*v+fcp` for `v>=hyst`,
`fvn*v-fcn` for `v<=-hyst`, otherwise zero. Only the selected model's vectors
are required. All joint arrays follow `joints` order.

Activation seeds the current measured pose, zero desired twist/wrench, initial
stiffness and block damping `2*sqrt(K)`, current joint positions as nullspace
target, nullspace stiffness 5 and damping `2*sqrt(5)`. Updates add gravity,
friction, dither and joint-limit repulsion to implementation torque. Final torque
is clamped to +/-120 Nm, nonfinite values become zero, and magnitudes below
1e-9 become zero. Deactivation writes zero torque.

## Frames

Control always runs in robot-base coordinates. `header.frame_id` identifies the
reference coordinates and `target_frame` identifies the frame being commanded.
For example, `target_frame: tool_tcp` and `header.frame_id: fixture` specify
the desired tool_tcp pose relative to fixture. The wrapper first transforms the
target into base coordinates and then converts it to the configured `control_frame`.
The controlled tool may be defined by a rigid URDF offset or by a separate TF
broadcast relative to the configured control frame. Its offset is treated
as rigid; no relative tool-motion twist is added.

Publishers using the former header-as-controlled-frame convention must move that
name to `target_frame` and use their actual reference coordinates in the header.

| Frame / field | Convention |
|---|---|
| Robot base B | Robot-base coordinates. The wrapper transforms incoming pose, twist and feedforward wrench from `header.frame_id` into these coordinates, using TF at the command timestamp. The robot base used by this wrapper is the URDF root link; it is independent of the command reference frame and `target_frame`. |
| Configured control frame E (`control_frame`) | Internal pose target and Jacobian origin. The Cartesian wrapper rejects empty or unknown control-frame names rather than selecting an arbitrary tool frame. Franka defaults to `fr3_link8`. The wrapper does not configure the model's optional TCP offset. |
| Compliance frame S | A URDF link or joint rigidly connected to E through fixed joints, including ancestors/branches. Joint names resolve to child links. Unknown frames and paths crossing movable joints fail configuration. Both translation and rotation matter. |
| Empty `compliance_frame` | Full-command gain blocks use header axes and are rotated into base axes at E. Stiffness-only topics and initial gains use base axes. |
| Explicit `compliance_frame` | Spring/damper gains use S's current moving axes at S's origin. Desired S pose is derived from the converted E target and fixed E-to-S transform. Choosing S=E selects E's moving axes. |
| `CartesianCommand.header.frame_id` | Coordinate frame of pose, twist and feedforward wrench. Leading slashes are ignored. Empty frames or unavailable timestamped transforms are rejected, retaining the last command. The configured `control_frame` remains fixed. |
| `CartesianCommand.target_frame` | Required tool frame C. Cached rigid URDF link/joint offsets are used when available; other tools use TF from C to E at the command timestamp. A URDF joint resolves to its child link. Leading slashes are ignored. Empty frames and unavailable transforms are rejected. |
| Feedforward wrench | Specified at C in header axes, converted to base axes and shifted to E. Nullspace projection retains E's Jacobian. |
| Launch `world` / `base_frame` | Optional identity static TF `world -> base_frame` only; does not change robot-base coordinates. Demo launchers may map world/base aliases to empty compliance frame; every nonempty `compliance_frame` must be a rigid URDF frame connected to E; externally broadcast `target_frame` tools use TF. |

Let F be the header reference, C the message controlled frame, E the robot
control frame, and S the configured compliance frame. Rigid URDF transforms `T_EC`
are cached at configuration time. External tool transforms are looked up in TF
at each command timestamp with zero wait, and used as rigid offsets for that command. TF supplies `T_BF` at the message timestamp:

- `T_BC_des = T_BF * T_FC_des`.
- `T_BE_des = T_BC_des * inverse(T_EC)`; pose guards run on this converted E target.
- Desired twist is first rotated into B axes, then shifted using
  `v_E_des = v_C_des - omega_des cross (R_BE_des * t_EC)`.
- Wrench is first rotated into B axes, then shifted using
  `M_E = M_C + (R_BE_measured * t_EC) cross F`. This measured lever ensures
  `J_E^T W_E = J_C^T W_C`; force and angular velocity are unchanged by the shift.

Reference-frame translation alone does not change the vector application point;
the lever terms come from changing the controlled point C to configured control point E.
Twists are desired absolute velocities re-expressed in header axes; frame motion
is not added as a relative-velocity correction. Without `compliance_frame`,
full-command gains rotate as `K_B = R_BF K_F R_BF^T` (likewise damping) at E.
With explicit `compliance_frame`, gains already use S axes and are unchanged by
the header. Selecting `target_frame` does not move the configured compliance
point. Stiffness-only topics retain configured gain axes because they have no header.
The buffered command is stored in base coordinates and held there even if its
reference later moves. TF runs only in the non-real-time callback; controlled-point
URDF transforms are cached at configuration time; external tool offsets use TF
on message receipt. No transformations run in update().

Positions are metres, angles radians, twists m/s and rad/s, wrenches N and Nm.
Six-vectors use `[linear; angular]` / `[force; moment]` order. Gain matrices are
row-major 3x3 blocks: position stiffness N/m, orientation stiffness Nm/rad,
position damping Ns/m, orientation damping Nms/rad.

## Command topics

Topics are relative to the controller node namespace, reliable KeepLast(1).

| Topic | Message | Behavior |
|---|---|---|
| `cartesian_command` | `compliant_controllers_msgs/msg/CartesianCommand` | Replaces target/gains. Timestamp must be nonzero, no earlier than activation and no more than 0.5 s old. No later command timeout: last accepted reference is held. TF lookup uses the exact stamp with zero wait in the non-real-time callback. |
| `stiffness_pos` | `std_msgs/msg/Float64MultiArray` | Nine finite symmetric positive-semidefinite row-major values. Updates translational stiffness and derives damping `2*sqrt(K)` by eigendecomposition, preserving other fields. |
| `stiffness_ori` | `std_msgs/msg/Float64MultiArray` | Same validation and damping derivation for rotational stiffness. |

Full commands supply `pose`, `velocity`, `wrench_ff`, and four nine-value
stiffness/damping blocks. These use the frame conventions above. Unlike the
stiffness-only topics, the full-command callback does not validate symmetry/PSD.
Pose guards run after transformation and act independently, retaining other command fields; nonfinite pose
errors trigger replacement even with the guard disabled. Quaternions are normalized.

Nonempty equal-length `nullspace_position` and `nullspace_stiffness` are copied;
matching `nullspace_damping` is copied, otherwise damping is zero. Nonempty
`torques_ff` is copied. Supply one value per controlled joint; the callback does
not enforce this length. `acceleration`, `nj` and `mode` are ignored. Full commands
replace preceding stiffness-topic updates rather than merging with them.

## Generic launch arguments

`launch/cartesian_wrapper.launch.py` loads a controller defined in manager YAML,
applies overrides, configures it and optionally activates it. It requires an
existing controller manager.

| Argument | Default | Effect |
|---|---|---|
| `namespace` | `""` | Spawner namespace and controller path. |
| `controller_name` | `cartesian_impedance_controller` | Controller to load/configure. |
| `controller_manager` | `""` | Defaults to `/<namespace>/controller_manager` or `/controller_manager`. |
| `controller_manager_timeout` | `30` | Spawner timeout, seconds. |
| `start_controller` | `true` | Activate after configuring. |
| `robot_profile_file` | `""` | YAML with selected profile's `ros__parameters`. |
| `friction_compensation_profile` | `friction_compensation_sigmoid` | Profile key; no profile loaded if path or key is empty. |
| `gravity_compensation_enabled`, `dithering_enabled`, `friction_compensation_enabled`, `max_step_guard_enabled` | `""` | Nonempty overrides profile/YAML as boolean; empty retains existing values. |
| `control_frame`, `compliance_frame`, `plugin_params_file`, `log_file` | `""` | Nonempty overrides profile/YAML. Empty retains values, so clear a YAML frame in YAML or through controller parameters before configure. |
| `arm_id` | `""` | Used only to default static TF child to `<arm_id>_link0`. |
| `base_frame` | `""` | Static TF child, defaults to `<arm_id>_link0`. |
| `publish_world_to_base` | `true` | Publishes identity `world -> base_frame`. |
| `robot_profile`, `impl_library`, `joints`, `robot_description_node`, `robot_description_param` | `""` | Declared but not forwarded by this generic include; set controller values in YAML. |
| `shutdown_on_done` | `false` | Currently inactive: completion watcher is constructed but not returned as a launch action. |

The environment variable `CONTROLLER_DEBUG` enables wrapper debug logs whenever
present, even with value `0`. Demo Docker entrypoints use `COMPLIANCE_FRAME` for
the renamed frame launch argument.
