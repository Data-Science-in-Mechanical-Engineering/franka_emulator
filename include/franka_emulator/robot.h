// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include "emulator/shared.h"
#include "emulator/semaphore.h"
#include "emulator/network.h"
#include "command_types.h"
#include "control_types.h"
#include "duration.h"
#include "lowpass_filter.h"
#include "robot_state.h"
#include "model.h"

#include <Eigen/Dense>
#include <functional>
#include <string>
#include <atomic>

/**
 * @file robot.h
 * Contains the franka::Robot type.
 */

namespace FRANKA_EMULATOR {

class Model;

/**
 * Maintains a network connection to the robot, provides the current robot state, gives access to
 * the model library and allows to control the robot.
 *
 * @note
 * The members of this class are threadsafe.
 *
 * @par Nominal end effector frame NE
 * The nominal end effector frame is configured outside of libfranka and cannot be changed here.
 *
 * @par End effector frame EE
 * By default, the end effector frame EE is the same as the nominal end effector frame NE
 * (i.e. the transformation between NE and EE is the identity transformation).
 * With Robot::setEE, a custom transformation matrix can be set.
 *
 * @anchor k-frame
 * @par Stiffness frame K
 * The stiffness frame is used for Cartesian impedance control, and for measuring and applying
 * forces.
 * It can be set with Robot::setK.
 */
class Robot {
 public:
  /**
   * Version of the robot server.
   */
  using ServerVersion = uint16_t;

  /**
   * Establishes a connection with the robot.
   *
   * @param[in] franka_address IP/hostname of the robot.
   * @param[in] realtime_config if set to Enforce, an exception will be thrown if realtime priority
   * cannot be set when required. Setting realtime_config to Ignore disables this behavior.
   * @param[in] log_size sets how many last states should be kept for logging purposes.
   * The log is provided when a ControlException is thrown.
   *
   * @throw NetworkException if the connection is unsuccessful.
   * @throw IncompatibleVersionException if this version of `libfranka` is not supported.
   * 
   * @attention **[Emulator]** `log_size` is ignored
   */
  explicit Robot(const std::string& franka_address,
                 RealtimeConfig realtime_config = RealtimeConfig::kEnforce,
                 size_t log_size = 50);

  /**
   * Move-constructs a new Robot instance.
   *
   * @param[in] other Other Robot instance.
   */
  Robot(Robot&& other) noexcept;

  /**
   * Move-assigns this Robot from another Robot instance.
   *
   * @param[in] other Other Robot instance.
   *
   * @return Robot instance.
   */
  Robot& operator=(Robot&& other) noexcept;

  /**
   * Closes the connection.
   */
  ~Robot() noexcept;

  /**
   * @name Motion generation and joint-level torque commands
   *
   * The following methods allow to perform motion generation and/or send joint-level torque
   * commands without gravity and friction by providing callback functions.
   *
   * Only one of these methods can be active at the same time; a franka::ControlException is thrown
   * otherwise.
   *
   * @anchor callback-docs
   * When a robot state is received, the callback function is used to calculate the response: the
   * desired values for that time step. After sending back the response, the callback function will
   * be called again with the most recently received robot state. Since the robot is controlled with
   * a 1 kHz frequency, the callback functions have to compute their result in a short time frame
   * in order to be accepted. Callback functions take two parameters:
   *
   * * A franka::RobotState showing the current robot state.
   * * A FRANKA_EMULATOR::Duration to indicate the time since the last callback invocation. Thus, the
   *   duration is zero on the first invocation of the callback function!
   *
   * The following incomplete example shows the general structure of a callback function:
   *
   * @code{.cpp}
   * double time = 0.0;
   * auto control_callback = [&time](const franka::RobotState& robot_state,
   *                                 franka::Duration time_step) -> franka::JointPositions {
   *   time += time_step.toSec();  // Update time at the beginning of the callback.
   *
   *   franka::JointPositions output = getJointPositions(time);
   *
   *   if (time >= max_time) {
   *     // Return MotionFinished at the end of the trajectory.
   *     return franka::MotionFinished(output);
   *   }
   *
   *   return output;
   * }
   * @endcode
   */

  /**
   * Starts a control loop for sending joint-level torque commands.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] control_callback Callback function providing joint-level torque commands.
   * See @ref callback-docs "here" for more details.
   * @param[in] limit_rate True if rate limiting should be activated. True by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   * 
   * @attention **[Emulator]** `limit_rate` and `cutoff_frequency` are ignored
   */
  void control(std::function<Torques(const RobotState&, FRANKA_EMULATOR::Duration)> control_callback,
               bool limit_rate = true,
               double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * Starts a control loop for sending joint-level torque commands and joint positions.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] control_callback Callback function providing joint-level torque commands.
   * See @ref callback-docs "here" for more details.
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] limit_rate True if rate limiting should be activated. True by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if joint-level torque or joint position commands are NaN or
   * infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   * 
   * @attention **[Emulator]** `limit_rate` and `cutoff_frequency` are ignored
   */
  void control(
      std::function<Torques(const RobotState&, FRANKA_EMULATOR::Duration)> control_callback,
      std::function<JointPositions(const RobotState&, FRANKA_EMULATOR::Duration)> motion_generator_callback,
      bool limit_rate = true,
      double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * Starts a control loop for sending joint-level torque commands and joint velocities.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] control_callback Callback function providing joint-level torque commands.
   * See @ref callback-docs "here" for more details.
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] limit_rate True if rate limiting should be activated. True by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if joint-level torque or joint velocitiy commands are NaN or
   * infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   * 
   * @attention **[Emulator]** `limit_rate` and `cutoff_frequency` are ignored
   */
  void control(
      std::function<Torques(const RobotState&, FRANKA_EMULATOR::Duration)> control_callback,
      std::function<JointVelocities(const RobotState&, FRANKA_EMULATOR::Duration)> motion_generator_callback,
      bool limit_rate = true,
      double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * Starts a control loop for sending joint-level torque commands and Cartesian poses.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] control_callback Callback function providing joint-level torque commands.
   * See @ref callback-docs "here" for more details.
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] limit_rate True if rate limiting should be activated. True by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if joint-level torque or Cartesian pose command elements are NaN
   * or infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   * 
   * @attention **[Emulator]** `limit_rate` and `cutoff_frequency` are ignored
   */
  void control(
      std::function<Torques(const RobotState&, FRANKA_EMULATOR::Duration)> control_callback,
      std::function<CartesianPose(const RobotState&, FRANKA_EMULATOR::Duration)> motion_generator_callback,
      bool limit_rate = true,
      double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * Starts a control loop for sending joint-level torque commands and Cartesian velocities.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] control_callback Callback function providing joint-level torque commands.
   * See @ref callback-docs "here" for more details.
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] limit_rate True if rate limiting should be activated. True by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if joint-level torque or Cartesian velocity command elements are
   * NaN or infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   * 
   * @attention **[Emulator]** `limit_rate` and `cutoff_frequency` are ignored
   */
  void control(std::function<Torques(const RobotState&, FRANKA_EMULATOR::Duration)> control_callback,
               std::function<CartesianVelocities(const RobotState&, FRANKA_EMULATOR::Duration)>
                   motion_generator_callback,
               bool limit_rate = true,
               double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * Starts a control loop for a joint position motion generator with a given controller mode.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] controller_mode Controller to use to execute the motion.
   * @param[in] limit_rate True if rate limiting should be activated. True by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if joint position commands are NaN or infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   * 
   * @attention **[Emulator]** `limit_rate` and `cutoff_frequency` are ignored
   */
  void control(
      std::function<JointPositions(const RobotState&, FRANKA_EMULATOR::Duration)> motion_generator_callback,
      ControllerMode controller_mode = ControllerMode::kJointImpedance,
      bool limit_rate = true,
      double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * Starts a control loop for a joint velocity motion generator with a given controller mode.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] controller_mode Controller to use to execute the motion.
   * @param[in] limit_rate True if rate limiting should be activated. True by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if joint velocity commands are NaN or infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   * 
   * @attention **[Emulator]** `limit_rate` and `cutoff_frequency` are ignored
   */
  void control(
      std::function<JointVelocities(const RobotState&, FRANKA_EMULATOR::Duration)> motion_generator_callback,
      ControllerMode controller_mode = ControllerMode::kJointImpedance,
      bool limit_rate = true,
      double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * Starts a control loop for a Cartesian pose motion generator with a given controller mode.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] controller_mode Controller to use to execute the motion.
   * @param[in] limit_rate True if rate limiting should be activated. True by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if Cartesian pose command elements are NaN or infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   * 
   * @attention **[Emulator]** `limit_rate` and `cutoff_frequency` are ignored
   */
  void control(
      std::function<CartesianPose(const RobotState&, FRANKA_EMULATOR::Duration)> motion_generator_callback,
      ControllerMode controller_mode = ControllerMode::kJointImpedance,
      bool limit_rate = true,
      double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * Starts a control loop for a Cartesian velocity motion generator with a given controller mode.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] controller_mode Controller to use to execute the motion.
   * @param[in] limit_rate True if rate limiting should be activated. True by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if Cartesian velocity command elements are NaN or infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   * 
   * @attention **[Emulator]** `limit_rate` and `cutoff_frequency` are ignored
   */
  void control(std::function<CartesianVelocities(const RobotState&, FRANKA_EMULATOR::Duration)>
                   motion_generator_callback,
               ControllerMode controller_mode = ControllerMode::kJointImpedance,
               bool limit_rate = true,
               double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * @}
   */

  /**
   * Starts a loop for reading the current robot state.
   *
   * Cannot be executed while a control or motion generator loop is running.
   *
   * This minimal example will print the robot state 100 times:
   * @code{.cpp}
   * franka::Robot robot("robot.franka.de");
   * size_t count = 0;
   * robot.read([&count](const franka::RobotState& robot_state) {
   *   std::cout << robot_state << std::endl;
   *   return count++ < 100;
   * });
   * @endcode
   *
   * @param[in] read_callback Callback function for robot state reading.
   *
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void read(std::function<bool(const RobotState&)> read_callback);

  /**
   * Waits for a robot state update and returns it.
   *
   * Cannot be executed while a control or motion generator loop is running.
   *
   * @return Current robot state.
   *
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   *
   * @see Robot::read for a way to repeatedly receive the robot state.
   */
  RobotState readOnce();

  /**
   * @name Commands
   *
   * Commands are executed by communicating with the robot over the network.
   * These functions should therefore not be called from within control or motion generator loops.
   * @{
   */

  /**
   * Returns the parameters of a virtual wall.
   *
   * @param[in] id ID of the virtual wall.
   *
   * @return Parameters of virtual wall.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   *
   * @attention **[Emulator]** The function returns invalid values
   */
  VirtualWallCuboid getVirtualWall(int32_t id);

  /**
   * Changes the collision behavior.
   *
   * Set separate torque and force boundaries for acceleration/deceleration and constant velocity
   * movement phases.
   *
   * Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
   * Forces or torques above the upper threshold are registered as collision and cause the robot to
   * stop moving.
   *
   * @param[in] lower_torque_thresholds_acceleration Contact torque thresholds during
   * acceleration/deceleration for each joint in \f$[Nm]\f$.
   * @param[in] upper_torque_thresholds_acceleration Collision torque thresholds during
   * acceleration/deceleration for each joint in \f$[Nm]\f$.
   * @param[in] lower_torque_thresholds_nominal Contact torque thresholds for each joint
   * in \f$[Nm]\f$.
   * @param[in] upper_torque_thresholds_nominal Collision torque thresholds for each joint
   * in \f$[Nm]\f$.
   * @param[in] lower_force_thresholds_acceleration Contact force thresholds during
   * acceleration/deceleration for \f$(x,y,z,R,P,Y)\f$ in \f$[N]\f$.
   * @param[in] upper_force_thresholds_acceleration Collision force thresholds during
   * acceleration/deceleration for \f$(x,y,z,R,P,Y)\f$ in \f$[N]\f$.
   * @param[in] lower_force_thresholds_nominal Contact force thresholds for \f$(x,y,z,R,P,Y)\f$
   * in \f$[N]\f$.
   * @param[in] upper_force_thresholds_nominal Collision force thresholds for \f$(x,y,z,R,P,Y)\f$
   * in \f$[N]\f$.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   *
   * @see RobotState::cartesian_contact
   * @see RobotState::cartesian_collision
   * @see RobotState::joint_contact
   * @see RobotState::joint_collision
   * @see Robot::automaticErrorRecovery for performing a reset after a collision.
   * 
   * @attention **[Emulator]** The function does nothing
   */
  void setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds_acceleration,
                            const std::array<double, 7>& upper_torque_thresholds_acceleration,
                            const std::array<double, 7>& lower_torque_thresholds_nominal,
                            const std::array<double, 7>& upper_torque_thresholds_nominal,
                            const std::array<double, 6>& lower_force_thresholds_acceleration,
                            const std::array<double, 6>& upper_force_thresholds_acceleration,
                            const std::array<double, 6>& lower_force_thresholds_nominal,
                            const std::array<double, 6>& upper_force_thresholds_nominal);

  /**
   * Changes the collision behavior.
   *
   * Set common torque and force boundaries for acceleration/deceleration and constant velocity
   * movement phases.
   *
   * Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
   * Forces or torques above the upper threshold are registered as collision and cause the robot to
   * stop moving.
   *
   * @param[in] lower_torque_thresholds Contact torque thresholds for each joint in \f$[Nm]\f$.
   * @param[in] upper_torque_thresholds Collision torque thresholds for each joint in \f$[Nm]\f$.
   * @param[in] lower_force_thresholds Contact force thresholds for \f$(x,y,z,R,P,Y)\f$
   * in \f$[N]\f$.
   * @param[in] upper_force_thresholds Collision force thresholds for \f$(x,y,z,R,P,Y)\f$
   * in \f$[N]\f$.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   *
   * @see RobotState::cartesian_contact
   * @see RobotState::cartesian_collision
   * @see RobotState::joint_contact
   * @see RobotState::joint_collision
   * @see Robot::automaticErrorRecovery for performing a reset after a collision.
   * 
   * @attention **[Emulator]** The function does nothing
   */
  void setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds,
                            const std::array<double, 7>& upper_torque_thresholds,
                            const std::array<double, 6>& lower_force_thresholds,
                            const std::array<double, 6>& upper_force_thresholds);

  /**
   * Sets the impedance for each joint in the internal controller.
   *
   * User-provided torques are not affected by this setting.
   *
   * @param[in] K_theta Joint impedance values \f$K_{\theta}\f$.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void setJointImpedance(
      const std::array<double, 7>& K_theta);  // NOLINT(readability-identifier-naming)

  /**
   * Sets the Cartesian impedance for (x, y, z, roll, pitch, yaw) in the internal controller.
   *
   * User-provided torques are not affected by this setting.
   *
   * @param[in] K_x Cartesian impedance values \f$K_x=(x \in [10,3000] \frac{N}{m}, y \in [10,3000]
   * \frac{N}{m}, z \in [10,3000] \frac{N}{m}, R \in [1,300] \frac{Nm}{rad}, P \in [1,300]
   * \frac{Nm}{rad}, Y \in [1,300]  \frac{Nm}{rad})\f$
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void setCartesianImpedance(
      const std::array<double, 6>& K_x);  // NOLINT(readability-identifier-naming)

  /**
   * Locks or unlocks guiding mode movement in (x, y, z, roll, pitch, yaw).
   *
   * If a flag is set to true, movement is unlocked.
   *
   * @note
   * Guiding mode can be enabled by pressing the two opposing buttons near the robot's flange.
   *
   * @param[in] guiding_mode Unlocked movement in (x, y, z, R, P, Y) in guiding mode.
   * @param[in] elbow True if the elbow is free in guiding mode, false otherwise.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * 
   * @attention **[Emulator]** The function does nothing
   */
  void setGuidingMode(const std::array<bool, 6>& guiding_mode, bool elbow);

  /**
   * Sets the transformation \f$^{EE}T_K\f$ from end effector frame to stiffness frame.
   *
   * The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
   *
   * @param[in] EE_T_K Vectorized EE-to-K transformation matrix \f$^{EE}T_K\f$, column-major.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   *
   * @see Robot for an explanation of the stiffness frame.
   */
  void setK(const std::array<double, 16>& EE_T_K);  // NOLINT(readability-identifier-naming)

  /**
   * Sets the transformation \f$^{NE}T_{EE}\f$ from nominal end effector to end effector frame.
   *
   * The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
   *
   * @param[in] NE_T_EE Vectorized NE-to-EE transformation matrix \f$^{NE}T_{EE}\f$, column-major.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   *
   * @see RobotState::NE_T_EE for end effector pose in nominal end effector frame.
   * @see RobotState::O_T_EE for end effector pose in world base frame.
   * @see RobotState::F_T_EE for end effector pose in flange frame.
   * @see Robot for an explanation of the NE and EE frames.
   */
  void setEE(const std::array<double, 16>& NE_T_EE);  // NOLINT(readability-identifier-naming)

  /**
   * Sets dynamic parameters of a payload.
   *
   * @note
   * This is not for setting end effector parameters, which have to be set in the administrator's
   * interface.
   *
   * @param[in] load_mass Mass of the load in \f$[kg]\f$.
   * @param[in] F_x_Cload Translation from flange to center of mass of load
   * \f$^Fx_{C_\text{load}}\f$ in \f$[m]\f$.
   * @param[in] load_inertia Inertia matrix \f$I_\text{load}\f$ in \f$[kg \times m^2]\f$,
   * column-major.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void setLoad(double load_mass,
               const std::array<double, 3>& F_x_Cload,  // NOLINT(readability-identifier-naming)
               const std::array<double, 9>& load_inertia);

  /**
   * Sets the cut off frequency for the given motion generator or controller.
   *
   * @deprecated Use franka::lowpassFilter() instead.
   *
   * Allowed input range for all the filters is between 1.0 Hz and 1000.0 Hz.
   * If the value is set to maximum (1000Hz) then no filtering is done.
   *
   * @param[in] joint_position_filter_frequency Frequency at which the commanded joint
   * position is cut off.
   * @param[in] joint_velocity_filter_frequency Frequency at which the commanded joint
   * velocity is cut off.
   * @param[in] cartesian_position_filter_frequency Frequency at which the commanded
   * Cartesian position is cut off.
   * @param[in] cartesian_velocity_filter_frequency Frequency at which the commanded
   * Cartesian velocity is cut off.
   * @param[in] controller_filter_frequency Frequency at which the commanded torque is cut
   * off.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * 
   * @attention **[Emulator]** The function does nothing
   */
  [[deprecated("Use franka::lowpassFilter instead")]] void setFilters(
      double joint_position_filter_frequency,
      double joint_velocity_filter_frequency,
      double cartesian_position_filter_frequency,
      double cartesian_velocity_filter_frequency,
      double controller_filter_frequency);
  /**
   * Runs automatic error recovery on the robot.
   *
   * Automatic error recovery e.g. resets the robot after a collision occurred.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * 
   * @attention **[Emulator]** The function sets the robot to default position
   */
  void automaticErrorRecovery();

  /**
   * Stops all currently running motions.
   *
   * If a control or motion generator loop is running in another thread, it will be preempted
   * with a franka::ControlException.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void stop();

  /**
   * @}
   */

  /**
   * Loads the model library from the robot.
   *
   * @return Model instance.
   *
   * @throw ModelException if the model library cannot be loaded.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  Model loadModel();

  /**
   * Returns the software version reported by the connected server.
   *
   * @return Software version of the connected server.
   */
  ServerVersion serverVersion() const noexcept;

  /// @cond DO_NOT_DOCUMENT
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;
  /// @endcond

private:
  typedef bool (Callback)(Robot*);
  void *_context                = nullptr;
  Callback *_callback           = nullptr;
  void _control();

  Eigen::Matrix<double, 6, 1> _cartesian_stiffness;
  Eigen::Matrix<double, 6, 1> _cartesian_damping;
  Eigen::Matrix<double, 7, 1> _joint_stiffness;
  Eigen::Matrix<double, 7, 1> _joint_damping;
  std::atomic<bool> _command_running;//Used to lock functions
  bool _control_running;             //Used by stop() and control(). Stop() is not a request to gazebo, it just sets this flag
  emulator::Semaphore _request_mutex;
  emulator::Semaphore _response_mutex;
  emulator::Semaphore _request_condition;
  emulator::Semaphore _response_condition;
  emulator::Semaphore _control_call_condition;
  emulator::Semaphore _control_return_condition;
  emulator::Shared _shared;
  Network _network;
  Model _model;
};

}  // namespace FRANKA_EMULATOR

/** @mainpage Welcome to `franka_emulator 1.0.1`!

Here you will find a `C++` library that replaces and behaves like [libfranka](https://frankaemika.github.io/docs/libfranka.html) with one exception - the robot is virtual! It has similar `C++` and `CMake` API and is created to give developers a possibility to write programs for Panda robot without having it.

@tableofcontents

@section Usage
Usage `franka_emulator` is supposed to be very similar to usage of [libfranka](https://frankaemika.github.io/docs/libfranka.html). Simplest `C++` program reads:
```
#include <franka_emulator/robot.h>
int main()
{
	franka_emulator::Robot robot("<IP>");
}
```
While typical `CMakeLists.txt` usage looks like:
```
find_package(FrankaEmulator 1.0.0 REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example PRIVATE Franka::FrankaEmulator)
```
And that's it, further changes are needed. And is that is not enough, you can add `-DFRANKA_EMULATOR_IMITATE=true` to `cmake` options to eliminate even these differences!

The simulation itself can be launched with:
```
./franka_emulator.sh <IP>
```
Where `<IP>` is the same string that needs to be passed to `franka_emulator::Robot` to make it connected to the simulation. The string is arbitrary (but without slashes) as it is not an actual `IP` address. Actually, the emulator does not use sockets at all.

@section Requirements
The library requires:
 - [Gazebo](http://gazebosim.org) (Standalone or as part of `ROS`)
 - [Eigen](https://eigen.tuxfamily.org)
 - [pinocchio](https://stack-of-tasks.github.io/pinocchio) (set with `-Dpinocchio_DIR=/absolute_path_to_pinocchio` or as part of `ROS`)
 - [libfranka](https://github.com/frankaemika/libfranka) (optionally) (set with `-DFranka_DIR=/absolute_path_to_libfranka/build` or as part of `ROS`)
 - [Google Test](https://github.com/google/googletest) (optionally)
 - [Doxygen](https://www.doxygen.nl/index.html) (optionally)
 - [CMake](https://cmake.org) >= `3.4.0`
 - C++11 compatible compiler
 - Fully preemptable Linux kernel (optionally)

@section Building
The library can be built with [CMake](https://cmake.org) using the following commands:
```
mkdir build
cd build
cmake .. #-DFRANKA_EMULATOR_IMITATE=true
cmake --build .
```

@section Installation
**Step 1**: Getting real-time permission
This can be done in various ways. As an example, the following commands will allow everyone to execute any real-time commands:
```
sudo -s
echo * - rtprio 99       >> /etc/security/limits.conf
echo * - priority 99     >> /etc/security/limits.conf
echo * - memlock 102400  >> /etc/security/limits.conf
exit
```
Reboot for changes to take place. You also may read [this](https://linux.die.net/man/5/limits.conf) for more secure options.

**Optional step 2**: Installing `franka_emulator`
Although the emulator can be run without installation, it can be installed with the commands:
```
cd build
sudo cmake --install .
sudo ldconfig
```
Note that if `FRANKA_EMULATOR_IMITATE` is true, the library will conflict with [libfranka](https://github.com/frankaemika/libfranka). In that case, installing both the emulator and the library will lead to undefined behavior.

**Optional step 3**: Installing real-time kernel
It is said that the fully preemptive kernel can do priority boosting with semaphores, which might increase the emulator's performance (because the library uses semaphores a lot). Note that real-time kernel might cause system instability (and may not). You will alwaus be able to roll back to generic kernel.

For `Debian`: `sudo apt-get install linux-image-rt-amd64` (assuming you have a `x86_64` machine, proceed [here](https://packages.debian.org/search?keywords=linux-image-rt&searchon=names&suite=stable&section=all) if not)

For `Arch`: `yay -S linux-rt-lts` (assuming you use `yay`, proceed [here](https://wiki.archlinux.org/title/AUR_helpers) to see other options)

For `Ubuntu`: Try using `sudo ./install_rt.sh` - it should download, compile and install the kernel along with `systemd-boot`. Improvise if the script fails.

@section Documentation
As the library has the same API as [libfranka](https://frankaemika.github.io/docs/index.html#), you may use [official wiki](https://frankaemika.github.io/libfranka/). However, you also can generate documentation with [Doxygen](https://www.doxygen.nl) using `doxygen` command. Generated documentation contains information of how the emulator's behavior differs from that of original [libfranka](https://frankaemika.github.io/docs/index.html#) (look for **[Emulator]** sections).

@section Contributors
 - Kyrylo Sovailo
 - Original [libfranka](https://github.com/intelligent-soft-robots/o80) is owned by [Franka Emika GmbH](https://www.franka.de)
 - Models from `model` directory were initially generated with [Gazebo](http://gazebosim.org) from `ROS` package `franka_description`
*/