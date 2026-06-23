#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <array>
#include <vector>

namespace franka_follower_controllers {

class PidController;

// Implements a (discretized with sample time `dt`) PID controller with
// anti-windup correction.
// Given x, x_dot and x_ddot the system state and first and second derivative,
// x_ref, x_dot_ref and x_ddot_ref the corresponding references and u the output
// of the PID, and K_x the gains,
// u = x_ddot_ref - Kp (x - x_ref) - Kd (x_dot - x_dot_ref) - Ki _int_ (x -
// x_ref).
// The anti-windup correction is a further modification to the integral term
// when output saturation is active. See [1] for a description of the problem
// and of the possible solutions. This class uses the back-calculation method.
// [1] Control System Design. Lecture notes for ME 155A. Karl Johan Astrom. 2002
// - Section 6.5.
class PidController {
 public:
  // Constructs a PID controller for the specified `size` and sample time `dt`.
  // Note: the resulting PID is not coupled between the different variables,
  // i.e. the gains are currently diagonal matrices. The resulting PID object
  // corresponds to `size` PIDs for SISO systems. All the vectorised quantities
  // in other methods will assume size `size`.
  // P, D and I gains are initialized to zero. Anti-windup gains are initialized
  // to 0.01. Saturation is disabled.
  // Note: the sample time `dt` is assumed to be constant thoughout the whole
  // execution. Jitter will negatively affect performance and stability of the
  // closed-loop system. It is the user's responsibility to keep the sample time
  // as close as possible to `dt`.
  explicit PidController(double dt_seconds);

  // Sets the controller proportional gains. The gain matrix is assumed to be
  // diagonal. p_gains is the diagonal of the matrix.
  bool SetProportionalGains(const std::array<double, 7>& p_gains);

  // Sets the controller derivative gains. The gain matrix is assumed to be
  // diagonal. d_gains is the diagonal of the matrix.
  bool SetDerivativeGains(const std::array<double, 7>& d_gains);

  // Sets the controller integral gains. The gain matrix is assumed to be
  // diagonal. i_gains is the diagonal of the matrix.
  bool SetIntegralGains(const std::array<double, 7>& i_gains);

  // Sets the controller anti-windup gains. The gain matrix is assumed to be
  // diagonal. antiwindup_gains is the diagonal of the matrix.
  bool SetAntiWindupGains(const std::array<double, 7>& antiwindup_gains);

  // Sets the saturation for the controller output.
  bool SetSaturation(const std::array<double, 7>& saturation_low,
                     const std::array<double, 7>& saturation_high);

  // Reset the PID internal state.
  // All the errors (proportional, derivative and integral) are reset to zero,
  // with zero velocity reference and feedforward term.
  bool Reset(const std::array<double, 7>& current_value);

  // Sets a setpoint for the controller. Feedforward and reference derivative
  // are assumed and will be set to zero.
  bool SetReference(const std::array<double, 7>& reference);

  // Sets a trajectory reference for the controller.
  bool SetTrajectoryReference(const std::array<double, 7>& reference,
                              const std::array<double, 7>& reference_derivative,
                              const std::array<double, 7>& feedforward);

  // Computes the controller output. SetReference or SetTrajectoryReference must
  // have been called before to have a valid reference.
  bool ComputePIDOutput(const std::array<double, 7>& current_value,
                        const std::array<double, 7>& current_value_derivative,
                        std::array<double, 7>* control_output);

  // Gets the current error tracked by the controller.
  bool GetCurrentError(std::array<double, 7>* error) const;

  // Serialization methods, to allow a PID controller's state to be serialized
  // to and deserialized from an array of doubles. This is useful e.g. for use
  // in MuJoCo plugins.
  //
  // State is serialized as:
  // | ref | ref_d | feedforward | error | error_d | error_i | dist_from_sat |
  //
  // Note that gains etc. are not serialized, as they are expected to be static
  // after an initialization phase. Setting the state of a PID controller using
  // a serialized state from another controller with different gains or size
  // may result in undefined behaviour.
  int GetNumberOfDoublesInSerializedState() const;
  bool GetStateSerializedAsDoubles(std::vector<double>* state) const;
  bool SetStateFromStateSerializedAsDoubles(
      const std::vector<double>& serialized_state);

 private:
  double dt_seconds_;
  // Gains section.
  std::array<double, 7> p_gains_;
  std::array<double, 7> d_gains_;
  std::array<double, 7> i_gains_;
  std::array<double, 7> antiwindup_gains_;

  std::array<double, 7> saturation_low_;
  std::array<double, 7> saturation_high_;

  // Current references.
  std::array<double, 7> current_reference_;
  std::array<double, 7> current_reference_derivative_;
  std::array<double, 7> current_feedforward_;

  // The following variables are used as buffers to avoid memory allocation
  // at runtime.
  std::array<double, 7> error_;
  std::array<double, 7> error_derivative_;
  std::array<double, 7> error_integral_;
  // Needed for the anti-windup computation.
  std::array<double, 7> distance_from_saturation_;
};
}  // namespace franka_follower_controllers
#endif  // PID_CONTROLLER_H_
