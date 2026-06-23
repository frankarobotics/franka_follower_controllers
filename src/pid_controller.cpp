#include "franka_follower_controllers/pid_controller.hpp"
#include <array>
#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <limits>
#include <vector>

namespace franka_follower_controllers {

namespace {

// Returns a const Map for the specified array.
Eigen::Map<const Eigen::Matrix<double, 7, 1>> EigenConstVector(
    const std::array<double, 7>& array) {
  return Eigen::Map<const Eigen::Matrix<double, 7, 1>>(array.data());
}

// Returns a Map for the specified array.
Eigen::Map<Eigen::Matrix<double, 7, 1>> EigenVector(std::array<double, 7>* array) {
  return Eigen::Map<Eigen::Matrix<double, 7, 1>>(array->data());
}

}  // namespace

PidController::PidController(double dt_seconds)
    : dt_seconds_(dt_seconds) {
    p_gains_.fill(0);
    d_gains_.fill(0);
    i_gains_.fill(0);
    antiwindup_gains_.fill(0.01);
    saturation_low_.fill(std::numeric_limits<double>::lowest());
    saturation_high_.fill(std::numeric_limits<double>::max());
    current_reference_.fill(0);
    current_reference_derivative_.fill(0);
    current_feedforward_.fill(0);
    error_.fill(0);
    error_derivative_.fill(0);
    error_integral_.fill(0);
    distance_from_saturation_.fill(0);
}

bool PidController::SetProportionalGains(const std::array<double, 7>& p_gains) {
  EigenVector(&p_gains_) = EigenConstVector(p_gains);
  return true;
}

bool PidController::SetDerivativeGains(const std::array<double, 7>& d_gains) {
  EigenVector(&d_gains_) = EigenConstVector(d_gains);
  return true;
}

bool PidController::SetIntegralGains(const std::array<double, 7>& i_gains) {
  EigenVector(&i_gains_) = EigenConstVector(i_gains);
  return true;
}

bool PidController::SetAntiWindupGains(
    const std::array<double, 7>& antiwindup_gains) {
  EigenVector(&antiwindup_gains_) = EigenConstVector(antiwindup_gains);
  return true;
}

bool PidController::Reset(const std::array<double, 7>& current_value) {
  // Reset integral.
  EigenVector(&error_integral_).setZero();
  EigenVector(&distance_from_saturation_).setZero();

  EigenVector(&current_reference_derivative_).setZero();
  EigenVector(&current_feedforward_).setZero();
  // Set the reference to coincide with the current plant value.
  EigenVector(&current_reference_) = EigenConstVector(current_value);
  return true;
}

bool PidController::SetSaturation(const std::array<double, 7>& saturation_low,
                                  const std::array<double, 7>& saturation_high) {
  EigenVector(&saturation_low_) = EigenConstVector(saturation_low);
  EigenVector(&saturation_high_) = EigenConstVector(saturation_high);
  return true;
}

bool PidController::SetReference(const std::array<double, 7>& reference) {
  EigenVector(&current_reference_derivative_).setZero();
  EigenVector(&current_feedforward_).setZero();
  EigenVector(&current_reference_) = EigenConstVector(reference);
  return true;
}

bool PidController::SetTrajectoryReference(
    const std::array<double, 7>& reference,
    const std::array<double, 7>& reference_derivative,
    const std::array<double, 7>& feedforward) {
  EigenVector(&current_reference_) = EigenConstVector(reference);
  EigenVector(&current_reference_derivative_) =
      EigenConstVector(reference_derivative);
  EigenVector(&current_feedforward_) = EigenConstVector(feedforward);
  return true;
}

bool PidController::ComputePIDOutput(
    const std::array<double, 7>& current_value,
    const std::array<double, 7>& current_value_derivative,
    std::array<double, 7>* control_output) {
  if (!control_output) {
    return false;
  }

  // Compute the error: e:= current - reference.
  auto reference = EigenConstVector(current_reference_);
  auto value = EigenConstVector(current_value);
  auto error = EigenVector(&error_);
  error = value - reference;

  // Compute error on derivative.
  auto error_derivative = EigenVector(&error_derivative_);
  auto value_derivative = EigenConstVector(current_value_derivative);
  auto reference_derivative = EigenConstVector(current_reference_derivative_);
  error_derivative = value_derivative - reference_derivative;

  // The integral is approximated as a forward difference, i.e.
  // I(t + 1) = I(t) + e dt.
  // That is, we update the integral after computing the current output.
  auto integral = EigenVector(&error_integral_);

  // Now compute the output.
  // Temporarily save the output in the distance_from_saturation_.
  auto unsaturated_output = EigenVector(&distance_from_saturation_);
  auto output = EigenVector(control_output);

  // Feedforward.
  unsaturated_output = EigenConstVector(current_feedforward_);
  // Proportional term.
  unsaturated_output -= EigenConstVector(p_gains_).asDiagonal() * error;
  // Derivative term.
  unsaturated_output -=
      EigenConstVector(d_gains_).asDiagonal() * error_derivative;
  // Integral term.
  unsaturated_output -= EigenConstVector(i_gains_).asDiagonal() * integral;

  output = unsaturated_output.cwiseMin(EigenConstVector(saturation_high_))
               .cwiseMax(EigenConstVector(saturation_low_));
  // Now that the output is saved, compute the distance between the actual value
  // and the saturated value as it is needed by the anti-windup term.
  // We want to compute sat(u) - u.
  unsaturated_output -= output;  // This computes u - sat(u).

  // Now update the integral for the next time step.
  integral += (error + EigenConstVector(antiwindup_gains_).asDiagonal() *
                           unsaturated_output) *
              dt_seconds_;

  return true;
}

bool PidController::GetCurrentError(std::array<double, 7>* error) const {
  if (!error) {
    return false;
  }
  EigenVector(error) = EigenConstVector(error_);
  return true;
}

int PidController::GetNumberOfDoublesInSerializedState() const {
  return p_gains_.size() * 7;
}

// | ref | ref_d | feedforward | error | error_d | error_i | dist_from_sat |
bool PidController::GetStateSerializedAsDoubles(
    std::vector<double>* state) const {
  if (!state || state->size() != static_cast<size_t>(GetNumberOfDoublesInSerializedState())) {
    return false;
  }

  auto it = state->begin();
  for (const std::array<double, 7>& quantity : {
           current_reference_,
           current_reference_derivative_,
           current_feedforward_,
           error_,
           error_derivative_,
           error_integral_,
           distance_from_saturation_,
       }) {
    it = std::copy(quantity.begin(), quantity.end(), it);
  }
  // internal invariant, sense check.
  assert(it == state->end());
  return true;
}

bool PidController::SetStateFromStateSerializedAsDoubles(
    const std::vector<double>& serialized_state) {
  if (serialized_state.size() != static_cast<size_t>(GetNumberOfDoublesInSerializedState())) {
    return false;
  }

  auto it = serialized_state.begin();
  std::array<std::array<double, 7>*, 7> quantities = {
      &current_reference_,
      &current_reference_derivative_,
      &current_feedforward_,
      &error_,
      &error_derivative_,
      &error_integral_,
      &distance_from_saturation_};

  for (std::array<double, 7>* quantity : quantities) {
    std::copy_n(it, quantity->size(), quantity->begin());
    it += quantity->size();
  }
  // internal invariant, sense check.
  assert(it == serialized_state.end());
  return true;
}

}  // namespace franka_follower_controllers
