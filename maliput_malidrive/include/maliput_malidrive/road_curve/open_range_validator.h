// Copyright 2019 Toyota Research Institute
#pragma once

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace road_curve {

/// Functor to validate if a number is within [min, max] and considering
/// assuming a tolerance. This tolerance extends the range to be
/// [min - tolerance, max + tolerance].
/// The functor not only validates that number is within the range, but also
/// adapts it to be in the interval (min, max) where the difference between the
/// close and open range is epsilon.
/// This functor is motivated to wrap MALIDRIVE_DEMAND() and
/// MALIDRIVE_IS_IN_RANGE() calls plus a std::clamp() to interface with ODRM.
class OpenRangeValidator {
 public:
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(OpenRangeValidator)
  OpenRangeValidator() = delete;

  /// Constructs the functor.
  ///
  /// @param min lower extreme of the range.
  /// @param max upper extreme of the range.
  /// @param tolerance is the range extension to be accepted.
  /// @param epsilon is minimum difference that separates a number within the
  ///        range to be distinct from range extremes (`min` and `max`).
  /// @throws maliput::common::assertion_error When `tolerance` is non positive.
  /// @throws maliput::common::assertion_error When `epsilon` is not in
  ///         [0, tolerance].
  /// @throws maliput::common::assertion_error When `min` + `epsilon` > `max` or
  ///         `max` - `epsilon` < `min`
  OpenRangeValidator(double min, double max, double tolerance, double epsilon)
      : min_(min), max_(max), tolerance_(tolerance), epsilon_(epsilon) {
    MALIDRIVE_THROW_UNLESS(tolerance_ > 0.);
    MALIDRIVE_IS_IN_RANGE(epsilon_, 0., tolerance_);
    MALIDRIVE_VALIDATE((min_ + epsilon_) <= max_, maliput::common::assertion_error,
                       std::string("Open range lower bound <") + std::to_string((min_ + epsilon_)) +
                           "> is greater than <" + std::to_string(max_) + ">");
    MALIDRIVE_VALIDATE(min <= (max_ - epsilon_), maliput::common::assertion_error,
                       std::string("Open range upper bound <") + std::to_string((max_ - epsilon_)) +
                           "> is less than <" + std::to_string(min) + ">");
  }

  /// Evaluates whether `s` is in range or not.
  /// @returns `s` when it is within the open range. If `s` is equal to either
  /// range extremes or the difference to them is less or equal to tolerance, it
  /// returns the closest open range value. Otherwise, it
  /// @throws maliput::common::assertion_error.
  double operator()(double s) const;

 private:
  double min_{};
  double max_{};
  double tolerance_{};
  double epsilon_{};
};

}  // namespace road_curve
}  // namespace malidrive
