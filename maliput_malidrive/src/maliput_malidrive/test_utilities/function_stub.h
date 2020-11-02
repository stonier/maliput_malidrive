// Copyright 2020 Toyota Research Institute
#pragma once

#include "malidrive/road_curve/function.h"
#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace road_curve {
namespace test {

/// Stub class that forwards the argument list of its constructor to each
/// private interface implementation.
class FunctionStub : public Function {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(FunctionStub);

  FunctionStub(double f_result, double f_dot_result, double f_dot_dot_result, double p0_result, double p1_result,
               bool is_g1_contiguous)
      : f_result_(f_result),
        f_dot_result_(f_dot_result),
        f_dot_dot_result_(f_dot_dot_result),
        p0_result_(p0_result),
        p1_result_(p1_result),
        is_g1_contiguous_(is_g1_contiguous) {}

 private:
  // Virtual private definitions of the public members. Public member
  // constraints apply the same.
  //@{
  double do_f(double) const override { return f_result_; }
  double do_f_dot(double) const override { return f_dot_result_; }
  double do_f_dot_dot(double) const override { return f_dot_dot_result_; }
  double do_p0() const override { return p0_result_; }
  double do_p1() const override { return p1_result_; }
  bool DoIsG1Contiguous() const override { return is_g1_contiguous_; }
  //@}

  const double f_result_{};
  const double f_dot_result_{};
  const double f_dot_dot_result_{};
  const double p0_result_{};
  const double p1_result_{};
  const bool is_g1_contiguous_{};
};

}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
