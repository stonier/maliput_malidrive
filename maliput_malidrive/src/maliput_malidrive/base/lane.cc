// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/base/lane.h"

#include <cmath>

#include "maliput/common/logger.h"
#include "maliput/math/saturate.h"
#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/road_curve/open_range_validator.h"

namespace malidrive {

using maliput::math::Vector3;

Lane::Lane(const maliput::api::LaneId& id, int xodr_track, int xodr_lane_id,
           const maliput::api::HBounds& elevation_bounds, const road_curve::RoadCurve* road_curve,
           std::unique_ptr<road_curve::Function> lane_width, std::unique_ptr<road_curve::Function> lane_offset,
           double p0, double p1)
    : maliput::geometry_base::Lane(id),
      xodr_track_(xodr_track),
      xodr_lane_id_(xodr_lane_id),
      elevation_bounds_(elevation_bounds),
      road_curve_(road_curve),
      p0_(p0),
      p1_(p1),
      road_curve_offset_(road_curve_, lane_offset.get(), p0, p1),
      lane_width_(std::move(lane_width)),
      lane_offset_(std::move(lane_offset)) {
  MALIDRIVE_THROW_UNLESS(xodr_track >= 0);

  MALIDRIVE_THROW_UNLESS(road_curve_ != nullptr);
  MALIDRIVE_THROW_UNLESS(lane_width_ != nullptr);
  MALIDRIVE_THROW_UNLESS(lane_offset_ != nullptr);

  MALIDRIVE_IS_IN_RANGE(std::abs(lane_width_->p0() - p0), 0., road_curve_->linear_tolerance());
  MALIDRIVE_IS_IN_RANGE(std::abs(lane_width_->p1() - p1), 0., road_curve_->linear_tolerance());
  MALIDRIVE_IS_IN_RANGE(std::abs(lane_offset_->p0() - p0), 0., road_curve_->linear_tolerance());
  MALIDRIVE_IS_IN_RANGE(std::abs(lane_offset_->p1() - p1), 0., road_curve_->linear_tolerance());

  // @{ The following if clause introduces an implementation knowledge abuse.
  //    road_curve::RoadCurve::LMax() is the result of the summatory of
  //    all the arc lengths of a potential piecewise ground curve. The range
  //    road_curve::RoadCurve::p0() to road_curve::RoadCurve::p1()
  //    maps it. `lane_ground_curve_lmax` holds the fraction that belongs to
  //    this lane.
  const double lane_ground_curve_lmax = road_curve_->LMax() * (p1_ - p0_) / (road_curve_->p1() - road_curve_->p0());
  if (lane_ground_curve_lmax > road_curve_->linear_tolerance()) {
    p_from_s_ = road_curve_offset_.PFromS();
    s_from_p_ = road_curve_offset_.SFromP();
    length_ = s_from_p_(p1);
    // Numerical integration might lead to errors in length up to linear_tolerance.
    // However, p <--> s functors above do not tolerate working with values that
    // go beyond the strict range of [p0, p1] and [0, length_]. Consequently, we
    // use road_curve::OpenRangeValidator() to a) validate that s arguments in
    // functions of this class are in range with linear_tolerance and b) values
    // are adjusted to be in the open range (0, length_) with half
    // linear_tolerance as the distance between closed and open range extrema.
    s_range_validation_ = road_curve::OpenRangeValidator(0., length_, road_curve_->linear_tolerance(),
                                                         road_curve_->linear_tolerance() / 2.);
  } else {
    maliput::log()->trace("Lane {} is shorter than linear tolerance. Will not construct the RoadCurveOffset for it.",
                          id.string());
    p_from_s_ = [p0 = p0_, p1 = p1_, lane_ground_curve_lmax](double s) -> double {
      return p0 + s / lane_ground_curve_lmax * (p1 - p0);
    };
    s_from_p_ = [p0 = p0_, p1 = p1_, lane_ground_curve_lmax](double p) -> double {
      return (p - p0) / (p1 - p0) * lane_ground_curve_lmax;
    };
    length_ = s_from_p_(p1);
    // There are no numerical integrations involved in p_from_s_ and s_from_p_
    // but to mimic the behavior, we will tolerate up to linear tolerance excess
    // in s range. Then, the s value will be saturated.
    s_range_validation_ = road_curve::OpenRangeValidator(0., length_, road_curve_->linear_tolerance(), /*epsilon*/ 0.);
  }
  // @}
}

maliput::api::RBounds Lane::do_lane_bounds(double s) const {
  const double p = p_from_s_(s_range_validation_(s));
  const double width = lane_width_->f(p);
  return {-width / 2., width / 2.};
}

maliput::api::RBounds Lane::do_segment_bounds(double s) const {
  s = s_range_validation_(s);
  // TODO(agalbachicar):  Once geometry restrictions are released, we should map
  //                      this lane s coordinate into adjacent lanes' s
  //                      coordinate to properly compute the asphalt extents.

  // TODO(agalbachicar):  Remove constant width and flat geometries constraint.
  int left_lanes_count = 0;
  const maliput::api::Lane* other_lane = to_left();
  while (other_lane != nullptr) {
    left_lanes_count++;
    other_lane = other_lane->to_left();
  }

  int right_lanes_count = 0;
  other_lane = to_right();
  while (other_lane != nullptr) {
    right_lanes_count++;
    other_lane = other_lane->to_right();
  }

  const double p = p_from_s_(s);
  const double width = lane_width_->f(p);

  return {-width * (static_cast<double>(right_lanes_count) + 0.5),
          width * (static_cast<double>(left_lanes_count) + 0.5)};
}

maliput::math::Vector3 Lane::DoToBackendPosition(const maliput::api::LanePosition& lane_pos) const {
  const double p = p_from_s_(s_range_validation_(lane_pos.s()));
  return road_curve_->W({p, to_reference_r(p, lane_pos.r()), lane_pos.h()});
}

Vector3 Lane::BackendFrameToLaneFrame(const Vector3& xyz) const {
  // Gets initial estimate of `p` from the RoadCurve.
  double p{road_curve_->WInverse(xyz).x()};
  // Delta p, to be reduced iteratively.
  double dp{2.0 * road_curve_->linear_tolerance()};

  constexpr int kMaxIterations{16};
  // Correction in p computed iteratively.
  // Start of iterations.
  for (int i = 0; i < kMaxIterations && std::abs(dp) > road_curve_->linear_tolerance(); ++i) {
    // Gets the position in the INERTIAL Frame at the centerlane.
    // Saturates p to Lane's range.
    p = maliput::math::saturate(p, p0_, p1_);
    const maliput::math::Vector3 prh_at_centerlane{p, lane_offset_->f(p), 0};
    const maliput::math::Vector3 w_p{road_curve_->W(prh_at_centerlane)};
    // Gets the vector difference between w_p and `xyz`.
    const maliput::math::Vector3 w_delta{xyz - w_p};
    // Computes the centerlane derivative with respect to p.
    const maliput::math::Vector3 w_dot{road_curve_->WDot(prh_at_centerlane, lane_offset_.get())};
    // Iterative updates of `p` with Newton's method:
    // Compute correction in p from component of w_delta / w_dot.norm() parallel to centerlane:
    //   dp = (w_delta / w_dot.norm()).dot(s_hat);
    // which is equivalent to the following:
    dp = w_delta.dot(w_dot) / w_dot.dot(w_dot);
  }
  p = maliput::math::saturate(p, p0_, p1_);

  // Recompute with final value of p:
  // Gets the position in the INERTIAL Frame at the centerlane.
  const maliput::math::Vector3 prh_at_centerlane{p, 0., 0.};
  const maliput::math::Vector3 w_p{road_curve_->W(prh_at_centerlane)};
  // Gets the vector difference between w_p and `xyz`.
  const maliput::math::Vector3 w_delta{xyz - w_p};
  // Computes the orthonormal basis at p, and projects the difference onto r_hat
  // and h_hat to get each component.
  const maliput::math::Vector3 s_hat{road_curve_->SHat(prh_at_centerlane)};
  const maliput::math::Vector3 h_hat{road_curve_->HHat(p, s_hat)};
  const maliput::math::Vector3 r_hat{h_hat.cross(s_hat)};
  return {p, r_hat.dot(w_delta) - lane_offset_->f(p), h_hat.dot(w_delta)};
}

void Lane::DoToLanePositionBackend(const maliput::math::Vector3& backend_pos, maliput::api::LanePosition* lane_position,
                                   maliput::math::Vector3* nearest_backend_pos, double* distance) const {
  const maliput::math::Vector3 unconstrained_prh{BackendFrameToLaneFrame(backend_pos)};
  MALIDRIVE_IS_IN_RANGE(unconstrained_prh[0], p0_, p1_);
  const double s = s_from_p_(unconstrained_prh[0]);
  const maliput::api::RBounds segment_boundaries = segment_bounds(s);
  const double r = maliput::math::saturate(unconstrained_prh[1], segment_boundaries.min(), segment_boundaries.max());
  const maliput::api::HBounds elevation_boundaries = elevation_bounds(s, r);
  const double h =
      maliput::math::saturate(unconstrained_prh[2], elevation_boundaries.min(), elevation_boundaries.max());

  *lane_position = {s, r, h};
  *nearest_backend_pos = DoToBackendPosition(*lane_position);
  *distance = (backend_pos - *nearest_backend_pos).norm();
}

maliput::api::Rotation Lane::DoGetOrientation(const maliput::api::LanePosition& lane_pos) const {
  const double p = p_from_s_(s_range_validation_(lane_pos.s()));
  const maliput::math::RollPitchYaw rpy =
      road_curve_->Orientation({p, to_reference_r(p, lane_pos.r()), lane_pos.h()}, lane_offset_.get());
  return maliput::api::Rotation::FromRpy(rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle());
}

maliput::api::LanePosition Lane::DoEvalMotionDerivatives(const maliput::api::LanePosition& position,
                                                         const maliput::api::IsoLaneVelocity& velocity) const {
  const double p = p_from_s_(s_range_validation_(position.s()));
  const double r = to_reference_r(p, position.r());
  const double h = position.h();
  // The definition of path-length of a path along σ yields dσ = |∂W/∂p| dp
  // evaluated at (p, r, h).
  // Similarly, path-length s along the segment surface at r = r0 (which is
  // along the Lane's centerline) is related to p by ds = |∂W/∂p| dp evaluated
  // at (p, lane_offset_r(p), 0).  Chaining yields ds/dσ:
  const double ds_dsigma = road_curve_->WDot({p, to_reference_r(p, 0.), 0.}, lane_offset_.get()).norm() /
                           road_curve_->WDot({p, r, h}, lane_offset_.get()).norm();
  return {ds_dsigma * velocity.sigma_v, velocity.rho_v, velocity.eta_v};
}

}  // namespace malidrive
