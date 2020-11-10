// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/base/malidrive_segment.h"

#include <cmath>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput_malidrive/road_curve/cubic_polynomial.h"
#include "maliput_malidrive/road_curve/function.h"
#include "maliput_malidrive/road_curve/line_ground_curve.h"
#include "maliput_malidrive/road_curve/road_curve.h"

namespace malidrive {
namespace test {
namespace {

using maliput::math::Vector2;

std::unique_ptr<road_curve::Function> MakeZeroCubicPolynomial(double p0, double p1, double linear_tolerance) {
  return std::make_unique<road_curve::CubicPolynomial>(0., 0., 0., 0., p0, p1, linear_tolerance);
}

class MalidriveSegmentTest : public ::testing::Test {
 protected:
  const maliput::api::SegmentId kId{"dut"};
  const double kP0{0.};
  const double kP1{100.};
  const double kLinearTolerance{1e-13};
  const double kScaleLength{1.};
  const Vector2 kXy0{10., 10.};
  const Vector2 kDXy{(kP1 - kP0) * std::sqrt(2.) / 2., (kP1 - kP0) * std::sqrt(2.) / 2.};

  void SetUp() override {
    road_curve_ = std::make_unique<road_curve::RoadCurve>(
        kLinearTolerance, kScaleLength,
        std::make_unique<road_curve::LineGroundCurve>(kLinearTolerance, kXy0, kDXy, kP0, kP1),
        MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance), MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance));
    reference_line_offset_ = MakeZeroCubicPolynomial(kP0, kP1, kLinearTolerance);
  }
  std::unique_ptr<road_curve::RoadCurve> road_curve_;
  std::unique_ptr<road_curve::Function> reference_line_offset_;
};

TEST_F(MalidriveSegmentTest, Constructor) {
  EXPECT_NO_THROW(MalidriveSegment(kId, road_curve_.get(), reference_line_offset_.get(), kP0, kP1));
}

TEST_F(MalidriveSegmentTest, ConstructorAssertions) {
  EXPECT_THROW(MalidriveSegment(kId, nullptr, reference_line_offset_.get(), kP0, kP1),
               maliput::common::assertion_error);
  EXPECT_THROW(MalidriveSegment(kId, road_curve_.get(), nullptr, kP0, kP1), maliput::common::assertion_error);
  const double kWrongP0{5.};
  const double kWrongP1{3.};
  EXPECT_THROW(MalidriveSegment(kId, road_curve_.get(), reference_line_offset_.get(), kWrongP0, kWrongP1),
               maliput::common::assertion_error);
}

TEST_F(MalidriveSegmentTest, RoadCurve) {
  const MalidriveSegment dut(kId, road_curve_.get(), reference_line_offset_.get(), kP0, kP1);
  EXPECT_EQ(road_curve_.get(), dut.road_curve());
}

TEST_F(MalidriveSegmentTest, ReferenceLineOffset) {
  const MalidriveSegment dut(kId, road_curve_.get(), reference_line_offset_.get(), kP0, kP1);
  EXPECT_EQ(reference_line_offset_.get(), dut.reference_line_offset());
}

}  // namespace
}  // namespace test
}  // namespace malidrive
