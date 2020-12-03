// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/road_curve/open_range_validator.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace road_curve {
namespace test {

GTEST_TEST(OpenRangeValidatorTest, ConstructorValidationTest) {
  const double kMin{0.5};
  const double kMax{3.};
  const double kTolerance{1e-3};
  const double kEpsilon{1e-5};

  // min > max
  EXPECT_THROW({ OpenRangeValidator(kMax, kMin, kTolerance, kEpsilon); }, std::runtime_error);

  // epsilon > tolerance
  EXPECT_THROW({ OpenRangeValidator(kMin, kMax, kEpsilon, kTolerance); }, std::runtime_error);

  // min + epsilon > max (applies the other way around, it's specially provided
  // in constructor code to validate numerical error).
  EXPECT_THROW({ OpenRangeValidator(kMin, kMax, 2 * kMax, kMax); }, std::runtime_error);
}

GTEST_TEST(OpenRangeValidatorTest, RangeTest) {
  const double kMin{0.5};
  const double kMax{3.};
  const double kTolerance{1e-3};
  const double kEpsilon{1e-5};

  const OpenRangeValidator dut(kMin, kMax, kTolerance, kEpsilon);
  // In the middle of the range.
  {
    const double kS{2.};
    EXPECT_DOUBLE_EQ(dut(kS), kS);
  }
  // In the maximum of the range.
  {
    const double kS{kMax};
    EXPECT_DOUBLE_EQ(dut(kS), kS - kEpsilon);
  }
  // In the minimum of the range.
  {
    const double kS{kMin};
    EXPECT_DOUBLE_EQ(dut(kS), kS + kEpsilon);
  }
  // Exceeding the maximum but within linear tolerance.
  {
    const double kS{kMax + kTolerance / 2.};
    EXPECT_DOUBLE_EQ(dut(kS), kMax - kEpsilon);
  }
  // Exceeding the minimum but within linear tolerance.
  {
    const double kS{kMin - kTolerance / 2.};
    EXPECT_DOUBLE_EQ(dut(kS), kMin + kEpsilon);
  }
  // Expects throw because of out of bounds.
  {
    const double kS{6.};
    EXPECT_THROW({ dut(kS); }, std::runtime_error);
  }
}

}  // namespace test
}  // namespace road_curve
}  // namespace malidrive
