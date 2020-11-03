// Copyright 2019 Toyota Research Institute
#include "maliput_malidrive/common/macros.h"

#include <exception>
#include <string>

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"

// This test file only tests the behavior of non copy-pasted macros that come
// from Maliput / Delphyne.

namespace malidrive {
namespace test {

GTEST_TEST(MalidriveInternalMacros, ThrowIfNotInRangeTest) {
  EXPECT_THROW({ MALIDRIVE_IS_IN_RANGE(2., 3., 4.); }, maliput::common::assertion_error);
  EXPECT_NO_THROW({ MALIDRIVE_IS_IN_RANGE(3.5, 3., 4.); });
  EXPECT_NO_THROW({ MALIDRIVE_IS_IN_RANGE(3., 3., 4.); });
  EXPECT_NO_THROW({ MALIDRIVE_IS_IN_RANGE(4., 3., 4.); });
}

namespace {

// Dumb class for testing purposes.
class OxymoronException final : public std::runtime_error {
 public:
  explicit OxymoronException(const std::string& msg) : std::runtime_error(msg) {}
};

}  // namespace

GTEST_TEST(MalidriveInternalMacros, ValidateTest) {
  EXPECT_THROW({ MALIDRIVE_VALIDATE(true == false, OxymoronException, "Exception message goes here."); },
               OxymoronException);

  EXPECT_NO_THROW({ MALIDRIVE_VALIDATE(true, OxymoronException, "Exception message goes here."); });
}

}  // namespace test
}  // namespace malidrive
