// Copyright 2020 Toyota Research Institute

#include <gtest/gtest.h>

namespace malidrive {
namespace test {

GTEST_TEST(Demo_Test, DummyTest) { EXPECT_TRUE(true); }

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace test
}  // namespace malidrive
