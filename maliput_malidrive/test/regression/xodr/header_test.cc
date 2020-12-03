// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/xodr/header.h"

#include <gtest/gtest.h>

namespace malidrive {
namespace xodr {
namespace test {
namespace {

GTEST_TEST(Header, EqualityOperator) {
  const Header kHeader{1. /* revMajor */,
                       1. /* revMinor */,
                       "TestHeader" /* name */,
                       1.21 /* version */,
                       "Wed Sep 19 12:00:00 2018" /* date */,
                       1.1 /* north */,
                       2.2 /* south */,
                       3.3 /* east */,
                       4.4 /* west */,
                       "TestVendor" /* vendor */};
  Header header = kHeader;

  EXPECT_EQ(kHeader, header);
  header.rev_major = 32;
  EXPECT_NE(kHeader, header);
  header.rev_major = 1.;
  header.rev_minor = 10.;
  EXPECT_NE(kHeader, header);
  header.rev_minor = 1.;
  header.name = std::nullopt;
  EXPECT_NE(kHeader, header);
  header.name = "TestHeader";
  header.version = 2.5;
  EXPECT_NE(kHeader, header);
  header.version = 1.21;
  header.date = "Wed May 27 12:00:00 2018";
  EXPECT_NE(kHeader, header);
  header.date = "Wed Sep 19 12:00:00 2018";
  header.north = 8.;
  EXPECT_NE(kHeader, header);
  header.north = 1.1;
  header.south = 5.1;
  EXPECT_NE(kHeader, header);
  header.south = 2.2;
  header.east = 58.2;
  EXPECT_NE(kHeader, header);
  header.east = 3.3;
  header.west = 58.1;
  EXPECT_NE(kHeader, header);
  header.west = 4.4;
  header.vendor = "DifferentVendor";
  EXPECT_NE(kHeader, header);
}

}  // namespace
}  // namespace test
}  // namespace xodr
}  // namespace malidrive
