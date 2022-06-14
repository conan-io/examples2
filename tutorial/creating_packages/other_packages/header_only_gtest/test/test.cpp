#include "../include/sum.h"

#include <limits.h>

#include "gtest/gtest.h"
namespace {

    TEST(SumTest, BasicSum) {
      EXPECT_EQ(4, sum(1, 3));
      EXPECT_EQ(0, sum(0, 0));
    }
}