#include "gtest/gtest.h"

TEST(StringEqualityTest, EqualStrings)
{
    std::string str1 = "Hello Bazel!";
    std::string str2 = "Hello Bazel!";
    EXPECT_EQ(str1, str2);
}

TEST(EqualityTest, PositiveNos)
{
    EXPECT_EQ(144, (12*12));
}
