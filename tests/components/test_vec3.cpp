#include "vec3.hpp"
#include <gtest/gtest.h>

using namespace kinematics::math;

TEST(VecTest, Equality) {
  Vec3 a{1.0, 2.0, 3.0};
  Vec3 b{1.0, 2.0, 3.0};

  EXPECT_TRUE(a == b);

  Vec3 c{2.0, 3.0, 4.0};

  EXPECT_FALSE(a == c);
}
