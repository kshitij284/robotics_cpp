#include "kinematics/jacobian.hpp"
#include <gtest/gtest.h>

using namespace kinematics::math;

class JacobianTest : public ::testing::Test {
protected:
  Jacobian j_default;
  Jacobian j_init;

public:
  JacobianTest() : j_init(3) {}
};

TEST_F(JacobianTest, DefaultInitialization_ZeroDimension) {
  EXPECT_EQ(j_default.rows(), 0);
  EXPECT_EQ(j_default.cols(), 0);
}

TEST_F(JacobianTest, InitializedwDOF_CorrectDimensions) {
  EXPECT_EQ(j_init.rows(), 6);
  EXPECT_EQ(j_init.cols(), 3);
}

TEST_F(JacobianTest, SubscriptRoundTrip_StoresAndRetrivesCorrectValue) {
  double val = 0.0;
  for (long i = 0; i < 3; i++) {
    for (long j = 0; j < 3; j++) {
      j_init[i, j] = val++;
    }
  }
  double test = 0.0;
  for (long i = 0; i < 3; i++) {
    for (long j = 0; j < 3; j++) {
      EXPECT_DOUBLE_EQ((j_init[i, j]), test);
      test++;
    }
  }
}

TEST_F(JacobianTest, Subscript_OutOfBound_AssertAbort) {
  EXPECT_DEATH((j_init[6, 4]),
               "Jacobian::subscript operator - out of bound access");
}

TEST_F(JacobianTest, Copy_ProducesIndependentEqualMatrices) {
  double val = 0.0;
  for (long i = 0; i < 3; i++) {
    for (long j = 0; j < 3; j++) {
      j_init[i, j] = val++;
    }
  }
  Jacobian j_copy = j_init;
  EXPECT_EQ(j_init, j_copy);
}

TEST_F(JacobianTest, Move_ProducesIndependentEqualMatrices) {
  double val = 0.0;
  for (long i = 0; i < 3; i++) {
    for (long j = 0; j < 3; j++) {
      j_init[i, j] = val++;
    }
  }
  Jacobian j_copy = j_init;
  Jacobian j_moved(std::move(j_copy));
  EXPECT_EQ(j_init, j_moved);
}
