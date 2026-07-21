#include "math/vec3.hpp"
#include <gtest/gtest.h>

using namespace kinematics::math;

class Vec3Test : public ::testing::Test {
protected:
  Vec3 test{1.0, 2.0, 3.0};
  Vec3 ones{1.0, 1.0, 1.0};
  Vec3 axis_x{1.0, 0.0, 0.0};
  Vec3 axis_y{0.0, 1.0, 0.0};
  Vec3 axis_z{0.0, 0.0, 1.0};
};

TEST_F(Vec3Test, Initialization_StoresAllZeros) {
  Vec3 new_vec;
  EXPECT_DOUBLE_EQ(new_vec.x, 0.0);
  EXPECT_DOUBLE_EQ(new_vec.y, 0.0);
  EXPECT_DOUBLE_EQ(new_vec.z, 0.0);
}

TEST_F(Vec3Test, Subscript_GivesCorrectValue) {
  EXPECT_DOUBLE_EQ(test[0], test.x);
  EXPECT_DOUBLE_EQ(test[1], test.y);
  EXPECT_DOUBLE_EQ(test[2], test.z);
}

TEST_F(Vec3Test, Equals_ReturnsTrueForEqual) {
  Vec3 test_copy{1.0, 2.0, 3.0};
  EXPECT_TRUE(test == test_copy);
}

TEST_F(Vec3Test, Equals_ReturnsFalseForUnequal) { EXPECT_FALSE(test == ones); }

TEST_F(Vec3Test, AddAssign_InplaceElementwiseAddition) {
  test += ones;
  EXPECT_EQ(test, (Vec3{2.0, 3.0, 4.0}));
}

TEST_F(Vec3Test, Add_ElementwiseAddition) {
  Vec3 sum = test + ones;
  EXPECT_EQ(sum, (Vec3{2.0, 3.0, 4.0}));
}

TEST_F(Vec3Test, SubtractAssign_InplaceElementwiseSubtraction) {
  test -= ones;
  EXPECT_EQ(test, (Vec3{0.0, 1.0, 2.0}));
}

TEST_F(Vec3Test, Subtract_ElementwiseSubtraction) {
  Vec3 sub = test - ones;
  EXPECT_EQ(sub, (Vec3{0.0, 1.0, 2.0}));
}

TEST_F(Vec3Test, Unary_NegatesVector) {
  Vec3 min_test = -test;
  EXPECT_DOUBLE_EQ(min_test.x, -test.x);
  EXPECT_DOUBLE_EQ(min_test.y, -test.y);
  EXPECT_DOUBLE_EQ(min_test.z, -test.z);
}

TEST_F(Vec3Test, MultiplyAssign_Scalar_InplaceElementwiseMultiplication) {
  test *= 5.0;
  EXPECT_EQ(test, (Vec3{5.0, 10.0, 15.0}));
}

TEST_F(Vec3Test, Multiply_Scalar_ElementewiseMultiplicatoin) {
  Vec3 mul = test * 3.0;
  EXPECT_EQ(mul, (Vec3{3.0, 6.0, 9.0}));
}

TEST_F(Vec3Test, DivideAssign_Scalar_InplaceElementwiseDivision) {
  Vec3 vector{5.0, 5.0, 5.0};
  vector /= 5.0;
  EXPECT_EQ(vector, ones);
}

TEST_F(Vec3Test, Divide_Scalar_ElementwiseDivision) {
  Vec3 vector{2.0, 4.0, 6.0};
  Vec3 div = vector / 2.0;
  EXPECT_EQ(div, test);
}

TEST_F(Vec3Test, Divide_DivisionByZero_AssertFails) {
  EXPECT_DEATH(test / 0.0, "Vec3::operator/= - division by zero");
}

TEST_F(Vec3Test, Norm_ReturnsNormOfVector) {
  EXPECT_DOUBLE_EQ(test.norm(), (std::sqrt(14)));
}

TEST_F(Vec3Test, Normalize_NormalizesVector) {
  Vec3 test_normalized = test / (std::sqrt(14));
  EXPECT_EQ(test.normalize(), test_normalized);
}

TEST_F(Vec3Test, IsUnit_ReturnTrueIfUnitVector) {
  EXPECT_TRUE((test.normalize()).is_unit());
  EXPECT_TRUE(axis_x.is_unit());
  EXPECT_TRUE(axis_y.is_unit());
  EXPECT_TRUE(axis_z.is_unit());
}

TEST_F(Vec3Test, IsUnit_ReturnFalseIfNotUnitVector) {
  EXPECT_FALSE(test.is_unit());
  EXPECT_FALSE(ones.is_unit());
}

TEST_F(Vec3Test, Dot_ReturnDotProductBetweenVectors) {
  EXPECT_DOUBLE_EQ(axis_x.dot(axis_y), 0.0);
  EXPECT_DOUBLE_EQ(axis_y.dot(axis_z), 0.0);
  EXPECT_DOUBLE_EQ(axis_z.dot(axis_x), 0.0);
  Vec3 slope1xy{1.0, 1.0, 0.0};
  EXPECT_DOUBLE_EQ(axis_x.dot(slope1xy),
                   (axis_x.norm() * slope1xy.norm() * std::cos(kPi / 4)));
}

TEST_F(Vec3Test, Cross_ReturnCrossProductVector) {
  EXPECT_EQ(axis_x.cross(axis_y), axis_z);
  EXPECT_EQ(axis_y.cross(axis_z), axis_x);
}
