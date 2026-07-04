#include "math/quaternion.hpp"
#include <gtest/gtest.h>

using namespace kinematics::math;

class QuaternionTest : public ::testing::Test {
protected:
  Quaternion identity = Quaternion::identity();
  Quaternion q90x = Quaternion::from_axis_angle(Vec3{1.0, 0.0, 0.0}, kPi / 2);
  Quaternion q90y = Quaternion::from_axis_angle(Vec3{0.0, 1.0, 0.0}, kPi / 2);
  Quaternion q90z = Quaternion::from_axis_angle(Vec3{0.0, 0.0, 1.0}, kPi / 2);
  Quaternion q180x = Quaternion::from_axis_angle(Vec3{1.0, 0.0, 0.0}, kPi);
  Quaternion q0x = Quaternion::from_axis_angle(Vec3{1.0, 0.0, 0.0}, 0.0);

  Quaternion test{1.0, 2.0, 3.0, 4.0};
  Quaternion min_test{-1.0, -2.0, -3.0, -4.0};
  Vec3 unit_x{1.0, 0.0, 0.0};
  Vec3 unit_y{0.0, 1.0, 0.0};
  Vec3 unit_z{0.0, 0.0, 1.0};
  Mat3 identity_mat{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
};

TEST_F(QuaternionTest, Identity_StoresCorrectValue) {
  // identity
  EXPECT_DOUBLE_EQ(identity.w, 1.0);
  EXPECT_DOUBLE_EQ(identity.x, 0.0);
  EXPECT_DOUBLE_EQ(identity.y, 0.0);
  EXPECT_DOUBLE_EQ(identity.z, 0.0);
}

TEST_F(QuaternionTest, FromAxisAngle_90x_StoreCorrectValue) {
  // from axis angle
  EXPECT_NEAR(q90x.w, std::sqrt(2.0) / 2.0, 1e-9);
  EXPECT_NEAR(q90x.x, std::sqrt(2.0) / 2.0, 1e-9);
  EXPECT_DOUBLE_EQ(q90x.y, 0.0);
  EXPECT_DOUBLE_EQ(q90x.z, 0.0);
}

TEST_F(QuaternionTest, IsUnit_TrueForUnitQuaternion) {
  EXPECT_TRUE(identity.is_unit());
}

TEST_F(QuaternionTest, IsUnit_FalseForNotUnit) {
  Quaternion not_unit{2.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(not_unit.is_unit());
}

TEST_F(QuaternionTest, FromAxisAngle_Zero_ProducesIdentity) {
  // from axis angle at zero is identity
  EXPECT_DOUBLE_EQ(q0x.w, 1.0);
  EXPECT_DOUBLE_EQ(q0x.x, 0.0);
  EXPECT_DOUBLE_EQ(q0x.y, 0.0);
  EXPECT_DOUBLE_EQ(q0x.z, 0.0);
}

TEST_F(QuaternionTest, FromMatrix_ConvertsCorrectly) {
  Mat3 mat{0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  Quaternion q_rot = Quaternion::from_matrix(mat);
  EXPECT_NEAR(q_rot.w, q90z.w, 1e-9);
  EXPECT_NEAR(q_rot.x, q90z.x, 1e-9);
  EXPECT_NEAR(q_rot.y, q90z.y, 1e-9);
  EXPECT_NEAR(q_rot.z, q90z.z, 1e-9);
}

TEST_F(QuaternionTest, Equals_TrueForEqual) {
  Quaternion test_eql{1.0, 2.0, 3.0, 4.0};
  EXPECT_TRUE(test == test_eql);
}

TEST_F(QuaternionTest, Equals_FalseForUnequal) { EXPECT_FALSE(q90x == q90y); }

TEST_F(QuaternionTest, Add_StoresCorrectValue) {
  Quaternion sum = identity + identity;
  EXPECT_DOUBLE_EQ(sum.w, 2.0);
  EXPECT_DOUBLE_EQ(sum.x, 0.0);
  EXPECT_DOUBLE_EQ(sum.y, 0.0);
  EXPECT_DOUBLE_EQ(sum.z, 0.0);
}

TEST_F(QuaternionTest, Subtract_StoresCorrectValue) {
  Quaternion sub = identity - identity;
  EXPECT_DOUBLE_EQ(sub.w, 0.0);
  EXPECT_DOUBLE_EQ(sub.x, 0.0);
  EXPECT_DOUBLE_EQ(sub.y, 0.0);
  EXPECT_DOUBLE_EQ(sub.z, 0.0);
}

TEST_F(QuaternionTest, Negate_ReturnsNegativeQuaternion) {
  Quaternion neg_q90x = -q90x;
  EXPECT_EQ(neg_q90x.w, -q90x.w);
  EXPECT_EQ(neg_q90x.x, -q90x.x);
  EXPECT_EQ(neg_q90x.y, -q90x.y);
  EXPECT_EQ(neg_q90x.z, -q90x.z);
}

TEST_F(QuaternionTest, Multiply_Scalar_PrePostMultiply_Commutative) {
  Quaternion scaled_fac_post = identity * 2.0;
  EXPECT_DOUBLE_EQ(scaled_fac_post.w, 2.0);
  EXPECT_DOUBLE_EQ(scaled_fac_post.x, 0.0);
  EXPECT_DOUBLE_EQ(scaled_fac_post.y, 0.0);
  EXPECT_DOUBLE_EQ(scaled_fac_post.z, 0.0);

  Quaternion scaled_fac_pre = 2.0 * identity;
  EXPECT_EQ(scaled_fac_pre, scaled_fac_post);
}

TEST_F(QuaternionTest, Multiply_Quaternion_HamiltonProduct) {
  Quaternion q_prod = q90x * q90x;
  EXPECT_EQ(q_prod, q180x);
}

TEST_F(QuaternionTest, Multiply_Quaternion_HamiltonProduct_NotCommutative) {
  Quaternion q_prod_xy = q90x * q90y;
  Quaternion q_prod_yx = q90y * q90x;
  EXPECT_NE(q_prod_xy, q_prod_yx);
}

TEST_F(QuaternionTest, Multiply_Quaternion_Identity_ReturnsOther) {
  Quaternion q_prod = q90x * identity;
  EXPECT_EQ(q_prod, q90x);
  q_prod = identity * q90x;
  EXPECT_EQ(q_prod, q90x);
}

TEST_F(QuaternionTest, Dot_Orthogonal_ReturnsZero) {
  double dot_prod = q0x.dot(q180x);
  EXPECT_NEAR(dot_prod, 0.0, 1e-9);
}

TEST_F(QuaternionTest, Dot_Identity_ReturnsOne) {
  EXPECT_DOUBLE_EQ(identity.dot(identity), 1.0);
}

TEST_F(QuaternionTest, Dot_Commutative) {
  EXPECT_EQ(q90x.dot(q90y), q90y.dot(q90x));
}

TEST_F(QuaternionTest, Dot_OppositeHemisphere_ReturnsLessThanZero) {
  Quaternion other = -q90x;
  double dot_prod = q90x.dot(other);
  EXPECT_LT(dot_prod, 0.0);
}

TEST_F(QuaternionTest, Norm_ReturnsMagnitude) {
  Quaternion q = 2.0 * identity;
  double norm_q = q.norm();
  EXPECT_NEAR(norm_q, 2.0, 1e-9);
}

TEST_F(QuaternionTest, Conjugate_FlipsVectorPart) {
  Quaternion conjugate_q90x = q90x.conjugate();
  EXPECT_EQ(conjugate_q90x.x, -q90x.x);
  EXPECT_EQ(conjugate_q90x.y, -q90x.y);
  EXPECT_EQ(conjugate_q90x.z, -q90x.z);
}

TEST_F(QuaternionTest, Conjugate_KeepsScalarPart) {
  Quaternion conjugate_q90x = q90x.conjugate();
  EXPECT_EQ(conjugate_q90x.w, q90x.w);
}

TEST_F(QuaternionTest, DoubleConjugate_ReturnsOriginal) {
  Quaternion double_conjugate = q90x.conjugate().conjugate();
  EXPECT_EQ(double_conjugate, q90x);
}

TEST_F(QuaternionTest, Inverse_RightMultiply_ReturnsIdentity) {
  Quaternion inverse_q90x = q90x.inverse();
  Quaternion prod = q90x * inverse_q90x;
  EXPECT_EQ(prod, identity);
}

TEST_F(QuaternionTest, Inverse_LeftMultiply_ReturnsIdentity) {
  Quaternion inverse_q90x = q90x.inverse();
  Quaternion prod = inverse_q90x * q90x;
  EXPECT_EQ(prod, identity);
}

TEST_F(QuaternionTest, Inverse_UnitQuaternion_EqualsConjugate) {
  EXPECT_TRUE(q90x.is_unit());
  EXPECT_EQ(q90x.inverse(), q90x.conjugate());
}

TEST_F(QuaternionTest, Rotate_Identity_VectorRemainsUnchanged) {
  EXPECT_EQ(unit_x, identity.rotate(unit_x));
}

TEST_F(QuaternionTest, Rotate_90x_UnitY_ProducesUnitZ) {
  EXPECT_EQ(unit_z, q90x.rotate(unit_y));
}

TEST_F(QuaternionTest, Rotate_90x_UnitZ_ProducesNegUnitY) {
  EXPECT_EQ(-unit_y, q90x.rotate(unit_z));
}

TEST_F(QuaternionTest, Rotate_90z_UnitX_ProducesUnitY) {
  EXPECT_EQ(unit_y, q90z.rotate(unit_x));
}

TEST_F(QuaternionTest, Rotate_180x_UnitY_ProducesNegUnitY) {
  EXPECT_EQ(-unit_y, q180x.rotate(unit_y));
}

TEST_F(QuaternionTest, ToRotationMatrix_StoresCorrectValues) {
  Mat3 rot_z90{0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  EXPECT_EQ(rot_z90, q90z.to_rotation_matrix());
}

TEST_F(QuaternionTest, Slerp_Zero_ReturnsSelf) {
  EXPECT_EQ(q90x, q90x.slerp(q90y, 0.0));
}

TEST_F(QuaternionTest, Slerp_One_ReturnsOther) {
  EXPECT_EQ(q90y, q90x.slerp(q90y, 1.0));
}

TEST_F(QuaternionTest, Slerp_SameQuaternion_ReturnsSelf) {
  EXPECT_EQ(q90x, q90x.slerp(q90x, 0.5));
  EXPECT_EQ(q90x, q90x.slerp(q90x, 0.0));
  EXPECT_EQ(q90x, q90x.slerp(q90x, 1.0));
}

TEST_F(QuaternionTest, Slerp_ReturnsUnitQuaternion) {
  Quaternion result = q90x.slerp(q90y, 0.0);
  EXPECT_TRUE(result.is_unit());
}

TEST_F(QuaternionTest, Slerp_OppositeHemisphere_TakesShortestPath) {
  Quaternion mid = q90x.slerp(-q90x, 0.5);
  double dot_prod = q90x.dot(mid);
  EXPECT_GE(dot_prod, 0.0);
}

TEST_F(QuaternionTest, Slerp_MidPoint_IsEquidistant) {
  Quaternion mid = q90x.slerp(-q90x, 0.5);
  double dp_mid_q90x = q90x.dot(mid);
  double dp_mid_min_q90x = (-q90x).dot(mid);

  double chordal_q90x = 1 - (dp_mid_q90x * dp_mid_q90x);
  double chordal_min_q90x = 1 - (dp_mid_min_q90x * dp_mid_min_q90x);

  EXPECT_NEAR(std::abs(dp_mid_q90x), std::abs(dp_mid_min_q90x), 1e-9);
  EXPECT_NEAR(chordal_q90x, chordal_min_q90x, 1e-9);
}

TEST_F(QuaternionTest, ToRotationMatrix_ReturnsIdentityMatrix) {
  EXPECT_EQ(identity_mat, identity.to_rotation_matrix());
}

TEST_F(QuaternionTest, ToRotationMatrix_90x_StoresCorrectValues) {
  Mat3 rot_90x{1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0};
  EXPECT_EQ(rot_90x, q90x.to_rotation_matrix());
}

TEST_F(QuaternionTest, ToRotationMatrix_IsOrthogonal) {
  Mat3 mat = q90y.to_rotation_matrix();
  EXPECT_TRUE(mat.is_orthogonal());
}

TEST_F(QuaternionTest, ToRotationMatrix_DeterminantIsOne) {
  Mat3 mat = q90y.to_rotation_matrix();
  EXPECT_NEAR(mat.determinant(), 1.0, 1e-9);
}

TEST_F(QuaternionTest, FromMatrix_Identity_ReturnsUnitQuaternion) {
  Quaternion test = Quaternion::from_matrix(identity_mat);
  EXPECT_EQ(test, identity);
};

TEST_F(QuaternionTest, FromMatrix_RoundTrip_RepresentSameRotation) {
  Quaternion test = Quaternion::from_matrix(identity_mat);
  Mat3 test_mat = test.to_rotation_matrix();
  EXPECT_EQ(identity_mat, test_mat);
}

TEST_F(QuaternionTest, Invariant_FromAxisAngle_AlwaysUnit) {
  EXPECT_TRUE(identity.is_unit());
  EXPECT_TRUE(q90x.is_unit());
  EXPECT_TRUE(q180x.is_unit());
  EXPECT_TRUE(q90y.is_unit());
  EXPECT_TRUE(q90z.is_unit());
}

TEST_F(QuaternionTest, Invariant_QuaternionTimesInverse_IsIdentity) {
  EXPECT_EQ((identity * identity.inverse()), identity);
  EXPECT_EQ((q90x * q90x.inverse()), identity);
  EXPECT_EQ((q90y * q90y.inverse()), identity);
  EXPECT_EQ((q90z * q90z.inverse()), identity);
  EXPECT_EQ((q180x * q180x.inverse()), identity);
}

TEST_F(QuaternionTest, Invariant_ChainedRotation_RemainsUnit) {
  Quaternion chained = q90x * q90y;
  EXPECT_TRUE(chained.is_unit());

  chained = q90z * q90y * q90z;
  EXPECT_TRUE(chained.is_unit());

  chained = q90z * q90x * q90y * q180x;
  EXPECT_TRUE(chained.is_unit());

  chained = q90x * q90x * q90x * q90x;
  EXPECT_TRUE(chained.is_unit());
}
