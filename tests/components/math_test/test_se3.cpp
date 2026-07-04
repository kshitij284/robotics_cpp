#include "math/quaternion.hpp"
#include "math/se3.hpp"
#include <gtest/gtest.h>

using namespace kinematics::math;

class SE3Test : public ::testing::Test {
protected:
  SE3 se3_default{};
  SE3 identity = SE3::identity();
  Quaternion quat_90x = Quaternion::from_axis_angle(Vec3{1, 0, 0}, kPi / 2);
  Mat3 mat_identity = Mat3::identity();
  Mat3 mat_90x = Mat3::m90x();
  Mat3 mat_90z = Mat3::m90z();
  Mat3 mat_180x{1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0};
  Vec3 vec_zero{0.0, 0.0, 0.0};
  Vec3 vec_tran_1x{1.0, 0.0, 0.0};
};

TEST_F(SE3Test, Initializer_StoresCorrectRotationValues) {
  EXPECT_EQ(se3_default.rotation(), mat_identity);
}

TEST_F(SE3Test, Initializer_StoresCorrectTranslationValues) {
  EXPECT_EQ(se3_default.translation(), vec_zero);
}

TEST_F(SE3Test, Initializer_WithOnlyRotation_StoresCorrectValues) {
  SE3 se3_r90x_t0(mat_90x);
  EXPECT_EQ(se3_r90x_t0.rotation(), mat_90x);
  EXPECT_EQ(se3_r90x_t0.translation(), vec_zero);
}

TEST_F(SE3Test, Initializer_WithOnlyTranslation_StoresCorrectValues) {
  SE3 se3_r0_t1x(vec_tran_1x);
  EXPECT_EQ(se3_r0_t1x.rotation(), mat_identity);
  EXPECT_EQ(se3_r0_t1x.translation(), vec_tran_1x);
}

TEST_F(SE3Test, FromRotation_StoresCorrectValues) {
  SE3 se3_mat90x = SE3::from_rotation(mat_90x);
  EXPECT_EQ(se3_mat90x.rotation(), mat_90x);
  EXPECT_EQ(se3_mat90x.translation(), vec_zero);
}
TEST_F(SE3Test, FromTranslation_StoresCorrectValues) {
  SE3 se3_t1x = SE3::from_translation(vec_tran_1x);
  EXPECT_EQ(se3_t1x.rotation(), mat_identity);
  EXPECT_EQ(se3_t1x.translation(), vec_tran_1x);
}
TEST_F(SE3Test, FromQuaternionTranslation_StoresCorrectValues) {
  SE3 se3_q90x_t1x = SE3::from_quaternion_translation(quat_90x, vec_tran_1x);
  EXPECT_EQ(se3_q90x_t1x.rotation(), mat_90x);
  EXPECT_EQ(se3_q90x_t1x.translation(), vec_tran_1x);
}

TEST_F(SE3Test, Multiply_ReturnsCompose) {
  SE3 se3_r90x_t0 = SE3::from_rotation(mat_90x);
  SE3 se3_compose = se3_r90x_t0 * se3_r90x_t0;
  EXPECT_EQ(se3_compose.rotation(), mat_180x);
  EXPECT_EQ(se3_compose.translation(), vec_zero);
}

TEST_F(SE3Test, Multiply_ComposesTranslationCorrectly) {
  SE3 se3_r90x_t1x = SE3(mat_90x, Vec3{1.0, 0.0, 0.0});
  SE3 se3_rI_t1y = SE3(mat_identity, Vec3{0.0, 1.0, 0.0});
  SE3 result = se3_r90x_t1x * se3_rI_t1y;
  // new t_result = mat_90x * (0, 1, 0) + (1, 0, 0) == (0, 0, 1) + (1, 0, 0) ==
  // (1, 0, 1)
  EXPECT_EQ(result.translation(), (Vec3{1.0, 0.0, 1.0}));
}

TEST_F(SE3Test, ActOnPoint_ReturnTransformedPointVec) {
  SE3 se3_r90z_t0 = SE3::from_rotation(mat_90z);
  Vec3 point_1x{1.0, 0.0, 0.0};
  Vec3 rotated_point{0.0, 1.0, 0.0};
  EXPECT_EQ(se3_r90z_t0.act_on_point(point_1x), rotated_point);
}

TEST_F(SE3Test, Inverse_ReturnsIdentityWhenComposedWithSelf) {
  SE3 se3_r90x_t0 = SE3::from_rotation(mat_90x);
  SE3 inv = se3_r90x_t0.inverse();
  EXPECT_EQ((se3_r90x_t0 * inv), identity);
}
