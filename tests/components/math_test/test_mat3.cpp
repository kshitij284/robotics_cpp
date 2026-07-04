#include "math/mat3.hpp"
#include <gtest/gtest.h>

using namespace kinematics::math;

class Mat3Test : public ::testing::Test {
protected:
  Mat3 mat;
  Mat3 identity = Mat3::identity();
  Mat3 zero = Mat3::zero();
  Mat3 m90x = Mat3::m90x();
  Mat3 m90y = Mat3::m90y();
  Mat3 m90z = Mat3::m90z();
  Mat3 ones{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  Mat3 A{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  Mat3 A_transpose{1.0, 4.0, 7.0, 2.0, 5.0, 8.0, 3.0, 6.0, 9.0};
  Mat3 B{9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0};
  Mat3 knownAB{30.0, 24.0, 18.0, 84.0, 69.0, 54.0, 138.0, 114.0, 90.0};

  Mat3 diagonal{2.0, 0.0, 0.0, 0.0, 7.0, 0.0, 0.0, 0.0, 3.0};
  Mat3 upper_triangle{21.0, 1.0, 3.0, 0.0, 2.0, 5.0, 0.0, 0.0, 1.0};
  Mat3 lower_triangle{2.0, 0.0, 0.0, 1.0, 0.5, 0.0, 3.0, 5.0, 42.0};
  Mat3 non_singular{2.0, 0.0, 1.0, 1.0, 3.0, 2.0, 1.0, 0.0, 1.0};
  Mat3 magic{2.0, 7.0, 6.0, 9.0, 5.0, 1.0, 4.0, 3.0, 8.0};
};

TEST_F(Mat3Test, Initialization_StoresIdentity) {
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      if (i == j) {
        EXPECT_DOUBLE_EQ((mat[i, j]), 1.0);
      } else {
        EXPECT_DOUBLE_EQ((mat[i, j]), 0.0);
      }
    }
  }
}

TEST_F(Mat3Test, Identity_StoresCorrectValues) {
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      if (i == j) {
        EXPECT_DOUBLE_EQ((identity[i, j]), 1.0);
      } else {
        EXPECT_DOUBLE_EQ((identity[i, j]), 0.0);
      }
    }
  }
}

TEST_F(Mat3Test, Zero_StoresCorrectValues) {
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      EXPECT_DOUBLE_EQ((zero[i, j]), 0.0);
    }
  }
}

TEST_F(Mat3Test, Equals_ReturnsTrueForEqual) {
  EXPECT_TRUE(mat == identity);
  EXPECT_TRUE(identity == identity);
}

TEST_F(Mat3Test, Equals_ReturnsFalseForUnequal) {
  EXPECT_FALSE(mat == zero);
  EXPECT_FALSE(identity == zero);
}

TEST_F(Mat3Test, AddAssign_InplaceElementWiseAddition) {
  Mat3 test = zero;
  test += identity;
  EXPECT_EQ(test, identity);
}

TEST_F(Mat3Test, Add_ElementWiseAddition) {
  Mat3 sum = identity + identity;
  Mat3 expected{2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0};
  EXPECT_EQ(expected, sum);
}

TEST_F(Mat3Test, SubtractAssign_InplaceElementWiseSubtraction) {
  Mat3 test = identity;
  test -= identity;
  EXPECT_EQ(test, zero);
}

TEST_F(Mat3Test, Subtract_ElementWiseSubtraction) {
  Mat3 test = identity - identity;
  EXPECT_EQ(test, zero);
}

TEST_F(Mat3Test, MultiplyAssign_Scalar_InplaceElementwiseMultiplication) {
  Mat3 test = ones;
  test *= 3.0;
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      EXPECT_DOUBLE_EQ((test[i, j]), 3.0);
    }
  }
}

TEST_F(Mat3Test, Multiply_Scalar_ElementwiseMultiplication) {
  Mat3 test = ones * 3.0;
  EXPECT_EQ(ones *= 3.0, test);
}

TEST_F(Mat3Test, Negation_NegatesAllElements) {
  Mat3 test = -A;
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      EXPECT_DOUBLE_EQ(-1 * (A[i, j]), (test[i, j]));
    }
  }
}

TEST_F(Mat3Test, MultiplyAssign_Matrix_MatrixMultiplicationInplace) {
  A *= B;
  EXPECT_EQ(A, knownAB);
}

TEST_F(Mat3Test, Multiply_Matrix_MatrixMultiplication) {
  EXPECT_EQ(knownAB, (A * B));
}

TEST_F(Mat3Test, DivideAssign_Scalar_ElementwiseInplaceDivision) {
  Mat3 twos = ones * 2;
  twos /= 2;
  EXPECT_EQ(twos, ones);
}

TEST_F(Mat3Test, Divide_Scalar_ElementwiseDivision) {
  Mat3 twos = ones * 2.0;
  Mat3 div_twos = twos / 2.0;
  EXPECT_EQ(div_twos, ones);
}

TEST_F(Mat3Test, TransposeInplace) {
  EXPECT_EQ(A.transpose_inplace(), A_transpose);
}

TEST_F(Mat3Test, Transpose_ReturnTranspose) {
  Mat3 transposed = A.transpose();
  EXPECT_EQ(transposed, A_transpose);
}

TEST_F(Mat3Test, IsSymmetric_TrueIfSymmetric) {
  EXPECT_TRUE(identity.is_symmetric());
  EXPECT_TRUE(zero.is_symmetric());
  EXPECT_TRUE(ones.is_symmetric());
}

TEST_F(Mat3Test, IsSymmetric_FalseIfAsymmetric) {
  EXPECT_FALSE(A.is_symmetric());
  EXPECT_FALSE(A_transpose.is_symmetric());
  EXPECT_FALSE(B.is_symmetric());
}

TEST_F(Mat3Test, IsOrthogonal_TrueIfOrthogonal) {
  EXPECT_TRUE(m90x.is_orthogonal());
  EXPECT_TRUE(m90y.is_orthogonal());
  EXPECT_TRUE(m90z.is_orthogonal());
}

TEST_F(Mat3Test, Determinant_StoresCorrectValues) {
  EXPECT_NEAR(identity.determinant(), 1.0, 1e-9);
  EXPECT_NEAR(zero.determinant(), 0.0, 1e-9);
  EXPECT_NEAR(ones.determinant(), 0.0, 1e-9);
  EXPECT_NEAR(m90x.determinant(), 1.0, 1e-9);
  EXPECT_NEAR(A.determinant(), 0.0, 1e-9); // singular
  EXPECT_NEAR(diagonal.determinant(), 42.0, 1e-9);
  EXPECT_NEAR(upper_triangle.determinant(), 42.0, 1e-9);
  EXPECT_NEAR(lower_triangle.determinant(), 42.0, 1e-9);
  EXPECT_NEAR(magic.determinant(), -360.0, 1e-9);
}

TEST_F(Mat3Test, Property_MatrixMultiplication_Identity_ReturnsSelf) {
  EXPECT_EQ((A * identity), A);
  EXPECT_EQ((zero * identity), zero);
  EXPECT_EQ((identity * identity), identity);
}

TEST_F(Mat3Test, Property_MatrixMultiplication_Zero_ReturnZero) {
  A *= zero;
  EXPECT_EQ(A, zero);
}

TEST_F(Mat3Test, Property_MatrixMultiplication_NonCommutative) {
  EXPECT_FALSE((A * B) == (B * A));
}
