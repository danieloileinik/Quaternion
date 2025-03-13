#include "quat_class.h"

#include <gtest/gtest.h>

TEST(QuatTest, DefaultConstructor)
{
	Quat q;
	EXPECT_FLOAT_EQ(q.data()[0], 0.0f);
	EXPECT_FLOAT_EQ(q.data()[1], 0.0f);
	EXPECT_FLOAT_EQ(q.data()[2], 0.0f);
	EXPECT_FLOAT_EQ(q.data()[3], 0.0f);
}
TEST(QuatTest, Length)
{
	Quat q(1.0f, 2.0f, 3.0f, 4.0f);
	float length = static_cast< float >(q);
	EXPECT_FLOAT_EQ(length, 5.4772258f);
	Quat q1(-4.0f, -4.0f, 4.0f, 4.0f);
	float length1 = static_cast< float >(q1);
	EXPECT_FLOAT_EQ(length1, 8.0f);
}
TEST(QuatTest, ConstructorWithComponents)
{
	Quat q(1.0f, 2.0f, 3.0f, 4.0f);
	EXPECT_FLOAT_EQ(q.data()[0], 2.0f);
	EXPECT_FLOAT_EQ(q.data()[1], 3.0f);
	EXPECT_FLOAT_EQ(q.data()[2], 4.0f);
	EXPECT_FLOAT_EQ(q.data()[3], 1.0f);
}

TEST(QuatTest, RotationConstructor)
{
	Quat::vector3_t vec = { 1.0f, 0.0f, 0.0f };
	Quat::vector3_t vec1 = { -6.666f, 4.444f, -2.222f };
	Quat q(3.14159f, false, vec);
	EXPECT_FLOAT_EQ(q.data()[0], 0.027412102f);
	EXPECT_FLOAT_EQ(q.data()[1], 0.0f);
	EXPECT_FLOAT_EQ(q.data()[2], 0.0f);
	EXPECT_FLOAT_EQ(q.data()[3], 0.99962419f);
	Quat q1(3.14159f, true, vec);
	EXPECT_FLOAT_EQ(q1.data()[0], 1.0f);
	EXPECT_FLOAT_EQ(q1.data()[1], 0.0f);
	EXPECT_FLOAT_EQ(q1.data()[2], 0.0f);
	EXPECT_FLOAT_EQ(q1.data()[3], 1.232592e-06f);
	Quat q2(142, false, vec1);
	EXPECT_FLOAT_EQ(q2.data()[0], -0.75810128f);
	EXPECT_FLOAT_EQ(q2.data()[1], 0.50540084f);
	EXPECT_FLOAT_EQ(q2.data()[2], -0.25270042f);
	EXPECT_FLOAT_EQ(q2.data()[3], 0.32556859f);
}

TEST(QuatTest, VectorByScalar)
{
	Quat q1(1.0f, 2.0f, 3.0f, 4.0f);
	Quat::vector3_t result = q1.vec_by_scalar(2.0f);
	EXPECT_FLOAT_EQ(result.x, 4.0f);
	EXPECT_FLOAT_EQ(result.y, 6.0f);
	EXPECT_FLOAT_EQ(result.z, 8.0f);
}
TEST(QuatTest, VectorMultiplication)
{
	Quat q1(1.0f, 2.0f, 3.0f, 4.0f);
	Quat q2(5.0f, 6.0f, 7.0f, 8.0f);
	Quat::vector3_t result = q1.vec_mul(q2);
	EXPECT_FLOAT_EQ(result.x, -4.0f);
	EXPECT_FLOAT_EQ(result.y, 8.0f);
	EXPECT_FLOAT_EQ(result.z, -4.0f);
}

TEST(QuatTest, ScalarMultiplicationWithQuat)
{
	Quat q1(1.0f, 2.0f, 3.0f, 4.0f);
	Quat q2(5.0f, 6.0f, 7.0f, 8.0f);
	q1 *= q2;
	EXPECT_FLOAT_EQ(q1.data()[0], 12.0f);
	EXPECT_FLOAT_EQ(q1.data()[1], 30.0f);
	EXPECT_FLOAT_EQ(q1.data()[2], 24.0f);
	EXPECT_FLOAT_EQ(q1.data()[3], -60.0f);
	Quat q3;
	q3 *= q2;
	EXPECT_FLOAT_EQ(q3.data()[0], 0.0f);
	EXPECT_FLOAT_EQ(q3.data()[1], 0.0f);
	EXPECT_FLOAT_EQ(q3.data()[2], 0.0f);
	EXPECT_FLOAT_EQ(q3.data()[3], 0.0f);
}
TEST(QuatTest, QuatMultByVector)
{
	Quat q1(1.0f, 2.0f, 3.0f, 4.0f);
	Quat q2(5.0f, 6.0f, 7.0f, 8.0f);
	Quat::vector3_t vec ={5.0f,6.0f,7.0f};
	Quat::vector3_t vec1 ={0.0f,-35.0f,-6.666f};
	q1 *= vec;
	EXPECT_FLOAT_EQ(q1.data()[0], 2.0f);
	EXPECT_FLOAT_EQ(q1.data()[1], 12.0f);
	EXPECT_FLOAT_EQ(q1.data()[2], 4.0f);
	EXPECT_FLOAT_EQ(q1.data()[3], -56.0f);
	q2 *= vec1;
	EXPECT_FLOAT_EQ(q2.data()[0], 233.338f);
	EXPECT_FLOAT_EQ(q2.data()[1], -135.004f);
	EXPECT_FLOAT_EQ(q2.data()[2], -243.33f);
	EXPECT_FLOAT_EQ(q2.data()[3], 298.328f);
}

TEST(QuatTest, ScalarMultiplicationWithScalar)
{
	Quat q(1.0f, 2.0f, 3.0f, 4.0f);
	q *= 2.0f;
	EXPECT_FLOAT_EQ(q.data()[0], 4.0f);
	EXPECT_FLOAT_EQ(q.data()[1], 6.0f);
	EXPECT_FLOAT_EQ(q.data()[2], 8.0f);
	EXPECT_FLOAT_EQ(q.data()[3], 2.0f);
	Quat q1(-17.0f, 22.0f, 31.0f, 40.0f);
	q1 *= 0.0f;
	EXPECT_FLOAT_EQ(q1.data()[0], 0.0f);
	EXPECT_FLOAT_EQ(q1.data()[1], 0.0f);
	EXPECT_FLOAT_EQ(q1.data()[2], 0.0f);
	EXPECT_FLOAT_EQ(q1.data()[3], 0.0f);
	Quat q2(1.0f, 2.0f, 3.0f, 4.0f);
	q2 *= -7.777f;
	EXPECT_FLOAT_EQ(q2.data()[0], -15.554f);
	EXPECT_FLOAT_EQ(q2.data()[1], -23.330999f);
	EXPECT_FLOAT_EQ(q2.data()[2], -31.108f);
	EXPECT_FLOAT_EQ(q2.data()[3], -7.777);
}

TEST(QuatTest, Addition)
{
	Quat q1(1.0f, 2.0f, 3.0f, 4.0f);
	Quat q2(5.0f, 6.0f, 7.0f, 8.0f);
	q1 += q2;
	EXPECT_FLOAT_EQ(q1.data()[0], 8.0f);
	EXPECT_FLOAT_EQ(q1.data()[1], 10.0f);
	EXPECT_FLOAT_EQ(q1.data()[2], 12.0f);
	EXPECT_FLOAT_EQ(q1.data()[3], 6.0f);


}
TEST(QuatTest, Addition1)
{
	Quat q4(-1.0f, 555.555f, 333.333f, 444.444f);
	Quat q5(777.777f, 777.777f, 777.777f, 777.777f);
	Quat q3;
	q3 = q4 + q5;
	EXPECT_FLOAT_EQ(q3.data()[0], 1333.332f);
	EXPECT_FLOAT_EQ(q3.data()[1], 1111.11f);
	EXPECT_FLOAT_EQ(q3.data()[2], 1222.2209f);
	EXPECT_FLOAT_EQ(q3.data()[3], 776.77698f);

}

TEST(QuatTest, Subtraction)
{
	Quat q1(5.0f, 6.0f, 7.0f, 8.0f);
	Quat q2(1.0f, 2.0f, 3.0f, 4.0f);
	q1 -= q2;
	EXPECT_FLOAT_EQ(q1.data()[0], 4.0f);
	EXPECT_FLOAT_EQ(q1.data()[1], 4.0f);
	EXPECT_FLOAT_EQ(q1.data()[2], 4.0f);
	EXPECT_FLOAT_EQ(q1.data()[3], 4.0f);

}
TEST(QuatTest, Substraction1)
{
	Quat q4(-1.0f, 555.555f, 333.333f, 444.444f);
	Quat q5(777.777f, 777.777f, 777.777f, 777.777f);
	Quat q3;
	q3 = q4 - q5;
	EXPECT_FLOAT_EQ(q3.data()[0], -222.22198f);
	EXPECT_FLOAT_EQ(q3.data()[1], -444.44397f);
	EXPECT_FLOAT_EQ(q3.data()[2], -333.33298f);
	EXPECT_FLOAT_EQ(q3.data()[3], -778.77698f);

}
TEST(QuatTest, Equality)
{
	Quat q1(1.0f, 2.0f, 3.0f, 4.0f);
	Quat q2(1.0f, 2.0f, 3.0f, 4.0f);
	EXPECT_TRUE(q1 == q2);
}

TEST(QuatTest, Inequality)
{
	Quat q1(1.0f, 2.0f, 3.0f, 4.0f);
	Quat q2(5.0f, 6.0f, 7.0f, 8.0f);
	EXPECT_TRUE(q1 != q2);
}

TEST(QuatTest, Conjugate)
{
	Quat q(1.0f, 2.0f, 3.0f, 4.0f);
	Quat conjugate = ~q;
	EXPECT_FLOAT_EQ(conjugate.data()[0], -2.0f);
	EXPECT_FLOAT_EQ(conjugate.data()[1], -3.0f);
	EXPECT_FLOAT_EQ(conjugate.data()[2], -4.0f);
	EXPECT_FLOAT_EQ(conjugate.data()[3], 1.0f);
}

TEST(QuatTest, RotationMatrix)
{
	Quat::vector3_t vec = { 1.0f, 0.0f, 0.0f };
	Quat q(3.14159f, false, vec);
	float** matrix = q.rotation_matrix();
	EXPECT_FLOAT_EQ(matrix[0][0], 1.0f);
	EXPECT_FLOAT_EQ(matrix[1][1], 0.99849713f);
	EXPECT_FLOAT_EQ(matrix[2][2], 0.99849713f);
	EXPECT_FLOAT_EQ(matrix[3][3], 1.0f);

	Quat::free_matrix(matrix);

	Quat::vector3_t vec1 = { -6.666f, 3.333f, -9.999f };
	Quat q1(2.28f, true, vec1);
	matrix = q1.rotation_matrix();
	EXPECT_NEAR(matrix[0][1], 0.37256825f,0.001f);
	EXPECT_NEAR(matrix[1][2], 0.051803917f,0.001f);
	EXPECT_FLOAT_EQ(matrix[2][3], 0.0f);
	EXPECT_FLOAT_EQ(matrix[3][3], 1.0f);

	Quat::free_matrix(matrix);
	Quat::vector3_t vec2 = { 0.0, 11.11, -3.5f };
	Quat q2(88.0f, false, vec2);
	matrix = q2.rotation_matrix();
	EXPECT_FLOAT_EQ(matrix[0][0], 0.034899957f);
	EXPECT_FLOAT_EQ(matrix[0][1], 0.30029085f);
	EXPECT_FLOAT_EQ(matrix[1][2], -0.27658701f);
	EXPECT_FLOAT_EQ(matrix[2][3], 0.0f);
	EXPECT_FLOAT_EQ(matrix[3][3], 1.0f);

	Quat::free_matrix(matrix);
	Quat q3(2.0f, 3.0f,4.0f,5.0f);
	matrix = q3.rotation_matrix();
	EXPECT_NEAR(matrix[0][1], 0.07407406f,0.001f);
	EXPECT_FLOAT_EQ(matrix[1][2], 0.51851857f);

	Quat::free_matrix(matrix);

}
TEST(QuatTest, Matrix)
{
	Quat q(1.0f, 2.0f, 3.0f, 4.0f);
	float** matrix = q.matrix();
	EXPECT_FLOAT_EQ(matrix[0][0], 1.0f);
	EXPECT_FLOAT_EQ(matrix[1][0], 2.0f);
	EXPECT_FLOAT_EQ(matrix[2][0], 3.0f);
	EXPECT_FLOAT_EQ(matrix[3][0], 4.0f);
	EXPECT_FLOAT_EQ(matrix[1][1], 1.0f);
	EXPECT_FLOAT_EQ(matrix[2][2], 1.0f);
	EXPECT_FLOAT_EQ(matrix[3][3], 1.0f);

	Quat::free_matrix(matrix);
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
