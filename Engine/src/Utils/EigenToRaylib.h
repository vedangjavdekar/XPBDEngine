#pragma once
#include "raylib.h"
#include "Eigen/Dense"

namespace Utils::Math
{
	template<typename T>
	Vector3 ToVector3(const T& MatrixLike)
	{
		return Vector3{ MatrixLike(0), MatrixLike(1), MatrixLike(2) };
	}

	template<typename T>
	Matrix ToMatrix(const T& MatrixLike)
	{
		Matrix m;
		m.m0 = MatrixLike(0);
		m.m1 = MatrixLike(1);
		m.m2 = MatrixLike(2);
		m.m3 = MatrixLike(3);

		m.m4 = MatrixLike(4);
		m.m5 = MatrixLike(5);
		m.m6 = MatrixLike(6);
		m.m7 = MatrixLike(7);

		m.m8 = MatrixLike(8);
		m.m9 = MatrixLike(9);
		m.m10 = MatrixLike(10);
		m.m11 = MatrixLike(11);

		m.m12 = MatrixLike(12);
		m.m13 = MatrixLike(13);
		m.m14 = MatrixLike(14);
		m.m15 = MatrixLike(15);

		return m;
	}
}