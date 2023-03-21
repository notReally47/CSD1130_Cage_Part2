/******************************************************************************/
/*!
\file		Matrix3x3.cpp
\author 	Ian Chua
\par    	email: i.chua@digipen.edu
\date   	Feb 12, 2023
\brief

Copyright (C) 2023 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the
prior written consent of DigiPen Institute of Technology is prohibited.
 */
 /******************************************************************************/

#include "Matrix3x3.h"
#include <math.h>
constexpr float PI = 3.14159265358f;

namespace CSD1130
{
	// constructors
	Matrix3x3::Matrix3x3(const float* pArr)
	{
		// initialize
		*this = Matrix3x3();

		// set copy parameter to this
		for (size_t i = 0; i < 9; i++)
		{
			float* curr = this->m + i;
			*curr = *(pArr + i);
		}
	}

	Matrix3x3::Matrix3x3(float _00, float _01, float _02, float _10, float _11, float _12, float _20, float _21, float _22)
	{
		// set parameters to corresponding attributes
		m00 = _00;
		m01 = _01;
		m02 = _02;

		m10 = _10;
		m11 = _11;
		m12 = _12;

		m20 = _20;
		m21 = _21;
		m22 = _22;
	}

	// operator overloads
	Matrix3x3& Matrix3x3::operator=(const Matrix3x3& rhs)
	{
		// copy values 
		for (size_t i = 0; i < 9; i++)
		{
			this->m[i] = rhs.m[i];
		}
		return *this;
	}

	Matrix3x3& Matrix3x3::operator*=(const Matrix3x3& rhs)
	{
		// use * operator overload
		*this = *this * rhs;
		return *this;
	}


	Matrix3x3 operator*(const Matrix3x3& lhs, const Matrix3x3& rhs)
	{
		// initialize output matrix
		Matrix3x3 out = Matrix3x3();


		// multiply 
		for (unsigned int i = 0; i < 3; ++i)
		{
			for (unsigned int j = 0; j < 3; ++j)
			{
				// calculate each value in the matrix 
				float sum = 0;

				// [row][col] x [col][row] for square matrix multiplication
				for (unsigned int k = 0; k < 3; ++k)
					sum += lhs.m2[i][k] * rhs.m2[k][j];

				// set output matrix
				out.m2[i][j] = sum;
			}
		}

		return out;
	}

	Vector2D operator*(const Matrix3x3& pMtx, const Vector2D& rhs)
	{
		Vector2D out = rhs;

		// 3x3 matrix * 2x1 matrix
		out.x = pMtx.m00 * rhs.x + pMtx.m01 * rhs.y + pMtx.m02;
		out.y = pMtx.m10 * rhs.x + pMtx.m11 * rhs.y + pMtx.m12;
		return out;
	}

	// functions
	void Mtx33Identity(Matrix3x3& pResult)
	{
		// empty matrix
		pResult = Matrix3x3();

		// set diagonals to 1.f
		pResult.m00 = 1.f;
		pResult.m11 = 1.f;
		pResult.m22 = 1.f;
	}

	void Mtx33Translate(Matrix3x3& pResult, float x, float y)
	{
		// set identity matrix
		Mtx33Identity(pResult);

		// add translation
		pResult.m02 += x;
		pResult.m12 += y;
	}

	void Mtx33Scale(Matrix3x3& pResult, float x, float y)
	{
		// set identity matrix
		Mtx33Identity(pResult);

		// set scale x and y
		pResult.m00 = x;
		pResult.m11 = y;
	}

	void Mtx33RotRad(Matrix3x3& pResult, float angle)
	{
		// create identity matrix
		Mtx33Identity(pResult);

		// set rotation matrix
		pResult.m00 = cosf(angle);
		pResult.m01 = -sinf(angle);
		pResult.m10 = sinf(angle);
		pResult.m11 = cosf(angle);
	}

	void Mtx33RotDeg(Matrix3x3& pResult, float angle)
	{
		// convert deg to rad
		angle *= PI / 180.f;

		// rotate using rad
		Mtx33RotRad(pResult, angle);
	}

	void Mtx33Transpose(Matrix3x3& pResult, const Matrix3x3& pMtx)
	{
		// invert the rows and columns
		for (size_t i = 0; i < 3; i++)
		{
			for (size_t j = 0; j < 3; j++)
			{
				pResult.m2[i][j] = pMtx.m2[j][i];
			}
		}
	}

	void Mtx33Inverse(Matrix3x3* pResult, float* determinant, const Matrix3x3& pMtx)
	{
		// calculate determinant 
		*determinant =	(pMtx.m00 * pMtx.m11 * pMtx.m22 +
						pMtx.m01 * pMtx.m12 * pMtx.m20 +
						pMtx.m10 * pMtx.m21 * pMtx.m02) -
						(pMtx.m02 * pMtx.m11 * pMtx.m20 +
						pMtx.m10 * pMtx.m01 * pMtx.m22 +
						pMtx.m12 * pMtx.m21 * pMtx.m00);

		// if inversion fails
		if (*determinant == 0.f) {
			pResult = nullptr;
			*determinant = 0.f;
			return;
		}

		// create adjoint matrix
		Matrix3x3 adjoint = 
		{
			(pMtx.m11 * pMtx.m22 - pMtx.m12 * pMtx.m21),	-(pMtx.m01 * pMtx.m22 - pMtx.m02 * pMtx.m21),	(pMtx.m01 * pMtx.m12 - pMtx.m02 * pMtx.m11),
			-(pMtx.m10 * pMtx.m22 - pMtx.m12 * pMtx.m20),	(pMtx.m00 * pMtx.m22 - pMtx.m02 * pMtx.m20),	-(pMtx.m00 * pMtx.m12 - pMtx.m02 * pMtx.m10),
			(pMtx.m10 * pMtx.m21 - pMtx.m11 * pMtx.m20),	-(pMtx.m00 * pMtx.m21 - pMtx.m01 * pMtx.m20),	(pMtx.m00 * pMtx.m11 - pMtx.m01 * pMtx.m10)
		};

		// set inverse matrix
		for (size_t i = 0; i < 9; i++)
		{
			pResult->m[i] = adjoint.m[i] / *determinant;
		}
	}
}