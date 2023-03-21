/******************************************************************************/
/*!
\file		Vector2D.cpp
\author 	Ian Chua
\par    	email: i.chua@digipen.edu
\date   	Feb 12, 2023
\brief

Copyright (C) 2023 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the
prior written consent of DigiPen Institute of Technology is prohibited.
 */
 /******************************************************************************/

#include "Vector2D.h"
#include <math.h>

namespace CSD1130
{
	// constructor
	Vector2D::Vector2D(float _x, float _y) :x(_x), y(_y) {}

	Vector2D& Vector2D::operator+=(const Vector2D& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		return *this;
	}

	Vector2D& Vector2D::operator-=(const Vector2D& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		return *this;
	}

	Vector2D& Vector2D::operator*=(float rhs)
	{
		x *= rhs;
		y *= rhs;
		return *this;
	}

	Vector2D& Vector2D::operator/=(float rhs)
	{
		x /= rhs;
		y /= rhs;
		return *this;
	}

	Vector2D Vector2D::operator-() const
	{
		Vector2D out{ -x, -y };
		return out;
	}

	Vector2D operator+(const Vector2D& lhs, const Vector2D& rhs)
	{
		Vector2D out{ lhs.x + rhs.x, lhs.y + rhs.y };
		return out;
	}

	Vector2D operator-(const Vector2D& lhs, const Vector2D& rhs)
	{
		Vector2D out{ lhs.x - rhs.x, lhs.y - rhs.y };
		return out;
	}

	Vector2D operator*(const Vector2D& lhs, float rhs)
	{
		Vector2D out{ lhs.x * rhs, lhs.y * rhs };
		return out;
	}

	Vector2D operator*(float rhs, const Vector2D& lhs)
	{
		Vector2D out{ lhs.x * rhs, lhs.y * rhs };
		return out;
	}

	Vector2D operator/(const Vector2D& lhs, float rhs)
	{
		Vector2D out{ lhs.x / rhs, lhs.y / rhs };
		return out;
	}

	// functions
	void Vector2DNormalize(Vector2D& pResult, const Vector2D& pVec0)
	{
		// divide x and y by length of vector to get 1 unit vector
		float len = Vector2DLength(pVec0);
		pResult.x = pVec0.x / len;
		pResult.y = pVec0.y / len;
	}

	float Vector2DLength(const Vector2D& pVec0)
	{
		// square root to get actual length
		float out = sqrtf(Vector2DSquareLength(pVec0));
		return out;
	}

	float Vector2DSquareLength(const Vector2D& pVec0)
	{
		// get squared length using pythagoras theorem
		float out = powf(pVec0.x, 2.f) + powf(pVec0.y, 2.f);
		return out;
	}

	float Vector2DDistance(const Vector2D& pVec0, const Vector2D& pVec1)
	{
		// square root to get actual distance
		float out = sqrtf(Vector2DSquareDistance(pVec0, pVec1));
		return out;
	}

	float Vector2DSquareDistance(const Vector2D& pVec0, const Vector2D& pVec1)
	{
		// get squared distance using pythagoras theorem
		float out = powf((pVec1.x - pVec0.x), 2.f) + powf((pVec1.y - pVec0.y), 2.f);
		return out;
	}

	float Vector2DDotProduct(const Vector2D& pVec0, const Vector2D& pVec1)
	{
		// x1 * x2 + y1 * y2
		float out = pVec0.x * pVec1.x + pVec0.y * pVec1.y;
		return out;
	}

	float Vector2DCrossProductMag(const Vector2D& pVec0, const Vector2D& pVec1)
	{
		// x1 * y2 - y1 * x2
		float out = pVec0.x * pVec1.y - pVec0.y * pVec1.x;
		return out;
	}
}