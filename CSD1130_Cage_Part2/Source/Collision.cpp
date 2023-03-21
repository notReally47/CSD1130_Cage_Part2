/******************************************************************************/
/*!
\file		Collision.cpp
\author 	DigiPen
\par    	email: digipen\@digipen.edu
\date   	January 01, 20xx
\brief

Copyright (C) 20xx DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the
prior written consent of DigiPen Institute of Technology is prohibited.
 */
/******************************************************************************/

#include "main.h"

/******************************************************************************/
/*!
 */
/******************************************************************************/
void BuildLineSegment(LineSegment &lineSegment,
					const CSD1130::Vec2& p0,
					const CSD1130::Vec2& p1)
{
	lineSegment.m_pt0 = p0;
	lineSegment.m_pt1 = p1;

	CSD1130::Vec2 line{ p1 - p0 };					// Create line => p1 - p0
	lineSegment.m_normal = { line.y, -line.x };		// Calculate normal => y, -x

	CSD1130::Vector2DNormalize(lineSegment.m_normal, lineSegment.m_normal); // Normalize normal
}

/******************************************************************************/
/*!
 */
/******************************************************************************/
int CollisionIntersection_CircleLineSegment(const Circle &circle,
											const CSD1130::Vec2 &ptEnd,
											const LineSegment &lineSeg,
											CSD1130::Vec2 &interPt,
											CSD1130::Vec2 &normalAtCollision,
											float &interTime,
											bool & checkLineEdges)
{
	float NBs = CSD1130::Vector2DDotProduct(circle.m_center, lineSeg.m_normal);
	float NP0 = CSD1130::Vector2DDotProduct(lineSeg.m_pt0, lineSeg.m_normal);

	CSD1130::Vec2 BsP0{};
	CSD1130::Vec2 BsP1{};

	CSD1130::Vec2 V = ptEnd - circle.m_center;			// Velocity vector
	CSD1130::Vec2 M = { V.y, -V.x };					// Normal to velocity vector

	CSD1130::Vector2DNormalize(M, M);					// normalize velocity vector's normal
	
	if (NBs - NP0 <= -circle.m_radius)
	{
		BsP0 = (lineSeg.m_pt0 - circle.m_radius * lineSeg.m_normal) - circle.m_center;	// Shortest distance from point to imaginary (P0') line segment
		BsP1 = (lineSeg.m_pt1 - circle.m_radius * lineSeg.m_normal) - circle.m_center;	// Shortest distance from point to imaginary (P1') line segment

		if (CSD1130::Vector2DDotProduct(M, BsP0) * CSD1130::Vector2DDotProduct(M, BsP1) < 0.f)
		{
			interTime = (CSD1130::Vector2DDotProduct(lineSeg.m_normal, lineSeg.m_pt0) - CSD1130::Vector2DDotProduct(lineSeg.m_normal, circle.m_center) - circle.m_radius) / CSD1130::Vector2DDotProduct(lineSeg.m_normal, V);
			if (0.f <= interTime  && interTime <= 1.f)
			{
				interPt				= circle.m_center + V * interTime;
				normalAtCollision	= -lineSeg.m_normal;
				return 1;
			}
			else return CheckMovingCircleToLineEdge(false, circle, ptEnd, lineSeg, interPt, normalAtCollision, interTime);
		}

	}
	else if (NBs - NP0 >= circle.m_radius)
	{
		BsP0 = (lineSeg.m_pt0 + circle.m_radius * lineSeg.m_normal) - circle.m_center;	// Shortest distance from point to imaginary (P0') line segment
		BsP1 = (lineSeg.m_pt1 + circle.m_radius * lineSeg.m_normal) - circle.m_center;	// Shortest distance from point to imaginary (P1') line segment

		if (CSD1130::Vector2DDotProduct(M, BsP0) * CSD1130::Vector2DDotProduct(M, BsP1) < 0.f)
		{
			interTime = (CSD1130::Vector2DDotProduct(lineSeg.m_normal, lineSeg.m_pt0) - CSD1130::Vector2DDotProduct(lineSeg.m_normal, circle.m_center) + circle.m_radius) / CSD1130::Vector2DDotProduct(lineSeg.m_normal, V);
			if (0.f <= interTime && interTime <= 1.f)
			{
				interPt				= circle.m_center + V * interTime;
				normalAtCollision	= lineSeg.m_normal;
				return 1;
			}
			else return CheckMovingCircleToLineEdge(false, circle, ptEnd, lineSeg, interPt, normalAtCollision, interTime);
		}
	}
	else return CheckMovingCircleToLineEdge(true, circle, ptEnd, lineSeg, interPt, normalAtCollision, interTime);

	return 0;
}

/******************************************************************************/
/*
* \param withinBothLines:	
* \param circle:			R, Bs
* \param ptEnd:				Be
* \param lineSeg:			n, p0, p1
* \param interPt:			Bi
* \param normalAtCollision:	
* \param interTime:			ti
*/
/******************************************************************************/
int CheckMovingCircleToLineEdge(bool withinBothLines,
								const Circle &circle,
								const CSD1130::Vec2 &ptEnd,
								const LineSegment &lineSeg,
								CSD1130::Vec2 &interPt,
								CSD1130::Vec2 &normalAtCollision,
								float &interTime)
{
	float dist0{}, dist1{}, s{}, m{};
	CSD1130::Vec2 BsP0	= lineSeg.m_pt0 - circle.m_center;	// Shortest distance from point to line segment
	CSD1130::Vec2 BsP1	= lineSeg.m_pt1 - circle.m_center;

	CSD1130::Vec2 P0P1	= lineSeg.m_pt1 - lineSeg.m_pt0;	// Line vector
	CSD1130::Vec2 V		= ptEnd - circle.m_center;			// Velocity vector
	CSD1130::Vec2 M		= { V.y, -V.x };					// Normal to velocity vector

	CSD1130::Vector2DNormalize(M, M);						// normalize velocity vector's normal

	if (withinBothLines)
	{
		if (CSD1130::Vector2DDotProduct(BsP0, P0P1) > 0.f) // BsP0.V > 0
		{
			m = CSD1130::Vector2DDotProduct(BsP0, V);
			if (m > 0.f)
			{
				dist0 = CSD1130::Vector2DDotProduct(BsP0, M);

				if (abs(dist0) > circle.m_radius)
					return 0; // no collision, edge of circle not touching line edge

				s			= sqrt(circle.m_radius * circle.m_radius - dist0 * dist0);
				interTime	= (m - s) / CSD1130::Vector2DLength(V);

				if (interTime <= 1.f)
				{
					interPt				= circle.m_center + V * interTime; // Bi = Bs + V *ti
					normalAtCollision	= interPt - lineSeg.m_pt0;
					CSD1130::Vector2DNormalize(normalAtCollision, normalAtCollision);
					return 1;
				}
			}
		} // end if (BsP0.V > 0)

		else // (BsP1.P0P1 < 0)
		{
			m = CSD1130::Vector2DDotProduct(BsP1, V);
			if (m > 0.f)
			{
				dist1 = CSD1130::Vector2DDotProduct(BsP1, M);

				if (abs(dist1) > circle.m_radius)
					return 0;

				s			= sqrt(circle.m_radius * circle.m_radius - dist1 * dist1);
				interTime	= (m - s) / CSD1130::Vector2DLength(V);

				if (interTime <= 1.f)
				{
					interPt				= circle.m_center + V * interTime; // Bi = Bs + V *ti
					normalAtCollision	= interPt - lineSeg.m_pt1;
					CSD1130::Vector2DNormalize(normalAtCollision, normalAtCollision);
					return 1;
				}
			}
		} // end else (BsP1.P0P1 < 0)
	} // end if (withinBothLines)

	else // !withinBothLines
	{
		bool P0Side = false;
		dist0 = CSD1130::Vector2DDotProduct(BsP0, M);
		dist1 = CSD1130::Vector2DDotProduct(BsP1, M);

		float dist0Abs = abs(dist0);
		float dist1Abs = abs(dist1);

		if (dist0Abs > circle.m_radius && dist1Abs > circle.m_radius)
			return 0;

		else if (dist0Abs <= circle.m_radius && dist1Abs <= circle.m_radius)
		{
			float m0Abs = abs(CSD1130::Vector2DDotProduct(BsP0, V));
			float m1Abs = abs(CSD1130::Vector2DDotProduct(BsP1, V));

			P0Side = m0Abs < m1Abs ? true : false;
		}

		else if (dist0Abs <= circle.m_radius)
			P0Side = true;

		else P0Side = false;

		if (P0Side)
		{
			m = CSD1130::Vector2DDotProduct(BsP0, V);
			if (m < 0.f)
				return 0;
			else
			{
				s			= sqrt(circle.m_radius * circle.m_radius - dist0 * dist0);
				interTime	= (m - s) / CSD1130::Vector2DLength(V);
				if (interTime <= 1.f)
				{
					interPt				= circle.m_center + V * interTime; // Bi = Bs + V *ti
					normalAtCollision	= interPt - lineSeg.m_pt0;
					CSD1130::Vector2DNormalize(normalAtCollision, normalAtCollision);
					return 1;
				}
			}
		} // end if P0Side

		else
		{
			m = CSD1130::Vector2DDotProduct(BsP1, V);
			if (m < 0.f)
				return 0;
			else
			{
				s			= sqrt(circle.m_radius * circle.m_radius - dist1 * dist1);
				interTime	= (m - s) / CSD1130::Vector2DLength(V);
				if (interTime <= 1.f)
				{
					interPt				= circle.m_center + V * interTime; // Bi = Bs + V *ti
					normalAtCollision	= interPt - lineSeg.m_pt1;
					CSD1130::Vector2DNormalize(normalAtCollision, normalAtCollision);
					return 1;
				}
			}
		} // end else !P0Side

	} // end !withinBothLines

	return 0;//no collision

} // end CheckMovingCircleToLineEdge





/******************************************************************************/
/*!
 */
/******************************************************************************/
void CollisionResponse_CircleLineSegment(const CSD1130::Vec2 &ptInter,
										const CSD1130::Vec2 &normal,
										CSD1130::Vec2 &ptEnd,
										CSD1130::Vec2 &reflected)
{
	CSD1130::Vec2 penetration	= ptEnd - ptInter;																// Penetration vector (BiBe)
	reflected					= penetration - 2 * CSD1130::Vector2DDotProduct(penetration, normal) * normal;	// Reflection vector
	ptEnd						= ptInter + reflected;															// Point after reflection

	CSD1130::Vector2DNormalize(reflected, reflected);
}
