#pragma once
#include <iostream>

#define max(a,b)            (((a) > (b)) ? (a) : (b))
#define min(a,b)            (((a) < (b)) ? (a) : (b))
struct Coordinate
{
	short x, y;
	Coordinate()
	{
	}
	Coordinate(short _x, short _y) : x(_x), y(_y)
	{
	}
	float dist(const Coordinate& rhs)
	{
		int absX = abs(x - rhs.x);
		int absY = abs(y - rhs.y);

		int diagDist = min(absX, absY);
		int straightDist = max(absX, absY) - diagDist;
		return diagDist*1.414213562373095f + straightDist;
	}
	float distSqrt(const Coordinate& rhs)
	{
		int absX = abs(x - rhs.x);
		int absY = abs(y - rhs.y);

		int diagDist = min(absX, absY);
		int straightDist = max(absX, absY) - diagDist;
		return diagDist*1.414213562373095f + straightDist;
	}
	void add(const Coordinate& rhs)
	{
		x += rhs.x;
		y += rhs.y;
	}
	Coordinate operator + (Coordinate & rhs)
	{
		return Coordinate(x + rhs.x, y + rhs.y);
	}
	Coordinate operator += (Coordinate & rhs)
	{
		x += rhs.x;
		y += rhs.y;
		return *this;
	}
	Coordinate operator -= (Coordinate & rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		return *this;
	}
	Coordinate operator * (int & rhs)
	{
		return Coordinate(x *rhs, y *rhs);
	}
};
typedef Coordinate xyLoc;