//////////////////////////////////////////////////////////////////////////
// Vector3:  a simple 3d vector class
// from “3D数学基础：图形与游戏开发”
//////////////////////////////////////////////////////////////////////////

#ifndef VECTOR3_H
#define VECTOR3_H

#include <cmath>
#include <cassert>

class Vector3
{
public:
	float x,y,z;
	
	//Constructor
	Vector3() {}
	Vector3(const Vector3 & rhs):x(rhs.x), y(rhs.y), z(rhs.z) {}
	Vector3(float ax, float ay, float az): x(ax), y(ay), z(az) {}
	
	//Operations
	Vector3 & operator =(const Vector3 &rhs)
	{
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		return *this;
	}

	// operator == and !=
	bool operator ==(const Vector3 &rhs)
	{
		return (x==rhs.x && y==rhs.y && z==rhs.z);
	}

	bool operator !=(const Vector3 &rhs)
	{
		return (x!=rhs.x || y!=rhs.y || z!=rhs.z);
	}

	// set x,y,z to 0.0f
	void zero()
	{
		x = y = z = 0.0f;
	}

	// operator -
	Vector3 operator -() const
	{
		return Vector3(-x, -y, -z);
	}

	// operator + and -(Vector3 &)
	Vector3 operator +(const Vector3 &rhs) const
	{
		return Vector3(x+rhs.x, y+rhs.y, z+rhs.z);
	}

	Vector3 operator -(const Vector3 &rhs) const
	{
		return Vector3(x-rhs.x, y-rhs.y, z-rhs.z);
	}

	// operator * and / with a scalar
	Vector3 operator *(float s) const
	{
		return Vector3(x*s, y*s, z*s);
	}

	Vector3 operator /(float s) const
	{
		float oneOverS = 1.0f /s; // no div by 0 check here
		return Vector3(x*oneOverS, y*oneOverS, z*oneOverS);
	}

	// +=,  -=, *=  /=
	Vector3 & operator +=(const Vector3 & rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		return *this;
	}

	Vector3 & operator -=(const Vector3 & rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		return *this;
	}

	Vector3 & operator *=(float s)
	{
		x *= s;
		y *= s;
		z *= s;
		return *this;
	}

	Vector3 & operator /=(float s)
	{
		float oneOverS = 1.0f / s;  // no div by 0 check here
		x *= oneOverS;
		y *= oneOverS;
		z *= oneOverS;
		return *this;
	}

	// normalize vec
	float normalize()
	{
		float mag = x*x + y*y + z*z;
		if (mag >0.0f)
		{
			mag = sqrt(mag);
			float oneOverMag = 1.0f / mag;
			x *= oneOverMag;
			y *= oneOverMag;
			z *= oneOverMag;
		}
		return mag;
	}

	float length2() const
	{
		return x*x + y*y + z*z;
	}

	float length() const
	{
		return sqrt(length2());
	}

	// dot product
	float operator *(const Vector3 & rhs) const
	{
		return (x*rhs.x + y*rhs.y + z*rhs.z);
	}

}; // end of class Vector3

// non member func
inline float vectorMag(const Vector3 &vec)
{
	return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

inline Vector3 crossProduct(const Vector3 &lhs, const Vector3 &rhs)
{
	return Vector3( lhs.y*rhs.z - lhs.z*rhs.y,   lhs.z*rhs.x - lhs.x*rhs.z,   lhs.x*rhs.y - lhs.y*rhs.x);
}

inline Vector3 operator / (const Vector3 &lhs, const Vector3 & rhs)
{
	assert(rhs.x * rhs.y * rhs.z != 0.0f );
	return Vector3(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z);
}

// left multiply by a scalar
inline Vector3 operator * (float s, const Vector3 &vec)
{
	return Vector3(s*vec.x, s*vec.y, s*vec.z);
}

// dis between two points
inline float distance(const Vector3 &lhs, const Vector3 & rhs)
{
	float dx = lhs.x - rhs.x;
	float dy = lhs.y - rhs.y;
	float dz = lhs.z - rhs.z;
	return sqrt(dx*dx +dy*dy +dz*dz);
}

// square dis between two points
inline float squareDist(const Vector3 &lhs, const Vector3 &rhs)
{
	float dx = lhs.x - rhs.x;
	float dy = lhs.y - rhs.y;
	float dz = lhs.z - rhs.z;
	return (dx*dx +dy*dy +dz*dz);
}

// global var - a zero 3d vector
extern const Vector3 kZeroVector3;

#endif