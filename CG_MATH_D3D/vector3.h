#pragma once
#include<math.h>
namespace CG_MATH
{

	class vector3
	{
	public:
		float x, y, z;
	// 构造函数；
		vector3() = delete;
		vector3(float nx, float ny, float nz) :x(nx), y(ny), z(nz) {}
		vector3(const vector3& a) :x(a.x), y(a.y), z(a.z) {}
	// 标准对象操作
		vector3& operator=(const vector3 a)
		{
			x = a.x;
			y = a.y;
			z = a.z;
			return *this;
		}
		bool operator==(const vector3 a) const
		{
			return x == a.x && y == a.y && z == a.z;
		}
		bool operator!=(const vector3 a) const
		{
			return x != a.x || y != a.y || z != a.z;
		}

	// 向量运算
		void zero() { x = y = z = 0.0f; }
		// 取反
		vector3 operator-() const { return vector3(-x, -y, -z); }
		vector3 operator+(const vector3& rhs) const
		{
			return vector3(x + rhs.x, y + rhs.y, z + rhs.z);
		}
		vector3 operator-(const vector3& rhs) const
		{
			return vector3(x - rhs.x, y - rhs.y, z - rhs.z);
		}
		// 与标量的乘除法
		vector3 operator*(float a) const
		{
			return vector3(x*a, y*a, z*a);
		}
		vector3 operator/(float a) const
		{
			// 这里不进行除0检查、
			float oneOveraA = 1.0f / a;
			return vector3(x *oneOveraA, y * oneOveraA, z * oneOveraA);
		}
		vector3& operator+=(const vector3& rhs)
		{
			x += rhs.x;
			y += rhs.y;
			z += rhs.z;
			return *this;
		}
		vector3& operator-=(const vector3& rhs)
		{
			x -= rhs.x;
			y -= rhs.y;
			z -= rhs.z;
			return *this;
		}
		vector3& operator*= (float a)
		{
			x *= a;
			y *= a;
			z *= a;
			return *this;
		}
		vector3& operator/= (float a)
		{
			float oneOveraA = 1.0f / a;
			x *= oneOveraA;
			y *= oneOveraA;
			z *= oneOveraA;
			return *this;
		}


		float operator*(const vector3& rhs) const
		{
			return x*rhs.x + y*rhs.y + z*rhs.z;
		}
		
		void normalize()
		{
			float magSq = x*x + y*y + z*z;
			if (magSq > 0.0f)
			{
				float oneOverMag = 1.0f / sqrtf(magSq);
				x *= oneOverMag;
				y *= oneOverMag;
				z *= oneOverMag;
			}
		}

	};

	inline float vectorMag(const vector3 &a)
	{
		return sqrtf(a.x*a.x + a.y*a.y + a.z *a.z);
	}
	inline vector3 cross(const vector3& lhs, const vector3& rhs)
	{
		return vector3(
			lhs.y*rhs.z - lhs.z*rhs.y,
			lhs.z*rhs.x - lhs.x*rhs.z,
			lhs.x*rhs.y - lhs.y*rhs.x);
	}
	// 实现标量左乘
	inline vector3 operator * (float k, const vector3 &v)
	{
		return vector3(k*v.x, k*v.y, k*v.z);
	}

	inline float distance(const vector3 &a, const vector3 &b)
	{
		float dx = a.x - b.x;
		float dy = a.y - b.y;
		float dz = a.z - b.z;
		return sqrtf(dx*dx + dy *dy + dz*dz);
	}

	// 全局变量， 全局零向量。
	extern const vector3 kZeroVector;

}
