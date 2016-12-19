#pragma once

namespace CG_MATH
{
	class vector3;
	class EulerAngles;
	class Quaternion;

	//-------------------------------------------------------------------------------
	// RoattionMatrix 类
	//实现了一个简单的3x3 矩阵， 仅用作旋转
	//矩阵假设为正交的， 在变换时指定方向


	class RotationMatrix
	{
	public:
		
		// 公共数据

		//矩阵得9个值

		float m11, m12, m13;
		float m21, m22, m23;
		float m31, m32, m33;

		//公共操作

		//置为单位矩阵

		void identity();

		//根据指定的方位构造矩阵

		void setup(const EulerAngles &orientation);

		// 根据四元数构造矩阵，假设该四元数参数代表指定方向的变换

		void fromInertialToObjectQuaternion(const Quaternion &q);
		void fromObjectToInertialQuaternion(const Quaternion &q);

		// 执行旋转

		vector3 inertialToObject(const vector3 &v) const;
		vector3 objectToInertial(const vector3 &v) const;
	};

}

