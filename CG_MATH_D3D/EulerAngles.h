#pragma once
namespace CG_MATH
{
	class Quaternion;
	class Matrix4x3;
	class RotationMatrix;

	//--------------------------------------------------------
	// class EulerAngles
	//
	// 该类用于表示heading-pitch-bank 欧拉角系统
	//heading +y, pitch +x, bank +z

	class EulerAngles
	{
	public:
	// 公共变量

		// 直接的表示方式
		// 用弧度保存三个角度

		float heading;
		float pitch;
		float bank;
		
	// 公共操作

		EulerAngles() {}
		EulerAngles(float h, float p, float b):
		heading(h), pitch(p), bank(b){}
		void identity() { pitch = bank = heading = 0.0f; }

		// 变换为 "限制集" 欧拉角

		void  canonize();

		// 从四元数转换到欧拉角。
		// 输入的四元数假设为 物体-世界 或 世界-物体 四元数。
		void fromObjectToInertialQuternion(const Quaternion &q);
		void fromInertialToObjectQuaternion(const Quaternion &q);

		// 从矩阵转换到欧拉角
		//输入矩阵假设为 物体-世界 或世界-物体 转换矩阵
		//平移部分被忽略，并且假设矩阵是正交的
		void fromObjectToWorldMatrix(const Matrix4x3 &m);
		void fromWorldToObjectMatrix(const Matrix4x3 &m);

		// 从旋转矩阵到欧拉角

		void fromRotationMatrix(const RotationMatrix &m);
	};

	extern const EulerAngles kEulerAnglesIdentity;
}
