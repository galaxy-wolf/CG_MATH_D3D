#pragma once

namespace CG_MATH
{

	class vector3;
	class EulerAngles;
	class Quaternion;
	class RotationMatrix;

	//--------------------------------------------------------------------------------------
	//Matrix4*3类
	// 实现3*4转换矩阵，能够表达任何3D仿射变换

	class Matrix4x3
	{
	public:
		
		// 公共数据

		// 矩阵的值
		//前3列包含线性变换，最后一列包含平移

		float m11, m12, m13;
		float m21, m22, m23;
		float m31, m32, m33;
		float tx, ty, tz;

		// 公共操作

		//置为单位矩阵

		void identity();

		// 直接访问平移部分

		void zeroTranslation();
		void setTranslation(const vector3 &d);
		void setupTranslation(const vector3 &d);

		// 构造执行父空间 <->局部空间变换的矩阵，假定局部空间在指定的位置和方位， 该方位可能是使用欧拉角或旋转矩阵表示的。

		void setupLocalToParent(const vector3 &pos, const EulerAngles &orient);
		void setupLocalToParent(const vector3 &pos, const RotationMatrix &orient);
		void setupParentToLocal(const vector3 &pos, const EulerAngles &orient);
		void setupParentToLocal(const vector3 &pos, const RotationMatrix &orient);

		// 构造绕坐标轴旋转的矩阵

		void setupRotate(int axis, float theta);

		// 构造绕任意轴旋转的矩阵

		void setupRotate(const vector3 &axis, float theta);

		//构造旋转矩阵，角位移由四元数形式给出

		void fromQuaternion(const Quaternion &q);

		// 构造沿坐标轴缩放的矩阵

		void setupScale(const vector3 &s);

		// 构造沿任意轴缩放的矩阵

		void setupScaleAlongAxis(const vector3 &axis, float k);

		// 构造切变矩阵

		void setupShear(int axis, float s, float t);

		// 构造投影矩阵， 投影平面过原点

		void setupProject(const vector3 &n);

		// 构造反射矩阵

		void setupReflect(int axis, float k = 0.0f);

		// 构造沿任意平面反射的矩阵

		void setupReflect(const vector3 &n);

	};

	// 运算符* 用来变换点或连接矩阵，乘法的顺序从左向右，与变换顺序相同。

	vector3 operator*(const vector3 &p, const Matrix4x3 &m);
	Matrix4x3 operator*(const Matrix4x3 &a, const Matrix4x3 &b);

	// 运算符*= 保持和c++标准语法的一致性

	vector3 &operator*=(vector3 &p, const Matrix4x3 &m);
	Matrix4x3 &operator*=(Matrix4x3 &a, const Matrix4x3 &m);


	// 计算3x3 部分的行列式
	float determinant(const Matrix4x3 &m);

	//计算矩阵的逆

	Matrix4x3 inverse(const Matrix4x3 &m);

	// 提取矩阵的平移部分

	vector3 getTranslation(const Matrix4x3 &m);

	// 从局部矩阵->父矩阵 或 父矩阵->局部矩阵 取位置/方位

	vector3 getPositionFromParentToLocalMatrix(const Matrix4x3 &m);
	vector3 getPositionFromLocalToParentMatrix(const Matrix4x3 &m);

}


