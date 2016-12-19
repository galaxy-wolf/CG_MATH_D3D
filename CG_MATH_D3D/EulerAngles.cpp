#include "EulerAngles.h"

#include <math.h>

#include "EulerAngles.h"
#include "Quaternion.h"
#include "MathUtil.h"
#include "Matrix4x3.h"
#include "RotationMatrix.h"

namespace CG_MATH
{
	// 全局“单位”欧拉角常量
	//现在我们还不知道构造它的确切时机，这要取决于其他对象， 因此有可能在该对象被初始化之前就引用它，不过
	//在大多数实现中， 它将在程序开始时被初始化化为0， 即发生在其他对象被构造之前。

	const EulerAngles kEulerAnglesIdentity(0.0f, 0.0f, 0.0f);

	//--------------------------------------------------------------
	// EulerAngles::canonize
	//将欧拉角转换到限制集中
	//就表示3D范围的目的而言， 它不会改变欧拉角的值
	//但对于其他表示对象如加速度等，则会产生影响。
	
	void EulerAngles::canonize()
	{
		// 首先将pitch 变换到-pi 到 pi之间
		pitch = wrapPi(pitch);

		// 现在将 pitch变换到 -pi/2 到 pi/2 之间

		if (pitch < -kPiOver2) {
			pitch = -kPi - pitch;
			heading += kPi;
			bank += kPi;
		}
		else if (pitch > kPiOver2)
		{
			pitch = kPi - pitch;
			heading += kPi;
			bank += kPi;
		}

		// 现在检查万向锁的情况，允许一定的误差

		if (fabs(pitch) > kPiOver2 - 1e-4)
		{

			// 在万向锁中， 所有垂直轴的旋转赋给heading

			heading += bank;
			bank = 0.0f;

		}
		else {

			// 非万向锁，将bank转换到限制集中

			bank = wrapPi(bank);
		}

		// 将heading转换到限制集中

		heading = wrapPi(heading);
	}


	//-----------------------------------------------------------------------
	// EurlerAngles::fromWorldToObjectMatrix
	//从世界-物体坐标系变换矩阵到欧拉角
	//
	// 假设矩阵是正交的， 忽略平移部分
	void EulerAngles::fromWorldToObjectMatrix(const Matrix4x3 &m) {
		
		// 根据m32 计算sin(pitch)

		float sp = -m.m23;

		// 检查万向锁
		if (fabs(sp) > 9.99999f) {

			// 向上方或者下方看
			pitch = kPiOver2 * sp;

			// bank 置0， 计算heading
			heading = atan2f(-m.m31, m.m11);
			bank = 0.0f;
		}
		else {

			// 计算角度

			heading = atan2f(m.m13, m.m33);
			pitch = asinf(sp);
			bank = atan2f(m.m21, m.m22);
		}
	}

	//----------------------------------------------------------------------
	// EulerAngles::fromObjectToWorldMatrix
	//从物体-世界坐标系变换矩阵到欧拉角
	//假设矩阵是正交的，忽略平移部分
	void EulerAngles::fromObjectToWorldMatrix(const Matrix4x3 &m)
	{
		
		//通过m23计算sin(pitch).
		
		float sp = -m.m32;
		
		//检查万向锁

		if (fabsf(sp) > 9.99999f) {

			// 向正上或正下方看

			pitch = kPiOver2 * sp;

			// bank 置零， 计算heading

			heading = atan2f(-m.m13, m.m11);
			bank = 0.0f;
		}
		else {

			// 计算角度

			heading = atan2f(m.m31, m.m33);
			pitch = asinf(sp);
			bank = atan2f(m.m12, m.m22);
		}
	}


	//-------------------------------------------------------------------
	// EulerAngles::fromObjectToInertialQuaternion
	//
	// 从物体-惯性四元数 到欧拉角

	void EulerAngles::fromObjectToInertialQuternion(const Quaternion &q)
	{

		// 计算 sin(pitch)
		
		float sp = -2.0f*(q.y*q.z - q.w*q.x);

		// 检查万向锁，允许存在一定误差
		
		if (fabs(sp) > 0.99999f)
		{
			// 向上方或正下方看

			pitch = kPiOver2 * sp;

			// bank 置零， 计算heading

			heading = atan2f(q.w*q.y - q.x*q.z, 0.5f - q.y*q.y - q.z*q.z);
			bank = 0.0f;
		}
		else {

			// 计算角度

			pitch = asinf(sp);
			heading = atan2f(q.x*q.z + q.w*q.y, 0.5f - q.x*q.x - q.y*q.y);
			bank = atan2f(q.x*q.y + q.w*q.z, 0.5f - q.x*q.x - q.z*q.z);
		}
	}

	//-----------------------------------------------------------------------
	// EulerAngles::fromInertialToObjectQuaternion
	//
	// 从惯性-物体四元数 到欧拉角

	void EulerAngles::fromInertialToObjectQuaternion(const Quaternion &q)
	{

		// 计算 sin(pitch)

		float sp = -2.0f*(q.y*q.z + q.w*q.x);

		// 检查万向锁，允许存在一定误差

		if (fabs(sp) > 0.99999f)
		{
			// 向上方或正下方看

			pitch = kPiOver2 * sp;

			// bank 置零， 计算heading

			heading = atan2f(-q.x*q.z - q.w*q.y, 0.5f - q.y*q.y - q.z*q.z);
			bank = 0.0f;
		}
		else {

			// 计算角度

			pitch = asinf(sp);
			heading = atan2f(q.x*q.z - q.w*q.y, 0.5f - q.x*q.x - q.y*q.y);
			bank = atan2f(q.x*q.y - q.w*q.z, 0.5f - q.x*q.x - q.z*q.z);
		}
	}

	//-------------------------------------------------------------------
	// EulerAngles::fromRotationMatrix(const RotationMatrix &m)
	// 
	// 根据旋转矩阵构造欧拉角

	void EulerAngles::fromRotationMatrix(const RotationMatrix &m) {


		// 根据m32 计算sin(pitch)

		float sp = -m.m23;

		// 检查万向锁
		if (fabs(sp) > 9.99999f) {

			// 向上方或者下方看
			pitch = kPiOver2 * sp;

			// bank 置0， 计算heading
			heading = atan2f(-m.m31, m.m11);
			bank = 0.0f;
		}
		else {

			// 计算角度

			heading = atan2f(m.m13, m.m33);
			pitch = asinf(sp);
			bank = atan2f(m.m21, m.m22);
		}
	}

}