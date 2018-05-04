#include "FPScamera.h"
#include "EulerAngles.h"
#include "MathUtil.h"
#include "RotationMatrix.h"
#include "vector3.h"

using CG_MATH::RotationMatrix;
using CG_MATH::kPi;
using CG_MATH::wrapPi;

//移动位置

void FPScamera::move(float front, float left, float up) {

	RotationMatrix m;
	m.setup(dir);

	// 移动发生在相机的物体坐标系

	pos += m.objectToInertial(vector3(-left, up, front));
}

//转动方向
//单位为角度
// heading 绕y轴旋转 pitch 绕x轴旋转， bank 绕z轴旋转

void FPScamera::rotate2D(float heading, float pitch) {

	//角度转弧度
	heading = heading / 180.0f*kPi;
	pitch = pitch / 180.0f*kPi;


	// 修改heading， 限制范围在[-Pi, Pi]
	dir.heading += heading;
	dir.heading = wrapPi(dir.heading);

	// 修改pitch， 限制范围在[-pi/2, pi/2]
	dir.pitch += pitch;
	if (dir.pitch < -kPi * 0.45f)
		dir.pitch = -kPi * 0.45f;
	else if (dir.pitch > kPi * 0.45f)
		dir.pitch = kPi * 0.45f;

}

//获取相机矩阵

Matrix4x3 FPScamera::getMatrix() {
	
	Matrix4x3 m;
	m.setupParentToLocal(pos, dir);

	return m;
}

