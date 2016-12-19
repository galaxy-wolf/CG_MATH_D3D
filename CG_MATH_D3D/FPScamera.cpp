#include "FPScamera.h"
#include "EulerAngles.h"
#include "MathUtil.h"

//移动位置

void FPScamera::move(float front, float left, float up) {

	// 前后移动

	
	Quaternion con_dir = conjugate(dir);

	if (fabs(front) > 0.0f) {
		
		// 获取旋转后的z轴

		Quaternion p;
		p.w = 0.0f; p.x = 0.0f; p.y = 0.0f; p.z = 1.0f;

		//从相机坐标转换到世界坐标，使用dir 的逆进行变换。
		p = conjugate(con_dir)*p*con_dir;

		// 移动位置pos

		pos += front*vector3(p.x, p.y, p.z);
	}

	// 左右移动

	if (fabs(left) > 0.0f) {

		// 获取旋转后的-x轴

		Quaternion p;
		p.w = 0.0f; p.x = -1.0f; p.y = 0.0f; p.z = 0.0f;

		//从相机坐标转换到世界坐标，使用dir 的逆进行变换。
		p = conjugate(con_dir)*p*con_dir;

		//移动位置pos

		pos += left*vector3(p.x, p.y, p.z);
	}

	// 上下移动

	if (fabs(up) > 0.0f) {

		//获取旋转后的y轴

		Quaternion p;
		p.w = 0.0f; p.x = 0.0f; p.y = 1.0f; p.z = 0.0f;

		//从相机坐标转换到世界坐标，使用dir 的逆进行变换。
		p = conjugate(con_dir)*p*con_dir;

		//移动位置pos

		pos += up*vector3(p.x, p.y, p.z);
	}


}

//转动方向
//单位为角度
// heading 绕y轴旋转 pitch 绕x轴旋转， bank 绕z轴旋转

void FPScamera::rotate2D(float heading, float pitch) {

	//角度转弧度
	heading = heading / 180.0f*kPi;
	pitch = pitch / 180.0f*kPi;


	// 修改heading， 限制范围在[-Pi, Pi]
	_controlHeading += heading;
	_controlHeading = wrapPi(_controlHeading);

	// 修改pitch， 限制范围在[-pi/2, pi/2]
	_controlPitch += pitch;
	if (_controlPitch < -kPi * 0.45f)
		_controlPitch = -kPi * 0.45f;
	else if (_controlPitch > kPi * 0.45f)
		_controlPitch = kPi * 0.45f;

	// 获得旋转四元数

	Quaternion h, p;
	h.setToRotateAboutY(-_controlHeading);
	p.setToRotateAboutX(-_controlPitch);

	dir = h*p;
}

//获取相机矩阵

Matrix4x3 FPScamera::getMatrix() {
	Matrix4x3 m1, m2;
	m1.setupTanslation(-pos);
	m2.fromQuaternion(dir);

	//先平移到惯性坐标系，再旋转到视坐标系。
	return m1*m2;
}

