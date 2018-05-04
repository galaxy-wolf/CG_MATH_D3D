#include "FPScamera.h"
#include "EulerAngles.h"
#include "MathUtil.h"
#include "RotationMatrix.h"
#include "vector3.h"

using CG_MATH::RotationMatrix;
using CG_MATH::kPi;
using CG_MATH::wrapPi;

//�ƶ�λ��

void FPScamera::move(float front, float left, float up) {

	RotationMatrix m;
	m.setup(dir);

	// �ƶ��������������������ϵ

	pos += m.objectToInertial(vector3(-left, up, front));
}

//ת������
//��λΪ�Ƕ�
// heading ��y����ת pitch ��x����ת�� bank ��z����ת

void FPScamera::rotate2D(float heading, float pitch) {

	//�Ƕ�ת����
	heading = heading / 180.0f*kPi;
	pitch = pitch / 180.0f*kPi;


	// �޸�heading�� ���Ʒ�Χ��[-Pi, Pi]
	dir.heading += heading;
	dir.heading = wrapPi(dir.heading);

	// �޸�pitch�� ���Ʒ�Χ��[-pi/2, pi/2]
	dir.pitch += pitch;
	if (dir.pitch < -kPi * 0.45f)
		dir.pitch = -kPi * 0.45f;
	else if (dir.pitch > kPi * 0.45f)
		dir.pitch = kPi * 0.45f;

}

//��ȡ�������

Matrix4x3 FPScamera::getMatrix() {
	
	Matrix4x3 m;
	m.setupParentToLocal(pos, dir);

	return m;
}

