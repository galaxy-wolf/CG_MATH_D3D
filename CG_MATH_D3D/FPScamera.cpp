#include "FPScamera.h"
#include "EulerAngles.h"
#include "MathUtil.h"
#include "RotationMatrix.h"

//�ƶ�λ��

void FPScamera::move(float front, float left, float up) {

	RotationMatrix m;
	m.setup(dir);

	//��Ҫ�õ������������������ı�ʾ��
	// 1�� m �Ǵ��������굽��������ת����m��ת�ñ�ʾ������ת����
	// 2�� ������ת����������м�Ϊ���X Y Z ������������ı�ʾ��

	pos += vector3(m.m13, m.m23, m.m33) * front + vector3(m.m11, m.m21, m.m31) *-1.0f*left + vector3(m.m12, m.m22, m.m32) * up;

	
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

