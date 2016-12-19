#include "FPScamera.h"
#include "EulerAngles.h"
#include "MathUtil.h"

//�ƶ�λ��

void FPScamera::move(float front, float left, float up) {

	// ǰ���ƶ�

	
	Quaternion con_dir = conjugate(dir);

	if (fabs(front) > 0.0f) {
		
		// ��ȡ��ת���z��

		Quaternion p;
		p.w = 0.0f; p.x = 0.0f; p.y = 0.0f; p.z = 1.0f;

		//���������ת�����������꣬ʹ��dir ������б任��
		p = conjugate(con_dir)*p*con_dir;

		// �ƶ�λ��pos

		pos += front*vector3(p.x, p.y, p.z);
	}

	// �����ƶ�

	if (fabs(left) > 0.0f) {

		// ��ȡ��ת���-x��

		Quaternion p;
		p.w = 0.0f; p.x = -1.0f; p.y = 0.0f; p.z = 0.0f;

		//���������ת�����������꣬ʹ��dir ������б任��
		p = conjugate(con_dir)*p*con_dir;

		//�ƶ�λ��pos

		pos += left*vector3(p.x, p.y, p.z);
	}

	// �����ƶ�

	if (fabs(up) > 0.0f) {

		//��ȡ��ת���y��

		Quaternion p;
		p.w = 0.0f; p.x = 0.0f; p.y = 1.0f; p.z = 0.0f;

		//���������ת�����������꣬ʹ��dir ������б任��
		p = conjugate(con_dir)*p*con_dir;

		//�ƶ�λ��pos

		pos += up*vector3(p.x, p.y, p.z);
	}


}

//ת������
//��λΪ�Ƕ�
// heading ��y����ת pitch ��x����ת�� bank ��z����ת

void FPScamera::rotate2D(float heading, float pitch) {

	//�Ƕ�ת����
	heading = heading / 180.0f*kPi;
	pitch = pitch / 180.0f*kPi;


	// �޸�heading�� ���Ʒ�Χ��[-Pi, Pi]
	_controlHeading += heading;
	_controlHeading = wrapPi(_controlHeading);

	// �޸�pitch�� ���Ʒ�Χ��[-pi/2, pi/2]
	_controlPitch += pitch;
	if (_controlPitch < -kPi * 0.45f)
		_controlPitch = -kPi * 0.45f;
	else if (_controlPitch > kPi * 0.45f)
		_controlPitch = kPi * 0.45f;

	// �����ת��Ԫ��

	Quaternion h, p;
	h.setToRotateAboutY(-_controlHeading);
	p.setToRotateAboutX(-_controlPitch);

	dir = h*p;
}

//��ȡ�������

Matrix4x3 FPScamera::getMatrix() {
	Matrix4x3 m1, m2;
	m1.setupTanslation(-pos);
	m2.fromQuaternion(dir);

	//��ƽ�Ƶ���������ϵ������ת��������ϵ��
	return m1*m2;
}

