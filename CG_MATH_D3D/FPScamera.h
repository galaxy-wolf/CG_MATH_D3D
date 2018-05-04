#pragma once

#include "vector3.h"
#include "Matrix4x3.h"
#include "EulerAngles.h"

using CG_MATH::vector3;
using CG_MATH::EulerAngles;
using CG_MATH::kEulerAnglesIdentity;
using CG_MATH::Matrix4x3;

class FPScamera
{
public:

	//��������
	
	vector3 pos;
	EulerAngles dir;


	//��������

	// ��ʼ�����λ�ã���������ԭ�㣬����-Z�ᡣ

	FPScamera() :pos(0.0f, 0.0f, 0.0f), dir(kEulerAnglesIdentity) {}
	
	//�ƶ�λ��

	void move(float front, float left, float up);

	//ת������

	void rotate2D(float heading, float pitch);

	//��ȡ�������

	Matrix4x3 getMatrix();

};

