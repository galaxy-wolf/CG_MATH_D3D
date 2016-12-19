#pragma once

#include "vector3.h"
#include "Quaternion.h"
#include "Matrix4x3.h"

using namespace CG_MATH;

class FPScamera
{
public:

	//��������
	
	vector3 pos;
	Quaternion dir;


	//��������

	// ��ʼ�����λ�ã���������ԭ�㣬����-Z�ᡣ

	FPScamera() :pos(0.0f, 0.0f, 0.0f), dir(kQuaternionIdentity), _controlHeading(0), _controlPitch(0), _controlBank(0){}
	
	//�ƶ�λ��

	void move(float front, float left, float up);

	//ת������

	void rotate2D(float heading, float pitch);

	//��ȡ�������

	Matrix4x3 getMatrix();

private:
	
	//˽������

	// ������������ŷ���ǣ� ��λΪ���ȡ�
	float _controlHeading;
	float _controlPitch;
	float _controlBank;
};

