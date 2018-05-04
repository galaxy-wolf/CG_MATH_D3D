#pragma once

namespace CG_MATH
{
	class Matrix4x3;

	//----------------------------------------------------------------------------------------
	//Matrix4x4��
	//ʵ��4x4ת������

	class Matrix4x4
	{
	public:

		// ��������

		//�����ֵ
		
		float m11, m12, m13, m14;
		float m21, m22, m23, m24;
		float m31, m32, m33, m34;
		float m41, m42, m43, m44;

		// ��������

		//��Ϊ��λ����

		void identity();

		// ��4x3������
		
		explicit Matrix4x4(const Matrix4x3 &a); 
		// Ĭ�Ϲ��캯��
		
		Matrix4x4() = default;

		// ����ͶӰ�任����

		void setupPerspectiveFov(float fov, float aspect, float near, float far);
		void setupFrustum(float left, float right, float bottom, float top, float near, float far);

	};

	// �����* �������Ӿ��󣬳˷���˳��������ң���任˳����ͬ��

	Matrix4x4 operator*(const Matrix4x4 &a, const Matrix4x4 &m);

	// �����*= ���ֺ�c++��׼�﷨һ�¡�

	Matrix4x4 &operator*= (Matrix4x4 &a, const Matrix4x4 &m);

	// ����������
	//Matrix4x4 inverse(const Matrix4x4 &m);

}