#include "test.h"
#include <cstdio>



XMMATRIX toXMMATRIX(const Matrix4x4& m) {

	XMMATRIX r(
		m.m11, m.m12, m.m13, m.m14,
		m.m21, m.m22, m.m23, m.m24,
		m.m31, m.m32, m.m33, m.m34,
		m.m41, m.m42, m.m43, m.m44);

	return r;
}

void testEulerAngle() {

	EulerAngles e;
	Matrix4x3 m;
	m.setupLocalToParent(vector3(2.0f, -2.0f, -4.0f), EulerAngles(kPi / 2.0f, kPiOver2 / 5.0f, kPiOver2 / 3.0f));

	e.fromObjectToWorldMatrix(m);
	printf("%f %f %f \n", e.heading, e.pitch, e.bank);

	m.setupParentToLocal(vector3(2.0f, -2.0f, -4.0f), EulerAngles(kPi / 2.0f, kPiOver2 / 5.0f, kPiOver2 / 3.0f));
	e.fromWorldToObjectMatrix(m);
	printf("%f %f %f \n", e.heading, e.pitch, e.bank);

	Quaternion h, p, b, q;
	h.setToRotateAboutY(-kPi / 2.0f);
	p.setToRotateAboutX(-kPiOver2 / 5.0f);
	b.setToRotateAboutZ(-kPiOver2 / 3.0f);

	q = h*p*b;

	q.setToRotateInertialToObject(EulerAngles(kPi / 2.0f, kPiOver2 / 5.0f, kPiOver2 / 3.0f));

	e.fromInertialToObjectQuaternion(q);
	printf("%f %f %f \n", e.heading, e.pitch, e.bank);

	e.fromObjectToInertialQuternion(conjugate(q));
	printf("%f %f %f \n", e.heading, e.pitch, e.bank);


}



void RedirectOutPut() {
	freopen("out.txt", "w+", stdout);
}