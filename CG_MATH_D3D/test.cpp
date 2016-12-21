#include "test.h"
#include <cstdio>

void RedirectOutPut() {
	freopen("out.txt", "w+", stdout);
}


XMMATRIX toXMMATRIX(const Matrix4x4& m) {

	XMMATRIX r(
		m.m11, m.m12, m.m13, m.m14,
		m.m21, m.m22, m.m23, m.m24,
		m.m31, m.m32, m.m33, m.m34,
		m.m41, m.m42, m.m43, m.m44);

	return r;
}

XMMATRIX toXMMATRIX(const RotationMatrix& m) {

	XMMATRIX r(
		m.m11, m.m12, m.m13, 0,
		m.m21, m.m22, m.m23, 0,
		m.m31, m.m32, m.m33, 0,
		0,	   0,	  0,	 1.0f);

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


XMMATRIX testMatrix4x3() {
	
	RedirectOutPut();

	Matrix4x3 m;
	m.identity();

	m.setupTanslation(vector3(2.0f, -2.0f, -4.0f));

	m.setupLocalToParent(vector3(2.0f, -2.0f, -4.0f), EulerAngles(kPiOver2, kPiOver2 / 2, 0));

	RotationMatrix r;
	r.setup(EulerAngles(kPiOver2, kPiOver2 / 2, 0));
	m.setupLocalToParent(vector3(2.0f, -2.0f, -4.0f), r);

	m.setupLocalToParent(vector3(2.0f, -2.0f, -4.0f), EulerAngles(kPiOver2, kPiOver2 / 2, 0));

	Matrix4x3 m2;

	m2.setupParentToLocal(vector3(2.0f, -2.0f, -4.0f), EulerAngles(kPiOver2, kPiOver2 / 2, 0));

	m *= m2;


	r.setup(EulerAngles(kPiOver2, kPiOver2 / 2, 0));
	m.setupLocalToParent(vector3(2.0f, -2.0f, -4.0f), r);

	vector3 pos1 = getPositionFromLocalToParentMatrix(m);
	printf("%f %f %f\n", pos1.x, pos1.y, pos1.z);



	r.setup(EulerAngles(kPiOver2, kPiOver2 / 2, 0));
	m2.setupParentToLocal(vector3(2.0f, -2.0f, -4.0f), r);

	vector3 pos2 = getPositionFromParentToLocalMatrix(m2);
	printf("%f %f %f\n", pos2.x, pos2.y, pos2.z);

	m = m*m2;

	m.setupRotate(1, kPiOver2 / 2);

	m.setupRotate(2, kPiOver2 / 2);


	m.setupRotate(3, kPiOver2 / 2);

	vector3 axis = vector3(1.0f, 1.0f, 1.0f);
	axis.normalize();

	m.setupRotate(axis, kPiOver2 / 3);

	Quaternion q;
	q.setToRotateAboutAxis(axis, kPi);
	m.fromQuaternion(q);

	m.setupScale(vector3(2.0f, 3.0f, 0.5f));
	m.setupScaleAlongAxis(axis, 2.0f);
	m.setupShear(3, 0.5f, 0.6f);

	m.setupProject(axis);

	m.setupReflect(3, -2.0f);

	m.setupReflect(axis);

	m *= inverse(m);

	return toXMMATRIX(m);

}

XMMATRIX testQuaternion() {

	Quaternion q;
	Matrix4x3 m;


	m.fromQuaternion(q);
	return toXMMATRIX(m);

}


void PrintRotationMatrix(const RotationMatrix &r) {

	printf("\n%f %f %f\n", r.m11, r.m12, r.m13);
	printf("%f %f %f\n", r.m21, r.m22, r.m23);
	printf("%f %f %f\n\n", r.m31, r.m32, r.m33);

}

XMMATRIX testRotationMatrix() {

	RedirectOutPut();

	RotationMatrix r, r1, r2;
	Quaternion q1, q2;

	q1.setToRotateObjectToInertial(EulerAngles(kPiOver2 / 2, kPiOver2 / 3, kPiOver2));
	q2.setToRotateInertialToObject(EulerAngles(kPiOver2 / 2, kPiOver2 / 3, kPiOver2));
	
	r.identity();

	r1.fromObjectToInertialQuaternion(q1);

	r.setup(EulerAngles(kPiOver2/2, kPiOver2/3, kPiOver2));

	r2.fromInertialToObjectQuaternion(q2);
	
	/*PrintRotationMatrix(r1);
	PrintRotationMatrix(r);
	PrintRotationMatrix(r2);*/

	vector3 a(1.0f, 2.0f, -2.0f);

	a = r1.inertialToObject(a);
	printf("%f %f %f\n", a.x, a.y, a.z);
	a = r1.objectToInertial(a);
	printf("%f %f %f\n", a.x, a.y, a.z);

	return toXMMATRIX(r);

}