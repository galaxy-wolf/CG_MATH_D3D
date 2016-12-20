#pragma once

#include <directxmath.h>

#include "EulerAngles.h"
#include "inputclass.h"
#include "MathUtil.h"
#include "Matrix4x3.h"
#include "Matrix4x4.h"
#include "Quaternion.h"
#include "RotationMatrix.h"
#include "vector3.h"


using namespace CG_MATH;
using namespace DirectX;

XMMATRIX toXMMATRIX(const Matrix4x4& m);
void testEulerAngle();

void RedirectOutPut();