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

DirectX::XMMATRIX toXMMATRIX(const CG_MATH::Matrix4x4& m);
int testAllFile();
void RedirectOutPut();
