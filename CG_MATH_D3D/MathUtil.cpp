#include "MathUtil.h"

namespace CG_MATH
{
	//---------------------------------------------------------
	// 通过加上适当的2Pi倍数，将角度限制在-pi 到pi 的区间内。
	float wrapPi(float theta)
	{
		theta += kPi;
		theta -= floorf(theta * k1Over2Pi) *k2Pi;
		theta -= kPi;
		return theta;
	}

	//----------------------------------------------------------
	// safeAcos
	// 和acos（x）相同， 但如果x超出范围将返回最接近的有效值
	// 返回值在0 到 pi之间，和c语言中标准acos函数相同。
	float safeAcos(float x)
	{
		// 检查边界条件

		if (x <= -1.0f)
		{
			return kPi;
		}
		if (x >= 1.0f)
		{
			return 0.0f;
		}

		return acosf(x);
	}
}