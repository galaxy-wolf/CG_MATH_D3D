#pragma once
#include <math.h>
namespace CG_MATH
{
	const float kPi = 3.14159265f;
	const float k2Pi = kPi * 2.0f;
	const float kPiOver2 = kPi / 2.0f;
	const float k1OverPi = 1.0f / kPi;
	const float k1Over2Pi = 1.0f / k2Pi;
	// 通过加上适当的2Pi 倍数将角度限制在-pi 到pi 的区间内。
	extern float wrapPi(float theta);
	// “安全” 反三角函数
	extern float safeAcos(float x);
	//计算sin 和 cos 值
	//在某些平台上，如果需要这两个值， 同时计算要比分开计算快。
	inline void sinCos(float *returnSin, float *returnCos, float theta)
	{
		//为了简单， 我们只使用标准三角函数
		//注意在某些平台上可以做得更好一些。
		*returnSin = sinf(theta);
		*returnCos = cosf(theta);
	}

}

