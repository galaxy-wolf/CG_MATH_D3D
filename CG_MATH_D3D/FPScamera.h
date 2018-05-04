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

	//公共数据
	
	vector3 pos;
	EulerAngles dir;


	//公共操作

	// 初始化相机位置，世界坐标原点，朝向-Z轴。

	FPScamera() :pos(0.0f, 0.0f, 0.0f), dir(kEulerAnglesIdentity) {}
	
	//移动位置

	void move(float front, float left, float up);

	//转动方向

	void rotate2D(float heading, float pitch);

	//获取相机矩阵

	Matrix4x3 getMatrix();

};

