#pragma once

#include "vector3.h"
#include "Quaternion.h"
#include "Matrix4x3.h"

using namespace CG_MATH;

class FPScamera
{
public:

	//公共数据
	
	vector3 pos;
	Quaternion dir;


	//公共操作

	// 初始化相机位置，世界坐标原点，朝向-Z轴。

	FPScamera() :pos(0.0f, 0.0f, 0.0f), dir(kQuaternionIdentity), _controlHeading(0), _controlPitch(0), _controlBank(0){}
	
	//移动位置

	void move(float front, float left, float up);

	//转动方向

	void rotate2D(float heading, float pitch);

	//获取相机矩阵

	Matrix4x3 getMatrix();

private:
	
	//私有数据

	// 控制相机方向的欧拉角， 单位为弧度。
	float _controlHeading;
	float _controlPitch;
	float _controlBank;
};

