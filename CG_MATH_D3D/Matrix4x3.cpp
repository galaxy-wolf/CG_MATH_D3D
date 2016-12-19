#include <assert.h>
#include <math.h>


#include "vector3.h"
#include "EulerAngles.h"
#include "Quaternion.h"
#include "RotationMatrix.h"
#include "MathUtil.h"
#include "Matrix4x3.h"


namespace CG_MATH
{

	///////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// 本类的设计目的是为了便于使用， 用户不用反复改变正负号或者转置知道结果“看起来正确”，当然，内部实现的细节是很重要的
	// 不仅是为了类的实现的正确性，也为了偶然可能发生的对矩阵元素的直接访问，或者为了优化
	// 因此， 这里描述一下矩阵类所用的约定
	//我们使用行向量，所以矩阵乘法形式如下：
	//		      |m11 m12 m13|     
	//			  |m21 m22 m23|   
	// |x y z| *  |m31 m32 m33| = |x' y' z'|
	//			  |tx  ty  tz |
	//
	//根据严格的线性代数法则，这种乘法是不成立的。
	//我们可以假设，输入和输出向量有第四个分量，都为1
	//另外，由于3x4矩阵是不能求逆的， 因此假设矩阵有第4行，为[0 0 0 1]
	//如下所示：
	//
	//               |m11 m12 m13 tx|   
	//			     |m21 m22 m23 ty|   
	//   |x y z 1| * |m31 m32 m33 tz| = |x' y' z' 1|
	//               |0   0   0   1 |   
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Matrix4x3 类成员
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////

	//-----------------------------------------------------------------------------------------------------
	// Matrix4x3::identity
	// 置为单位矩阵

	void Matrix4x3::identity() {
		m11 = 1.0f; m12 = 0.0f; m13 = 0.0f; 
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f; 
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f; 
		tx = 0.0f; ty = 0.0f; tz = 0.0f;
		
	}

	//----------------------------------------------------------------------------------------------------
	// Matrix4x3::zeroTranslation
	//将包含平移部分的第四列置为零

	void Matrix4x3::zeroTanslation() {
		tx = ty = tz = 0.0f;
	}

	//----------------------------------------------------------------------------------------------------
	// Matrix4x3::setTranslation
	//平移部分赋值，参数为向量形式

	void Matrix4x3::setTranslation(const vector3 &d) {
		tx = d.x; ty = d.y; tz = d.z;
	}

	//----------------------------------------------------------------------------------------------------
	// Matrix4x3::setupTanslation
	//生成平移矩阵，参数为向量形式

	void Matrix4x3::setupTanslation(const vector3 &d) {
		
		//线性变换部分置为单位矩阵

		m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;

		// 平移部分赋值

		tx = d.x; ty = d.y; tz = d.z;

	}

	//----------------------------------------------------------------------------------------------
	// Matrix4x3::setupLocalToParent
	// 构造执行局部->父空间变换的矩阵，局部空间的位置和范围在父空间中描述
	//该方法最常见的用途是构造物体->世界的变换矩阵，这个变换时非常直接的。
	//首先从物体空间变换到惯性空间，接着变换到世界空间
	//方位可以由欧拉角后旋转矩阵指定

	void Matrix4x3::setupLocalToParent(const vector3 &pos, const EulerAngles &orient)
	{
		
		// 创建一个旋转矩阵
		RotationMatrix orientMatrix;
		orientMatrix.setup(orient);

		// 构造3x4矩阵

		setupLocalToParent(pos, orientMatrix);
	}

	void Matrix4x3::setupLocalToParent(const vector3 &pos, const RotationMatrix &orient) {

		// 复制矩阵得旋转部分
		//根据RotationMatrix 中的注释，旋转矩阵“一般”是惯性-物体矩阵
		//是父-局部关系
		//我们求的是局部-父关系的矩阵，因此要做转置

		m11 = orient.m11; m12 = orient.m21; m13 = orient.m31;
		m21 = orient.m12; m22 = orient.m22; m23 = orient.m32;
		m31 = orient.m13; m32 = orient.m23; m33 = orient.m33;

		//现在设置平移部分。 平移再3x3 部分"之后“， 因此我们只需要简单复制其位置即可

		tx = pos.x; ty = pos.y; tz = pos.z;

	}

	//-------------------------------------------------------------------------------------------------
	// Matrix4x3::setupParentToLocal
	// 构造执行父-局部空间变换的矩阵，局部空间的位置和方位在父空间中描述
	// 该方法最常见的用途是构造世界-物体的变换矩阵
	// 通常这个变换首先从世界转换到惯性空间，接着转换到物体空间
	// 3x4 矩阵可以完成后一个转换
	// 所以我们想构造两个矩阵T和R，再连接M=RT
	// 方位可以由欧拉角或旋转矩阵指定

	void Matrix4x3::setupParentToLocal(const vector3 &pos, const EulerAngles &orient) {

		// 创建一个旋转矩阵

		RotationMatrix orientMatrix;
		orientMatrix.setup(orient);

		//构造3x4 矩阵

		setupParentToLocal(pos, orientMatrix);
	}

	void Matrix4x3::setupParentToLocal(const vector3 &pos, const RotationMatrix &orient) {
		
		// 赋值矩阵的旋转部分
		// 可以直接复制元素(不用转置)

		m11 = orient.m11; m12 = orient.m12; m13 = orient.m13;
		m21 = orient.m21; m22 = orient.m22; m23 = orient.m23;
		m31 = orient.m31; m32 = orient.m32; m33 = orient.m33;

		// 设置平移部分
		// 一般来说，从世界空间到惯性空间只需要平移负的量
		// 但必须记得旋转是“先”发生的，所以应该旋转平移部分
		// 这和先创建平移-pos的矩阵，再创建旋转矩阵R，
		// 再把它们连接成RT是一样的

		tx = -(pos.x*m11 + pos.y*m21 + pos.z*m31);
		ty = -(pos.x*m12 + pos.y*m22 + pos.z*m32);
		tz = -(pos.x*m13 + pos.y*m23 + pos.z*m33);

	}

	//------------------------------------------------------------------------------------------
	// Matrix4x3::setupRotate
	//构造绕坐标轴旋转的矩阵
	//旋转轴从1开始索引指定
	//
	// 1 => 绕x轴旋转
	// 2 => 绕y轴旋转
	// 3 => 绕z轴旋转
	// theta 是旋转量，以弧度表示，用右手法则定义“正方向”
	//平移部分置零

	void Matrix4x3::setupRotate(int axis, float theta) {
		
		// 取得旋转角的sin和cos值

		float s, c;
		sinCos(&s, &c, theta);

		//判断旋转轴

		switch (axis) {

		case 1: // 绕x轴旋转

			m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
			m21 = 0.0f; m22 = c;    m23 = s;
			m31 = 0.0f; m32 = -s;    m33 = c;
			break;

		case 2: //绕y轴旋转

			m11 = c;    m12 = 0.0f; m13 = -s;
			m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
			m31 = s;   m32 = 0.0f; m33 = c;
			break;

		case 3: //绕z轴旋转

			m11 = c; m12 = s; m13 = 0;
			m21 = -s; m22 = c;  m23 = 0;
			m31 = 0; m32 = 0;  m33 = 1;
			break;

		default:

			//非法索引

			assert(false);
 		}

		// 平移部分置零

		tx = ty = tz = 0.0f;
	}

	//-------------------------------------------------------------------------------------------------------
	// Matrix4x3::setupRotate
	//构造绕任意轴旋转，任意轴通过原点
	//旋转轴为单位向量
	//theta 是旋转向量，以弧度表示，用右手法则来定义"正方向"
	//平移部分置零

	void Matrix4x3::setupRotate(const vector3 &axis, float theta) {

		// 检查轴是否为单位向量

		assert(fabs(axis*axis - 1.0f) < .01f);

		// 取得旋转角的sin和cos值

		float s, c;
		sinCos(&s, &c, theta);

		// 计算1-cos(theta) 和一些公用的子表达式

		float a = 1.0f - c;
		float ax = a*axis.x;
		float ay = a*axis.y;
		float az = a*axis.z;

		// 矩阵元素赋值。
		// 仍有优化的机会，因为有许多相同的子表达式，我们把这个任务留给编译器

		m11 = ax*axis.x + c;
		m12 = ax*axis.y + axis.z*s;
		m13 = ax*axis.z - axis.y*s;

		m21 = ay*axis.x - axis.z*s;
		m22 = ay*axis.y + c;
		m23 = ay*axis.z + axis.x*s;

		m31 = az*axis.x + axis.y*s;
		m32 = az*axis.y - axis.x*s;
		m33 = az*axis.z + c;

		//平移部分置零

		tx = ty = tz = 0.0f;
	}


	//-------------------------------------------------------------------------------------------------------
	// Matrix4x3::fromQuaternion
	//从四元数转换到矩阵
	//平移部分置零

	void Matrix4x3::fromQuaternion(const Quaternion &q) {

		//计算一些公用的子表达式

		float ww = 2.0f*q.w;
		float xx = 2.0f*q.x;
		float yy = 2.0f*q.y;
		float zz = 2.0f*q.z;

		//矩阵元素赋值。
		// 仍有优化的机会，因为有许多相同的子表达式，我们把这个任务留给编译器

		m11 = 1.0f - yy*q.y - zz*q.z;
		m12 = xx*q.y + ww*q.z;
		m13 = xx*q.z - ww*q.y;

		m21 = xx*q.y - ww*q.z;
		m22 = 1.0f - xx*q.x - zz*q.z;
		m23 = yy*q.z + ww*q.x;

		m31 = xx*q.z + ww*q.y;
		m32 = yy*q.z - ww*q.x;
		m33 = 1.0f - xx*q.x - yy*q.y;

		//平移部分置零

		tx = ty = tz = 0.0f;
	}

	//---------------------------------------------------------------------------------------
	// Matrix4x3::setupScale
	//
	//构造沿各坐标轴缩放的矩阵
	//对于缩放因子k使用向量vector3(k,k,k) 表示
	//
	//平移部分置零

	void Matrix4x3::setupScale(const vector3 &s) {

		// 矩阵元素赋值，非常直接

		m11 = s.x;  m12 = 0.0f; m13 = 0.0f;
		m21 = 0.0f; m22 = s.y;  m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = s.z;

		//平移部分置零

		tx = ty = tz = 0.0f;
	}

	//------------------------------------------------------------------------------------------------------
	// Matrix4x3::setupScaleAlongAxis
	//
	//构造沿任意轴缩放的矩阵
	//旋转轴为单位向量
	//平移部分置零

	void Matrix4x3::setupScaleAlongAxis(const vector3 &axis, float k) {

		// 检查旋转轴是否为单位向量

		assert(fabs(axis*axis - 1.0f) < .01f);

		//计算k-1和常用的子表达式

		float a = k - 1.0f;
		float ax = a*axis.x;
		float ay = a*axis.y;
		float az = a*axis.z;

		//矩阵元素赋值，这里我们自己完成优化操作，因为对角元素相等

		m11 = ax*axis.x + 1.0f;
		m22 = ay*axis.y + 1.0f;
		m33 = az*axis.z + 1.0f;

		m12 = m21 = ax*axis.y;
		m13 = m31 = ax*axis.z;
		m23 = m32 = ay*axis.z;

		//平移部分置零

		tx = ty = tz = 0.0f;

	}

	//-----------------------------------------------------------------------------------
	// Matrix4x3::setupShear
	//构造切变矩阵
	//切变类型由一个索引指定，切变效果如下伪代码所示：
	//
	//	axis == 1 => y+= s*x, z+= t*x
	//  axis == 2 => x+= s*y, z+= t*y
	//  axis == 3 => x+= s*z, y+= t*z
	//平移部分置零

	void Matrix4x3::setupShear(int axis, float s, float t) {

		//判断切变类型

		switch (axis) {

		case 1: //用x切变y和z

			m11 = 1.0f; m12 = s;    m13 = t;
			m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
			m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
			break;

		case 2: //用y切变x和z

			m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
			m21 = s;    m22 = 1.0f; m23 = t;
			m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
			break;

		case 3: //用z切变x和y

			m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
			m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
			m31 = s;    m32 = t;    m33 = 1.0f;
			break;

		default:
			//非法索引

			assert(false);
		}

		// 平移部分置零

		tx = ty = tz = 0.0f;
	}

	//-------------------------------------------------------------------------------------------------
	// Matrix4x3::setupProject
	//构造投影矩阵，投影平面过原点，且垂直于单位向量n

	void Matrix4x3::setupProject(const vector3 &n) {

		//检查旋转轴是否为单位向量

		assert(fabs(n*n - 1.0f) < 0.01f);

		//矩阵元素赋值， 这里我们自己完成优化操作，因为其对角元素相等

		m11 = 1.0f - n.x*n.x;
		m22 = 1.0f - n.y*n.y;
		m33 = 1.0f - n.z*n.z;

		m12 = m21 = -n.x*n.y;
		m13 = m31 = -n.x*n.y;
		m23 = m32 = -n.y*n.z;

		//平移部分置零

		tx = ty = tz = 0.0f;
	}

	//-----------------------------------------------------------------------------------------
	// Matrix4x3::setupReflect
	//
	//构造反射平面，反射平面平行于坐标平面
	//
	//反射平面由一个索引指定
	//
	// 1=> 沿x= k 平面反射
	// 2=> 沿y= k 平面反射
	// 3=> 沿z= k 平面反射
	//
	//平移部分置为合适的值，因为k！= 0 时平移是一定会发生的

	void Matrix4x3::setupReflect(int axis, float k) {

		//判断反射平面

		switch (axis) {

		case 1: //沿x=k 平面反射

			m11 = -1.0f; m12 = 0.0f; m13 = 0.0f;
			m21 = 0.0f;  m22 = 1.0f; m23 = 0.0f;
			m31 = 0.0f;  m32 = 0.0f; m33 = 1.0f;

			tx = 2.0f * k;
			ty = 0.0f;
			tz = 0.0f;

			break;

		case 2: //沿y=k 平面反射

			m11 = 1.0f;  m12 = 0.0f;  m13 = 0.0f;
			m21 = 0.0f;  m22 = -1.0f; m23 = 0.0f;
			m31 = 0.0f;  m32 = 0.0f;  m33 = 1.0f;

			tx = 0.0f;
			ty = 2.0f * k;
			tz = 0.0f;

			break;

		case 3: //沿z=k 平面反射

			m11 = 1.0f;  m12 = 0.0f; m13 = 0.0f;
			m21 = 0.0f;  m22 = 1.0f; m23 = 0.0f;
			m31 = 0.0f;  m32 = 0.0f; m33 = -1.0f;

			tx = 0.0f;
			ty = 0.0f;
			tz = 2.0f * k;

			break;

		default:

			// 非法索引

			assert(false);
		}
	}

	//--------------------------------------------------------------------------------------------------
	//Matrix4x3::setupReflect
	//
	//构造反射矩阵，反射平面为通过原点的任意平面，且垂直于单位向量n
	//平移部分置零

	void Matrix4x3::setupReflect(const vector3 &n) {

		//检查旋转轴是否为单位向量

		assert(fabs(n*n - 1.0f) < 0.01f);

		//计算公共子表达式

		float ax = -2.0f*n.x;
		float ay = -2.0f*n.y;
		float az = -2.0f*n.z;

		//矩阵元素赋值，这里我们自己完成优化操作，因为其对角元素相等

		m11 = ax*n.x + 1.0f;
		m22 = ay*n.y + 1.0f;
		m33 = az*n.z + 1.0f;

		m12 = m21 = ax*n.y;
		m13 = m31 = ax*n.z;
		m23 = m32 = ay*n.z;

		//平移部分置零

		tx = ty = tz = 0.0f;
	}

	//----------------------------------------------------------------------------------------------
	// matrix4x3 * vector
	//
	//变换该点，这使得使用向量类就像在纸上做线性代数一样直观
	//

	vector3 operator*(const vector3 &p, const Matrix4x3 &m) {

		//根据线性代数法则

		return vector3(
			p.x*m.m11 + p.y*m.m21 + p.z*m.m31,
			p.x*m.m12 + p.y*m.m22 + p.z*m.m32,
			p.x*m.m13 + p.y*m.m23 + p.z*m.m33
		);
	}

	//----------------------------------------------------------------------------------
	// Matrix4x3 * Matrix4x3
	//
	// 矩阵连乘，这使得使用矩阵类就像在纸上做线性代数一样直观
	//提供*=运算符，以符合c语言的语法习惯
	//

	Matrix4x3 operator*(const Matrix4x3 &a, const Matrix4x3 &b) {

		Matrix4x3 r;

		//计算左上3x3(线性变换)部分

		r.m11 = a.m11*b.m11 + a.m12*b.m21 + a.m13*b.m31;
		r.m12 = a.m11*b.m12 + a.m12*b.m22 + a.m13*b.m32;
		r.m13 = a.m11*b.m13 + a.m12*b.m23 + a.m13*b.m33;

		r.m21 = a.m21*b.m11 + a.m22*b.m21 + a.m23*b.m31;
		r.m22 = a.m21*b.m12 + a.m22*b.m22 + a.m23*b.m32;
		r.m23 = a.m21*b.m13 + a.m22*b.m23 + a.m23*b.m33;

		r.m31 = a.m31*b.m11 + a.m32*b.m21 + a.m33*b.m31;
		r.m32 = a.m31*b.m12 + a.m32*b.m22 + a.m33*b.m32;
		r.m33 = a.m31*b.m13 + a.m32*b.m23 + a.m33*b.m33;

		//计算平移部分
		r.tx = a.tx*b.m11 + a.ty*b.m21 + a.tz*b.m31 + b.tx;
		r.ty = a.tx*b.m21 + a.ty*b.m22 + a.tz*b.m32 + b.ty;
		r.tz = a.tx*b.m31 + a.ty*b.m23 + a.tz*b.m33 + b.tz;

		return r;
	}

	Matrix4x3 &operator*= (Matrix4x3 &a, const Matrix4x3 &b) {
		a = a*b;
		return a;
	}

	vector3 &operator*=(vector3 &p, const Matrix4x3 &m) {
		p = p*m;
		return p;
	}

	//--------------------------------------------------------------------------------------
	// determinant
	//计算矩阵左上3x3部分的行列式

	float determinant(const Matrix4x3 &m) {
		return
			m.m11*(m.m22*m.m33 - m.m23*m.m32)
			+ m.m12*(m.m23*m.m31 - m.m21*m.m33)
			+ m.m13*(m.m21*m.m32 - m.m22*m.m31);
	}

	//------------------------------------------------------------------------------------------------------
	//inverse
	//
	//求矩阵的逆，使用经典的伴随矩阵除以行列式的方法

	Matrix4x3 inverse(const Matrix4x3 &m) {

		//计算行列式

		float det = determinant(m);

		//如果是奇异的，即行列式为零，则没有逆矩阵

		assert(fabsf(det) > 0.000001f);

		//计算1/行列式，这样除法只需要做一次

		float oneOverDet = 1.0f / det;

		//计算3x3部分的逆，用伴随矩阵除以行列式

		Matrix4x3 r;

		r.m11 = (m.m22*m.m33 - m.m23*m.m32) * oneOverDet;
		r.m12 = (m.m13*m.m32 - m.m12*m.m33) * oneOverDet;
		r.m13 = (m.m12*m.m23 - m.m13*m.m22) * oneOverDet;

		r.m21 = (m.m23*m.m31 - m.m21*m.m33) * oneOverDet;
		r.m22 = (m.m11*m.m33 - m.m13*m.m31) * oneOverDet;
		r.m23 = (m.m13*m.m21 - m.m11*m.m23) * oneOverDet;

		r.m31 = (m.m21*m.m32 - m.m22*m.m31) * oneOverDet;
		r.m32 = (m.m12*m.m31 - m.m11*m.m32) * oneOverDet;
		r.m33 = (m.m11*m.m22 - m.m12*m.m21) * oneOverDet;

		// 计算平移部分的逆

		r.tx = -(m.tx*r.m11 + m.ty*r.m21 + m.tz*r.m31);
		r.ty = -(m.tx*r.m12 + m.ty*r.m22 + m.tz*r.m32);
		r.tz = -(m.tx*r.m13 + m.ty*r.m23 + m.tz*r.m33);

		//返回值，这种方法需要调用拷贝构造函数
		//如果速度非常重要，需要用单独的函数在适当的地方给出返回值。

		return r;
	}

	//--------------------------------------------------------------------------------------------------
	//getTranslation
	//
	//以向量的形式返回平移部分

	vector3 getTranslation(const Matrix4x3 &m) {
		return vector3(m.tx, m.ty, m.tz);
	}

	//--------------------------------------------------------------------------------------------------
	//getPositionFromParentToLocalMatrix
	//
	//从父->局部变换矩阵中提取物体位置
	//假设矩阵代表刚体变换

	vector3 getPositionFromParentToLocalMatrix(const Matrix4x3 &m) {

		//负的平移值乘以3x3部分的转置
		//假设矩阵是正交的(该方法不能应用于非刚体变换)


		return vector3(
			-(m.tx*m.m11 + m.ty*m.m12 + m.tz*m.m13),
			-(m.tx*m.m21 + m.ty*m.m22 + m.tz*m.m23),
			-(m.tx*m.m31 + m.ty*m.m32 + m.tz*m.m33)
		);
	}

	//--------------------------------------------------------------------------------------------------
	//getPositionFromLocalToParentMatrix
	//
	//从局部->父(如物体->世界)变换矩阵中提取物体的位置

	vector3 getPositionFromLocalToParentMatrix(const Matrix4x3 &m) {

		// 所需要的位置就是平移部分

		return vector3(m.tx, m.ty, m.tz);
	}
}
