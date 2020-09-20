#include "RT_analysis.h"
#define THRESHOLD_RAY 0.0000001
namespace Antops
{
	RTAnalysis::RTAnalysis()
	{
	}

	RTAnalysis::~RTAnalysis()
	{
	}

	bool RTAnalysis::Init(const RayTracingOption & opt)
	{
		opt_ = opt;
		return true;
	}

	int RTAnalysis::CalcReflect(const RayLineCluster & in, RayLineCluster & out)
	{
		Prepare();

		out.normal_line.reserve(in.normal_line.size());
		out.start_point.reserve(in.start_point.size());
		Vector3 norm;
		Vector3 start;
		bool isIntersect = false;
		for (size_t i = 0; i < in.normal_line.size(); i++) {
			isIntersect = false;
			CalcReflectByQuadricSurface(in.start_point[i], in.normal_line[i],
				norm, start, isIntersect);

			if (isIntersect || !opt_.is_ign_non_intersection) {
				// 保留反射
				out.normal_line.push_back(norm);
				out.start_point.push_back(start);
			}
		}
		return 0;
	}

	int RTAnalysis::CalcNormalOfLineMirror(
		const std::vector<Vector3>& startPiont,
		const std::vector<Vector3>& direction,
		std::vector<Vector3> &normal,
		std::vector<Vector3> &intersection,
		std::vector<bool> &isIntersect,
		std::vector<double>& t)
	{
		Prepare();
		for (size_t i = 0; i < startPiont.size(); i++) {
			bool isTmep = false;
			double port = 0;
			CalcNormalOfLineMirrorByQuadricSurface(startPiont[i], direction[i], normal[i], intersection[i], isTmep, port);
			isIntersect[i] = isTmep;
			t[i] = port;
		}
		return 0;
	}

	void RTAnalysis::CalcReflectByQuadricSurface(
		const Vector3 & startPiont,
		const Vector3 & direction,
		Vector3 & reflex,
		Vector3 & intersection,
		bool & isIntersect)
	{
		double t;
		const std::vector<double>& tempData = opt_.param->param;


		// 将世界坐标系转到模型的相对坐标系
		Vector3 tempStartPiont = R_translate_matrix_[0] * R_rotat_matrix_[0] * startPiont;
		Vector3 tempDirection = R_rotat_matrix_[0] * direction; // 向量只用求旋转

		if (RayCurvedSurface(tempData, tempDirection, tempStartPiont, t, intersection))
		{
			Vector3 intersectionGlobal = translate_matrix_[0] * rotat_matrix_[0] * intersection;

			if (!IsInRestriction(intersectionGlobal)) //不满足限制条件
			{
				intersection = startPiont; // 让交点等于起点 方向不变 避免对无交点时特殊处理
				reflex = direction;
				isIntersect = false;
				return;
			}

			double x = 2 * tempData[0] * intersection.x + tempData[3] * intersection.y +
				tempData[5] * intersection.z + tempData[6];
			double y = 2 * tempData[1] * intersection.y + tempData[3] * intersection.x +
				tempData[4] * intersection.z + tempData[7];
			double z = 2 * tempData[2] * intersection.z + tempData[4] * intersection.y +
				tempData[5] * intersection.x + tempData[8];
			Vector3 tempn(x, y, z);
			if (tempn.Dot(tempDirection) > 0.0)
				tempn.Set(-x, -y, -z);

			reflex = RayTracing::ReflectLight(tempDirection, tempn);
			isIntersect = true;

			// 将模型的相对坐标系转到世界坐标系
			intersection = intersectionGlobal;
			reflex = rotat_matrix_[0] * reflex; // 更新方向

		}
		else
		{
			intersection = startPiont; // 让交点等于起点 方向不变 避免对无交点时特殊处理
			reflex = direction;
			isIntersect = false;
		}

		return;
	}

	void RTAnalysis::CalcNormalOfLineMirrorByQuadricSurface(
		const Vector3 & startPiont, 
		const Vector3 & direction,
		Vector3 & normal, 
		Vector3 & intersection, 
		bool & isIntersect,
		double & t)
	{
		const std::vector<double>& tempData = opt_.param->param;

		// 将世界坐标系转到模型的相对坐标系
		Vector3 tempStartPiont = R_translate_matrix_[0] * R_rotat_matrix_[0] * startPiont;
		Vector3 tempDirection = R_rotat_matrix_[0] * direction; // 向量只用求旋转

		double t1, t2;
		bool isOK1 = false;
		bool isOK2 = false;
		Vector3 intersection1, intersection2;
		RayCurvedSurface(tempData, tempDirection, tempStartPiont, t1, t2, isOK1,
			isOK2, intersection1, intersection2);
		if (isOK1&&isOK2)
		{
			Vector3 intersectionGlobal1 = translate_matrix_[0] *
				rotat_matrix_[0] * intersection1;
			Vector3 intersectionGlobal2 = translate_matrix_[0] *
				rotat_matrix_[0] * intersection2;
			bool isInRestriction1 = IsInRestriction(intersectionGlobal1);
			bool isInRestriction2 = IsInRestriction(intersectionGlobal2);
			if (isInRestriction1&&isInRestriction2) // 两个解都满足 取绝地值小的
			{
				if (abs(t1) > abs(t2))
				{
					t = t2;
					intersection = intersection2;
				}
				else
				{
					t = t1;
					intersection = intersection1;
				}
			}
			else if (isInRestriction1)
			{
				t = t1;
				intersection = intersection1;
			}
			else if (isInRestriction2)
			{
				t = t2;
				intersection = intersection2;
			}
			else
			{
				intersection = startPiont; // 让交点等于起点 方向不变 避免对无交点时特殊处理
				isIntersect = false;
				return;
			}
		}
		else if (isOK1)
		{
			Vector3 intersectionGlobal1 = translate_matrix_[0] *
				rotat_matrix_[0] * intersection1;
			if (IsInRestriction(intersectionGlobal1))
			{
				t = t1;
				intersection = intersection1;
			}
			else
			{
				intersection = startPiont; // 让交点等于起点 方向不变 避免对无交点时特殊处理
				isIntersect = false;
				return;
			}
		}
		else if (isOK2)
		{
			Vector3 intersectionGlobal2 = translate_matrix_[0] *
				rotat_matrix_[0] * intersection2;
			if (IsInRestriction(intersectionGlobal2))
			{
				t = t2;
				intersection = intersection2;
			}
			else
			{
				intersection = startPiont; // 让交点等于起点 方向不变 避免对无交点时特殊处理
				isIntersect = false;
				return;
			}
		}
		else
		{
			intersection = startPiont; // 让交点等于起点 方向不变 避免对无交点时特殊处理
			isIntersect = false;
			return;
		}
		double x = 2 * tempData[0] * intersection.x + tempData[3] * intersection.y +
			tempData[5] * intersection.z + tempData[6];
		double y = 2 * tempData[1] * intersection.y + tempData[3] * intersection.x +
			tempData[4] * intersection.z + tempData[7];
		double z = 2 * tempData[2] * intersection.z + tempData[4] * intersection.y +
			tempData[5] * intersection.x + tempData[8];
		Vector3 tempn(x, y, z);
		if (tempn.Dot(tempDirection) > 0.0)
			tempn.Set(-x, -y, -z);

		intersection = translate_matrix_[0] * rotat_matrix_[0] * intersection;
		normal = rotat_matrix_[0] * tempn; // 更新方向
		isIntersect = true;
	}

	bool RTAnalysis::RayCurvedSurface(
		const std::vector<double> &a,
		Vector3 n,
		Vector3 org, 
		double & t, 
		Vector3 & interPoint)
	{
		double x0 = org.x, y0 = org.y, z0 = org.z;
		double x1 = n.x, y1 = n.y, z1 = n.z;

		double A = a[0] * x1 * x1 + a[1] * y1 * y1 + a[2] * z1 * z1 + a[3] * x1 * y1 +
			a[4] * z1 * y1 + a[5] * x1 * z1;
		double B = 2 * a[0] * x1 * x0 + 2 * a[1] * y1 * y0 + 2 * a[2] * z1 * z0 +
			a[3] * (x0 * y1 + x1 * y0) + a[4] * (z0 * y1 + z1 * y0) + a[5] * (z0 * x1 + z1 * x0) +
			a[6] * x1 + a[7] * y1 + a[8] * z1;
		double C = a[0] * x0 * x0 + a[1] * y0 * y0 + a[2] * z0 * z0 +
			a[3] * x0 * y0 + a[4] * z0 * y0 + a[5] * x0 * z0 +
			a[6] * x0 + a[7] * y0 + a[8] * z0 + a[9];

		if (A < -THRESHOLD_RAY || A > THRESHOLD_RAY)
		{
			double temp = B * B - 4 * A * C;
			if (temp >= 0)
				temp = pow(temp, 0.5);
			else
				return false;

			double tempt1, tempt2;
			tempt1 = (-B + temp) / 2.0 / A;
			tempt2 = (-B - temp) / 2.0 / A; // 求根公式的两个解

			if (tempt1 >= 0.0 && tempt2 >= 0.0) // 都大于等于0 需要先比较是否在都在限制条件内
			{
				bool isIn1 = false;
				Vector3 interPoint1(x0 + x1 * tempt1, y0 + y1 * tempt1, z0 + z1 * tempt1);
				// 判断是否在给出的区间内
				if (a[10] - THRESHOLD_RAY < interPoint1.x  &&
					interPoint1.x < a[11] + THRESHOLD_RAY &&
					a[12] - THRESHOLD_RAY < interPoint1.y &&
					interPoint1.y < a[13] + THRESHOLD_RAY &&
					a[14] - THRESHOLD_RAY < interPoint1.z &&
					interPoint1.z < a[15] + THRESHOLD_RAY)
				{
					isIn1 = true;
				}
				bool isIn2 = false;
				Vector3 interPoint2(x0 + x1 * tempt2, y0 + y1 * tempt2, z0 + z1 * tempt2);
				// 判断是否在给出的区间内
				if (a[10] - THRESHOLD_RAY < interPoint2.x  &&
					interPoint2.x < a[11] + THRESHOLD_RAY &&
					a[12] - THRESHOLD_RAY < interPoint2.y &&
					interPoint2.y < a[13] + THRESHOLD_RAY &&
					a[14] - THRESHOLD_RAY < interPoint2.z &&
					interPoint2.z < a[15] + THRESHOLD_RAY)
				{
					isIn2 = true;
				}
				if (isIn1 && isIn2)
				{
					if (tempt1 > tempt2)
					{
						interPoint = interPoint1;
						return true;
					}
					else
					{
						interPoint = interPoint2;
						return true;
					}

				}
				else if (isIn1 && !isIn2)
				{
					interPoint = interPoint1;
					return true;
				}
				else if (!isIn1 && isIn2)
				{
					interPoint = interPoint2;
					return true;
				}
				else
				{
					return false;
				}

			}
			else if (tempt1 < 0.0 && tempt2 < 0.0) // 都小于0 无解
			{
				return false;
			}
			else                           // 取正值
			{
				if (tempt1 < tempt2)
					t = tempt2;
				else
					t = tempt1;
			}
		}
		else                          // 只有一个交点，与法线平行
			t = -C / B;

		if (t < 0.0)
			return false;
		else
		{
			// 判断是否在给出的区间内
			interPoint.Set(x0 + x1 * t, y0 + y1 * t, z0 + z1 * t);
			if (a[10] - THRESHOLD_RAY < interPoint.x  &&
				interPoint.x < a[11] + THRESHOLD_RAY &&
				a[12] - THRESHOLD_RAY < interPoint.y &&
				interPoint.y < a[13] + THRESHOLD_RAY &&
				a[14] - THRESHOLD_RAY < interPoint.z &&
				interPoint.z < a[15] + THRESHOLD_RAY)
			{
				return true;
			}
			else
				return false;
		}
	}

	void RTAnalysis::RayCurvedSurface(
		const std::vector<double>& a,
		Vector3 n, 
		Vector3 org, 
		double & t1, 
		double & t2, 
		bool & isOk1,
		bool & isOk2,
		Vector3 &interPoint1,
		Vector3 &interPoint2)
	{
		double x0 = org.x, y0 = org.y, z0 = org.z;
		double x1 = n.x, y1 = n.y, z1 = n.z;

		double A = a[0] * x1 * x1 + a[1] * y1 * y1 + a[2] * z1 * z1 + a[3] * x1 * y1 +
			a[4] * z1 * y1 + a[5] * x1 * z1;
		double B = 2 * a[0] * x1 * x0 + 2 * a[1] * y1 * y0 + 2 * a[2] * z1 * z0 +
			a[3] * (x0 * y1 + x1 * y0) + a[4] * (z0 * y1 + z1 * y0) + a[5] * (z0 * x1 + z1 * x0) +
			a[6] * x1 + a[7] * y1 + a[8] * z1;
		double C = a[0] * x0 * x0 + a[1] * y0 * y0 + a[2] * z0 * z0 +
			a[3] * x0 * y0 + a[4] * z0 * y0 + a[5] * x0 * z0 +
			a[6] * x0 + a[7] * y0 + a[8] * z0 + a[9];

		if (A < -THRESHOLD_RAY || A > THRESHOLD_RAY)
		{
			double temp = B * B - 4 * A * C;
			if (temp >= 0)
				temp = pow(temp, 0.5);
			else
			{
				isOk1 = false;
				isOk2 = false;
				return;
			}
			double tempt1, tempt2;
			tempt1 = (-B + temp) / 2.0 / A;
			tempt2 = (-B - temp) / 2.0 / A; // 求根公式的两个解

			interPoint1.Set(x0 + x1 * tempt1, y0 + y1 * tempt1, z0 + z1 * tempt1);
			// 判断是否在给出的区间内
			if (a[10] - THRESHOLD_RAY < interPoint1.x  &&
				interPoint1.x < a[11] + THRESHOLD_RAY &&
				a[12] - THRESHOLD_RAY < interPoint1.y &&
				interPoint1.y < a[13] + THRESHOLD_RAY &&
				a[14] - THRESHOLD_RAY < interPoint1.z &&
				interPoint1.z < a[15] + THRESHOLD_RAY)
			{
				t1 = tempt1;
				isOk1 = true;
			}
			interPoint2.Set(x0 + x1 * tempt2, y0 + y1 * tempt2, z0 + z1 * tempt2);
			// 判断是否在给出的区间内
			if (a[10] - THRESHOLD_RAY < interPoint2.x  &&
				interPoint2.x < a[11] + THRESHOLD_RAY &&
				a[12] - THRESHOLD_RAY < interPoint2.y &&
				interPoint2.y < a[13] + THRESHOLD_RAY &&
				a[14] - THRESHOLD_RAY < interPoint2.z &&
				interPoint2.z < a[15] + THRESHOLD_RAY)
			{
				t2 = tempt2;
				isOk2 = true;
			}
		}
		else // 只有一个交点，与法线平行
		{
			isOk2 = false;
			double tempt1 = -C / B;
			interPoint1.Set(x0 + x1 * tempt1, y0 + y1 * tempt1, z0 + z1 * tempt1);
			if (a[10] - THRESHOLD_RAY < interPoint1.x  &&
				interPoint1.x < a[11] + THRESHOLD_RAY &&
				a[12] - THRESHOLD_RAY < interPoint1.y &&
				interPoint1.y < a[13] + THRESHOLD_RAY &&
				a[14] - THRESHOLD_RAY < interPoint1.z &&
				interPoint1.z < a[15] + THRESHOLD_RAY)
			{
				t1 = tempt1;
				isOk1 = true;
			}
			else
			{
				isOk1 = false;
			}
		}
	}
}
