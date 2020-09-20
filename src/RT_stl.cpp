#include "RT_stl.h"
#include "../ray_tracing.h"
#include <vtkPolyData.h>
namespace Antops
{
	RTSTL::RTSTL()
	{
	}

	RTSTL::~RTSTL()
	{
	}

	bool RTSTL::Init(const RayTracingOption & opt)
	{
		opt_ = opt;
		return true;
	}

	int RTSTL::CalcReflect(const RayLineCluster & in, RayLineCluster & out)
	{
		return 0;
	}

	int RTSTL::CalcNormalOfLineMirror(
		const std::vector<Vector3>& startPiont,
		const std::vector<Vector3>& direction,
		std::vector<Vector3> &normal,
		std::vector<Vector3> &intersection,
		std::vector<bool> &isIntersect,
		std::vector<double>& t)
	{
		return 0;
	}

	bool RTSTL::IsIntersect(
		const Vector3 & orig, 
		const Vector3 & dir,
		const Vector3 & v0,
		const Vector3 & v1, 
		const Vector3 & v2, 
		Vector3 & intersection,
		double & t)
	{
		double u, v;
		// E1
		Vector3 E1 = v1 - v0;

		// E2
		Vector3 E2 = v2 - v0;

		// P
		Vector3 P = dir.Cross(E2);

		// determinant
		double det = E1.Dot(P);

		Vector3 T;
		T = orig - v0;

		// If determinant is near zero, ray lies in plane of triangle
		//if (det < 0.00000001 && det > -0.00000001)
		//	return false;

		// Calculate u and make sure u <= 1
		u = T.Dot(P);
		double fInvDet = 1.0f / det;
		u *= fInvDet;

		if (u < 0.0 || u > 1)
			return false;

		// Q
		Vector3 Q = T.Cross(E1);

		// Calculate v and make sure u + v <= 1
		v = dir.Dot(Q);
		v *= fInvDet;
		if (v < 0.0 || u + v > 1)
			return false;

		// Calculate t, scale parameters, ray intersects triangle
		t = E2.Dot(Q);
		t *= fInvDet;

		intersection = orig + dir * t;

		return true;
	}

	void RTSTL::CalcReflectByPolyData(
		const Vector3 & startPiont,
		const Vector3 & direction,
		Vector3 & reflex,
		Vector3 & intersection,
		bool & isIntersect)
	{
		vtkPolyData* polyData = (vtkPolyData*)opt_.param->data_ptr;
		int EleNum = polyData->GetNumberOfCells();
		double t;
		for (int i = 0; i < EleNum; i++)  //求与反射面的交点
		{
			vtkIdList * p;
			p = polyData->GetCell(i)->GetPointIds();
			double * point;
			point = polyData->GetPoint(p->GetId(0));
			Vector3 NodesXYZ1(point[0], point[1], point[2]);
			point = polyData->GetPoint(p->GetId(1));
			Vector3 NodesXYZ2(point[0], point[1], point[2]);
			point = polyData->GetPoint(p->GetId(2));
			Vector3 NodesXYZ3(point[0], point[1], point[2]);
			if (this->IsIntersect(startPiont, direction, NodesXYZ1,
				NodesXYZ2, NodesXYZ3, intersection, t))
			{
				if (t >= 0)
				{
					if (!IsInRestriction(intersection)) //不满足限制条件
					{
						intersection = startPiont; // 让交点等于起点 方向不变 避免对无交点时特殊处理
						isIntersect = false;
						return;
					}
					Vector3 tempa = NodesXYZ1 - NodesXYZ2;
					Vector3 tempb = NodesXYZ1 - NodesXYZ3;
					Vector3 n_light = tempa.Cross(tempb);  //法向量

					isIntersect = true;
					reflex = RayTracing::ReflectLight(direction, n_light);
					return;
				}
			}
		}
		isIntersect = false;
	}
}
