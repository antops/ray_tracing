#include "RT_stl.h"

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
		return false;
	}

	int RTSTL::CalcReflect(const RayLineCluster & in, RayLineCluster & out)
	{
		return 0;
	}

	bool RTSTL::IsIntersect(const Vector3 & orig, const Vector3 & dir,
		const Vector3 & v0, const Vector3 & v1, const Vector3 & v2, 
		Vector3 & intersection, double & t)
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
}
