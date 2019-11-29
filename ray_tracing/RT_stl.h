#pragma once
#include <vector>
#include "RT_base.h"

namespace Antops
{
	class RTSTL : public RTBase {
	public:
		RTSTL();
		virtual ~RTSTL();
		bool Init(const RayTracingOption& opt);
		virtual int CalcReflect(const RayLineCluster & in, RayLineCluster & out);
	private:
		// 判断三角形和射线是否有交点
		bool IsIntersect(const Vector3 & orig, const Vector3 & dir,
			const Vector3 & v0, const Vector3 & v1, const Vector3 & v2,
			Vector3 & intersection, double & t);
	};
}