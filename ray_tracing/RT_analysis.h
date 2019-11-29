#pragma once
#include <vector>
#include "RT_base.h"

namespace Antops
{
	class RTAnalysis : public RTBase {
	public:
		RTAnalysis();
		virtual ~RTAnalysis();
		bool Init(const RayTracingOption& opt);
		virtual int CalcReflect(const RayLineCluster & in, RayLineCluster & out);
	private:
		// 直线与面相交
		bool RayCurvedSurface(const std::vector<double> &a,
			Vector3 n, Vector3 org, double & t, Vector3 & interPoint);
		void CalcReflectByQuadricSurface(
			const Vector3 & startPiont,
			const Vector3 & direction,
			Vector3 & reflex,
			Vector3 & intersection,
			bool & isIntersect);

	};
}