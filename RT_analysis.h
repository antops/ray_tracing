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
		virtual int CalcNormalOfLineMirror(
			const std::vector<Vector3>& startPiont,
			const std::vector<Vector3>& direction,
			std::vector<Vector3> &normal,
			std::vector<Vector3> &intersection,
			std::vector<bool> &isIntersect,
			std::vector<double>& t);
	private:
		// 直线与面相交
		bool RayCurvedSurface(
			const std::vector<double> &a,
			Vector3 n, 
			Vector3 org,
			double & t,
			Vector3 & interPoint);
		void CalcNormalOfLineMirrorByQuadricSurface(
			const Vector3 & startPiont, 
			const Vector3 & direction,
			Vector3 & normal,
			Vector3 & intersection, 
			bool & isIntersect,
			double & t);
		void CalcReflectByQuadricSurface(
			const Vector3 & startPiont,
			const Vector3 & direction,
			Vector3 & reflex,
			Vector3 & intersection,
			bool & isIntersect);

		void RayCurvedSurface(
			const std::vector<double>& a,
			Vector3 n, 
			Vector3 org, 
			double & t1, 
			double & t2,
			bool & isOk1, 
			bool & isOk2,
			Vector3 &interPoint1, 
			Vector3 &interPoint2)

	};
}