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
		virtual int CalcNormalOfLineMirror(
			const std::vector<Vector3>& startPiont,
			const std::vector<Vector3>& direction,
			std::vector<Vector3> &normal,
			std::vector<Vector3> &intersection,
			std::vector<bool> &isIntersect,
			std::vector<double>& t);
	protected:
		// 判断三角形和射线是否有交点
		bool IsIntersect(
			const Vector3 & orig, 
			const Vector3 & dir,
			const Vector3 & v0,
			const Vector3 & v1, 
			const Vector3 & v2,
			Vector3 & intersection,
			double & t);

		void CalcReflectByPolyData(
			const Vector3 & startPiont,
			const Vector3 & direction,
			Vector3 & reflex,
			Vector3 & intersection,
			bool & isIntersect);
	};
}