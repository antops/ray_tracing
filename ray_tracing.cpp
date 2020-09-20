#include "ray_tracing.h"
#include "src/RT_base.h"
#include "../common/include/MyLog.h"
namespace Antops
{
	RayTracing::RayTracing(const RayTracingOption & param)
		: opt_(param)
	{
		calc_object_ = RTBase::RTFactory(param);
	}

	RayTracing::~RayTracing()
	{
		if (calc_object_) {
			delete calc_object_;
		}
	}

	int RayTracing::CalcNormalOfLineMirror(
		const std::vector<Vector3>& startPiont,
		const std::vector<Vector3>& direction,
		std::vector<Vector3> &normal,
		std::vector<Vector3> &intersection,
		std::vector<bool> &isIntersect,
		std::vector<double>& t)
	{
		if (!calc_object_) {
			LOG(ERROR) << "Ray Tracing object is null";
			return -1;
		}
		return calc_object_->CalcNormalOfLineMirror(startPiont, direction, normal, intersection, isIntersect, t);
	}

	int RayTracing::CalcReflect(const RayLineCluster & in, RayLineCluster & out)
	{
		if (!calc_object_) {
			LOG(ERROR) << "Ray Tracing object is null";
			return -1;
		}
		
		return calc_object_->CalcReflect(in, out);
	}

	Vector3 RayTracing::ReflectLight(const Vector3 & a, const Vector3 & n)
	{
		Vector3 tempN = n;
		if (a.Dot(n) > 0)
			tempN = Vector3(0, 0, 0) - n;
		//先单位化
		double absa = pow(a.Dot(a), 0.5);
		double absn = pow(tempN.Dot(tempN), 0.5);
		Vector3 tempa = a * (1 / absa);
		Vector3 tempn = tempN * (1 / absn);
		double I = 2 * tempn.Dot(tempa);
		if (I < 0)
			I = -I;
		else
			tempa = Vector3(0.0, 0.0, 0.0) - tempa;

		return tempn * I + tempa;
	}

}
