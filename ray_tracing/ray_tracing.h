/*
*	created by liyun
*   function 光线追踪引擎对外接口
*/
#pragma once
#include <vector>
#include "../common/common/include/Vector3.h"
#include "../common/common/include/coordinate.h"
#include "../common/common/include/component_param.h"

using Common::Vector3;
using Common::Coordinate;
using Common::RestrictionParam;

namespace Antops
{
	struct RayTracingOption
	{
		bool is_ign_non_intersection = false; // 是否忽略没有反射的点
		bool is_ign_restriction = false;	  // 是否忽略限制条件
		bool is_use_cuda = true;              // 是否使用cuda加速
		void* data = nullptr;                 // stl 指针，目前使用vtk
		//ComponentParam param;
		std::vector<double> param;
		Coordinate coor;
		std::vector<RestrictionParam> restriction;
	};

	struct RayLineCluster
	{
		std::vector<Vector3> start_point;  // 起点
		std::vector<Vector3> normal_line;  // 法线
	};
	class RTBase;
	class RayTracing
	{
	public:
		RayTracing(const RayTracingOption& param);

		virtual ~RayTracing();

		// 计算直线与mirror相交，并输出交点与法线 (用于PVVA)
		int CalcNormalOfLineMirror(
			const std::vector<Vector3> &startPiont,  // 起点
			const std::vector<Vector3> &normal,		 // 法线
			std::vector<Vector3> &intersection,		 // 交点
			std::vector<bool> &isIntersect,			 // 是否相交
			std::vector<double> &t);				 //	t 值

		// 计算反射，输入RayLineCluster，输出RayLineCluster (用于光追)
		int CalcReflect(const RayLineCluster & in, RayLineCluster & out);

		// 根据入射光线和法线求反射光线
		static Vector3 ReflectLight(const Vector3& a, const Vector3& n);
	private:
		RayTracingOption opt_;
		RTBase* calc_object_;
	};
}

