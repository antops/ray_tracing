/*
*	created by liyun
*   function ����׷���������ӿ�
*/
#pragma once
#include <vector>
#include "../common/include/Vector3.h"
#include "../common/include/coordinate.h"
#include "../common/include/ray_line_cluster.h"
#include "../common/include/component_param.h"

using Common::Vector3;
using Common::Coordinate;
using Common::RestrictionParam;
using Common::RayLineCluster;
using Common::ComponentParam;

namespace Antops
{	
	// ��׷�㷨����
	enum TraceAlgoType {
		DefaultType,
		AnalysisType = 0,
		BaseSTL = 1, // �������stl���㣬��������
		CUDASTL = 2, // cuda���ٵ�stl����Ҫ֧��cuda
		
	};
	struct RayTracingOption
	{
		bool is_ign_non_intersection = false; // �Ƿ����û�з���ĵ�
		bool is_ign_restriction = false;	  // �Ƿ������������
		ComponentParam* param = nullptr;
		TraceAlgoType algo_type = DefaultType;
	};

	class RTBase;
	class RayTracing
	{
	public:
		explicit RayTracing(const RayTracingOption& option);

		virtual ~RayTracing();

		// ����ֱ����mirror�ཻ������������뷨�� (����PVVA)
		int CalcNormalOfLineMirror(
			const std::vector<Vector3>& startPiont, // ���
			const std::vector<Vector3>& direction,
			std::vector<Vector3> &normal,          // ����
			std::vector<Vector3> &intersection,    // ����
			std::vector<bool> &isIntersect,        // �Ƿ��ཻ
			std::vector<double>& t);               // t ֵ

		// ���㷴�䣬����RayLineCluster�����RayLineCluster (���ڹ�׷)
		int CalcReflect(const RayLineCluster & in, RayLineCluster & out);

		// ����������ߺͷ����������
		static Vector3 ReflectLight(const Vector3& a, const Vector3& n);
	private:
		RayTracingOption opt_;
		RTBase* calc_object_;
	};
}

