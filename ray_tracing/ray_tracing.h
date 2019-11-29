/*
*	created by liyun
*   function ����׷���������ӿ�
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
		bool is_ign_non_intersection = false; // �Ƿ����û�з���ĵ�
		bool is_ign_restriction = false;	  // �Ƿ������������
		bool is_use_cuda = true;              // �Ƿ�ʹ��cuda����
		void* data = nullptr;                 // stl ָ�룬Ŀǰʹ��vtk
		//ComponentParam param;
		std::vector<double> param;
		Coordinate coor;
		std::vector<RestrictionParam> restriction;
	};

	struct RayLineCluster
	{
		std::vector<Vector3> start_point;  // ���
		std::vector<Vector3> normal_line;  // ����
	};
	class RTBase;
	class RayTracing
	{
	public:
		RayTracing(const RayTracingOption& param);

		virtual ~RayTracing();

		// ����ֱ����mirror�ཻ������������뷨�� (����PVVA)
		int CalcNormalOfLineMirror(
			const std::vector<Vector3> &startPiont,  // ���
			const std::vector<Vector3> &normal,		 // ����
			std::vector<Vector3> &intersection,		 // ����
			std::vector<bool> &isIntersect,			 // �Ƿ��ཻ
			std::vector<double> &t);				 //	t ֵ

		// ���㷴�䣬����RayLineCluster�����RayLineCluster (���ڹ�׷)
		int CalcReflect(const RayLineCluster & in, RayLineCluster & out);

		// ����������ߺͷ����������
		static Vector3 ReflectLight(const Vector3& a, const Vector3& n);
	private:
		RayTracingOption opt_;
		RTBase* calc_object_;
	};
}

