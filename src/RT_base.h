#pragma once
#include <vector>
#include "../ray_tracing.h"
#include "../../common/include/Vector3.h"
#include "../../common/include/Matrix4D.h"
using Common::Vector3;
using Common::Matrix4D;

namespace Antops
{
	class RTBase {
	public:
		RTBase();
		virtual ~RTBase();
		
		virtual bool Init(const RayTracingOption& opt) = 0;
		virtual int CalcReflect(const RayLineCluster & in, RayLineCluster & out) = 0;
		virtual int CalcNormalOfLineMirror(
			const std::vector<Vector3>& startPiont,
			const std::vector<Vector3>& direction,
			std::vector<Vector3> &normal,
			std::vector<Vector3> &intersection,
			std::vector<bool> &isIntersect,
			std::vector<double>& t) = 0;

		static RTBase* RTFactory(const RayTracingOption& opt);

	protected:
		void Prepare();
		void CalcMatrix();
		bool IsInRestriction(const Vector3 &intersectionGlobal);
	protected:
		RayTracingOption opt_;

		bool calcmatrix_flag_ = false;
		// 世界坐标系转到模型的相对坐标系矩阵（逆矩阵）先旋转后平移
		std::vector<Matrix4D> R_rotat_matrix_;
		std::vector<Matrix4D> R_translate_matrix_;

		// 模型的相对坐标系转到世界坐标矩阵
		std::vector<Matrix4D> rotat_matrix_;
		std::vector<Matrix4D> translate_matrix_;

	};

}