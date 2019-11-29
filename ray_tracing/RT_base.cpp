#include "RT_base.h"
#include "RT_analysis.h"
#include "RT_stl.h"
#include <memory>
#include "../common/common/include/Vector3D.h"

using Common::Vector3D;
namespace Antops
{
	RTBase::RTBase()
	{
	}
	RTBase::~RTBase()
	{
	}

	RTBase * RTBase::RTFactory(const RayTracingOption & opt)
	{
		std::unique_ptr<RTBase> ptr;
		if (!opt.param.empty()) {
			// 解析
			ptr.reset(new RTAnalysis());
		}
		else if (opt.data) {
			ptr.reset(new RTSTL());
		}
		if (ptr->Init(opt)) return ptr.release();
		return nullptr;
	}

	void RTBase::Prepare()
	{
		CalcMatrix();
	}

	void RTBase::CalcMatrix()
	{
		if (calcmatrix_flag_) return;
		const auto& restriction = opt_.restriction;
		int num = restriction.size() + 1;
		R_translate_matrix_.resize(num);
		R_rotat_matrix_.resize(num);
		rotat_matrix_.resize(num);
		translate_matrix_.resize(num);

		// 世界坐标系转到模型的相对坐标系矩阵（逆矩阵）先旋转后平移
		const auto& coor = opt_.coor;
		Vector3D RotateAsix(coor.rotate_axis.x, coor.rotate_axis.y, coor.rotate_axis.z);
		R_rotat_matrix_[0] = Matrix4D::getRotateMatrix(-coor.rotate_theta, RotateAsix);
		Vector3D rotatTranslate(coor.pos.x, coor.pos.y, coor.pos.z);
		rotatTranslate = R_rotat_matrix_[0] * rotatTranslate; // 先旋转在平移（把平移的坐标一起旋转）
		R_translate_matrix_[0] = Matrix4D::getTranslateMatrix(rotatTranslate * (-1));

		// 模型的相对坐标系转到世界坐标矩阵
		rotat_matrix_[0] = Matrix4D::getRotateMatrix(coor.rotate_theta, RotateAsix);
		translate_matrix_[0] = Matrix4D::getTranslateMatrix(coor.pos.x, coor.pos.y, coor.pos.z);


		for (int i = 1; i < num; ++i)
		{
			const auto& rcoor = restriction[i - 1].coor;

			// 世界坐标系转到模型的相对坐标系矩阵（逆矩阵）先旋转后平移
			Vector3D RotateAsix(rcoor.rotate_axis.x, rcoor.rotate_axis.y, rcoor.rotate_axis.z);
			R_rotat_matrix_[i] = Matrix4D::getRotateMatrix(-rcoor.rotate_theta, RotateAsix);
			Vector3D rotatTranslate(rcoor.pos.x, rcoor.pos.y, rcoor.pos.z);
			rotatTranslate = R_rotat_matrix_[i] * rotatTranslate; // 先旋转在平移（把平移的坐标一起旋转）
			R_translate_matrix_[i] = Matrix4D::getTranslateMatrix(rotatTranslate * (-1));

			// 模型的相对坐标系转到世界坐标矩阵
			rotat_matrix_[i] = Matrix4D::getRotateMatrix(rcoor.rotate_theta, RotateAsix);
			translate_matrix_[i] = Matrix4D::getTranslateMatrix(rcoor.pos.x, rcoor.pos.y, rcoor.pos.z);
		}

		calcmatrix_flag_ = true;
		return;
	}

	bool RTBase::IsInRestriction(const Vector3 & intersectionGlobal)
	{
		if (opt_.is_ign_restriction) return true;
		const auto& restriction = opt_.restriction;
		for (size_t i = 0; i < restriction.size(); i++)
		{
			const vector<double>& tempRestrictionData = restriction[i].param;
			// 将世界坐标系转到限制条件的相对坐标系
			if (tempRestrictionData.size() < 4) continue;

			Vector3 intersectionLocal = R_translate_matrix_[i + 1] *
				R_rotat_matrix_[i + 1] * intersectionGlobal;
			if (intersectionLocal.z > tempRestrictionData[1] ||
				intersectionLocal.z < 0.0)
			{
				return false;
			}
			if (restriction[i].type == 0)
			{
				double tempRadius = intersectionLocal.x * intersectionLocal.x +
					intersectionLocal.y * intersectionLocal.y;

				if (tempRadius > tempRestrictionData[0] * tempRestrictionData[0])
				{
					return false;
				}
			}
			else
			{
				if (abs(intersectionLocal.x) > tempRestrictionData[0] / 2 ||
					abs(intersectionLocal.y) > tempRestrictionData[0] / 2)
					return false;
			}

		}
		return true;
	}
}
