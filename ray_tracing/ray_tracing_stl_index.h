#pragma once
#include <vector>
#include <set>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include "../../TBT_common/Vector3.h"
#include "RayTracing.h"

namespace TBTBack
{
	class RayTracingSTLIndex : public RayTracing
	{
	public:
		RayTracingSTLIndex();
		virtual ~RayTracingSTLIndex();

	protected:
		virtual void prepare();

		// ����ģ���ʷ����ݼ��㷴��
		// ������������Ԫ�ཻ�ж�
		// ���ҵ����߾�����cube
		// ����cube�ڶ�Ӧ��������Ԫ
		virtual void calcReflectByPolyData(
			const Vector3 & startPiont,
			const Vector3 & direction,
			Vector3 & reflex,
			Vector3 & intersection,
			bool & isIntersect);
	private:
		struct Index {
			Index(
				size_t _x,
				size_t _y,
				size_t _z) :
				x(_x), y(_y), z(_z) {};
			size_t x;
			size_t y;
			size_t z;
			bool operator < (const Index& right) const
			{
				if (x == right.x) {
					if (y == right.y)
						return z < right.z;
					else
						return y < right.y;
				}
				else
					return x < right.x;
			}
		};
		// ����box
		void BuildBox();

		// ����box
		void SplitBox();

		// ����index
		void BuildIndex();

		// �ж��Ƿ��ں�����
		// ����ڣ���������ӵ���������
		bool IsInBox(
			const Vector3 & startPiont,
			const Vector3 & direction,
			Vector3 & p1,
			Vector3 & p2);

		// �ҵ�ֱ�߾��������и��
		void FindAllGrid(
			const Vector3 & p1,
			const Vector3 & p2,
			std::set<Index>& grid_set);

		void InsertIndex(size_t x, size_t y, size_t z, std::set<Index>& grid_set);

	private:
		// x, y, z��Ӧ��������Ԫ��index
		std::vector<std::vector<std::vector<std::set<int>>>> matrix_index_;
		double x_max_;
		double x_min_;
		double y_max_;
		double y_min_;
		double z_max_;
		double z_min_;

		size_t x_num_ = 0;
		size_t y_num_ = 0;
		size_t z_num_ = 0;

		double x_gap_;
		double y_gap_;
		double z_gap_;
	};
}