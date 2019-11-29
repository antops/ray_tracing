#include "ray_tracing_stl_index.h"
#include <iostream>
namespace TBTBack
{
	RayTracingSTLIndex::RayTracingSTLIndex()
	{
	}

	RayTracingSTLIndex::~RayTracingSTLIndex()
	{
	}

	void RayTracingSTLIndex::prepare()
	{
		RayTracing::prepare();
		BuildBox();
		SplitBox();
		BuildIndex();
	}

	void RayTracingSTLIndex::BuildBox()
	{
		int EleNum = polyData->GetNumberOfCells();
		// 建立box
		vtkIdList * p;
		p = polyData->GetCell(0)->GetPointIds();
		double * point;
		point = polyData->GetPoint(p->GetId(0));
		x_max_ = point[0];
		x_min_ = point[0];
		y_max_ = point[1];
		y_min_ = point[1];
		z_max_ = point[2];
		z_min_ = point[2];
		for (int i = 0; i < EleNum; i++)
		{
			vtkIdList * p;
			p = polyData->GetCell(i)->GetPointIds();
			double * point;
			point = polyData->GetPoint(p->GetId(0));
			if (point[0] > x_max_) x_max_ = point[0];
			if (point[0] < x_min_) x_min_ = point[0];

			if (point[1] > y_max_) y_max_ = point[1];
			if (point[1] < y_min_) y_min_ = point[1];

			if (point[2] > z_max_) z_max_ = point[2];
			if (point[2] < z_min_) z_min_ = point[2];

			point = polyData->GetPoint(p->GetId(1));
			if (point[0] > x_max_) x_max_ = point[0];
			if (point[0] < x_min_) x_min_ = point[0];

			if (point[1] > y_max_) y_max_ = point[1];
			if (point[1] < y_min_) y_min_ = point[1];

			if (point[2] > z_max_) z_max_ = point[2];
			if (point[2] < z_min_) z_min_ = point[2];

			point = polyData->GetPoint(p->GetId(2));
			if (point[0] > x_max_) x_max_ = point[0];
			if (point[0] < x_min_) x_min_ = point[0];

			if (point[1] > y_max_) y_max_ = point[1];
			if (point[1] < y_min_) y_min_ = point[1];

			if (point[2] > z_max_) z_max_ = point[2];
			if (point[2] < z_min_) z_min_ = point[2];
		}
	}

	void RayTracingSTLIndex::SplitBox()
	{
		// 划分box
		double gap_x_Max = x_max_ - x_min_;
		double gap_y_Max = y_max_ - y_min_;
		double gap_z_Max = z_max_ - z_min_;

		double tmpMax = gap_x_Max;
		if (tmpMax < gap_y_Max) tmpMax = gap_y_Max;
		if (tmpMax < gap_z_Max) tmpMax = gap_z_Max;

		double gap = tmpMax / 10;

		x_num_ = gap_x_Max / gap + 0.5;
		y_num_ = gap_y_Max / gap + 0.5;
		z_num_ = gap_z_Max / gap + 0.5;

		x_gap_ = gap_x_Max / double(x_num_);
		y_gap_ = gap_y_Max / double(y_num_);
		z_gap_ = gap_z_Max / double(z_num_);

		matrix_index_.resize(x_num_);
		for (size_t i = 0; i < x_num_; i++) {
			matrix_index_[i].resize(y_num_);
			for (size_t j = 0; j < y_num_; j++) {
				matrix_index_[i][j].resize(z_num_);
			}
		}
	}

	void RayTracingSTLIndex::BuildIndex()
	{
		// 建立index
		int EleNum = polyData->GetNumberOfCells();
		for (int i = 0; i < EleNum; i++) {
			vtkIdList * p;
			p = polyData->GetCell(i)->GetPointIds();
			double * point;
			point = polyData->GetPoint(p->GetId(0));
			int x_index = (point[0] - x_min_) / x_gap_;
			int y_index = (point[1] - y_min_) / y_gap_;
			int z_index = (point[2] - z_min_) / z_gap_;
			if (x_index >= x_num_) x_index = x_num_ - 1;
			if (y_index >= y_num_) y_index = y_num_ - 1;
			if (z_index >= z_num_) z_index = z_num_ - 1;

			matrix_index_[x_index][y_index][z_index].insert(i);

			point = polyData->GetPoint(p->GetId(1));
			x_index = (point[0] - x_min_) / x_gap_;
			y_index = (point[1] - y_min_) / y_gap_;
			z_index = (point[2] - z_min_) / z_gap_;
			if (x_index >= x_num_) x_index = x_num_ - 1;
			if (y_index >= y_num_) y_index = y_num_ - 1;
			if (z_index >= z_num_) z_index = z_num_ - 1;

			matrix_index_[x_index][y_index][z_index].insert(i);

			point = polyData->GetPoint(p->GetId(2));
			x_index = (point[0] - x_min_) / x_gap_;
			y_index = (point[1] - y_min_) / y_gap_;
			z_index = (point[2] - z_min_) / z_gap_;
			if (x_index >= x_num_) x_index = x_num_ - 1;
			if (y_index >= y_num_) y_index = y_num_ - 1;
			if (z_index >= z_num_) z_index = z_num_ - 1;

			matrix_index_[x_index][y_index][z_index].insert(i);
		}
	}
	 
	void RayTracingSTLIndex::calcReflectByPolyData(
		const Vector3 & startPiont,
		const Vector3 & direction,
		Vector3 & reflex,
		Vector3 & intersection,
		bool & isIntersect)
	{
		isIntersect = false;
		Vector3 p1, p2;
		if (!IsInBox(startPiont, direction, p1, p2)) return;

		std::set<Index> grid_set;
		FindAllGrid(p1, p2, grid_set);
		for (const auto& index : grid_set) {
			for (const auto& num : matrix_index_[index.x][index.y][index.z]) {
				vtkIdList * p;
				p = polyData->GetCell(num)->GetPointIds();
				double * point;
				point = polyData->GetPoint(p->GetId(0));
				Vector3 NodesXYZ1(point[0], point[1], point[2]);
				point = polyData->GetPoint(p->GetId(1));
				Vector3 NodesXYZ2(point[0], point[1], point[2]);
				point = polyData->GetPoint(p->GetId(2));
				Vector3 NodesXYZ3(point[0], point[1], point[2]);
				double t = 0.0;
				if (this->isIntersect(startPiont, direction, NodesXYZ1,
					NodesXYZ2, NodesXYZ3, intersection, t)) {
					if (t >= 0)
					{
						if (!isInRestriction(intersection)) //不满足限制条件
						{
							intersection = startPiont; // 让交点等于起点 方向不变 避免对无交点时特殊处理
							isIntersect = false;
							return;
						}
						Vector3 tempa = NodesXYZ1 - NodesXYZ2;
						Vector3 tempb = NodesXYZ1 - NodesXYZ3;
						Vector3 n_light = tempa.Cross(tempb);  //法向量

						isIntersect = true;
						reflex = reflectLight(direction, n_light);
						return;
					}
				}
			}
		}
		return;
	}

	bool RayTracingSTLIndex::IsInBox(
		const Vector3 & startPiont,
		const Vector3 & direction,
		Vector3 & p1,
		Vector3 & p2)
	{
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
		std::vector<Vector3> inter_point;
		if (abs(direction.x) > 0.000000001) 
		{
			// 左
			double t = (x_min_ - startPiont.x) / direction.x;
			y = startPiont.y + t * direction.y;
			if (y < y_max_ && y > y_min_)
			{
				z = startPiont.z + t * direction.z;
				if (z < z_max_ && z > z_min_)
				{
					inter_point.push_back(Vector3(x_min_, y, z));
				}
			}
			// 右
			t = (x_max_ - startPiont.x) / direction.x;
			y = startPiont.y + t * direction.y;
			if (y < y_max_ && y > y_min_)
			{
				z = startPiont.z + t * direction.z;
				if (z < z_max_ && z > z_min_)
				{
					inter_point.push_back(Vector3(x_max_, y, z));
				}
			}
		}
		if (abs(direction.y) > 0.000000001)
		{
			// 前
			double t = (y_min_ - startPiont.y) / direction.y;
			x = startPiont.x + t * direction.x;
			if (x < x_max_ && x > x_min_)
			{
				z = startPiont.z + t * direction.z;
				if (z < z_max_ && z > z_min_)
				{
					inter_point.push_back(Vector3(x, y_min_, z));
				}
			}
			// 后
			t = (y_max_ - startPiont.y) / direction.y;
			x = startPiont.x + t * direction.x;
			if (x < x_max_ && x > x_min_)
			{
				z = startPiont.z + t * direction.z;
				if (z < z_max_ && z > z_min_)
				{
					inter_point.push_back(Vector3(x, y_max_, z));
				}
			}
		}
		if (abs(direction.z) > 0.000000001)
		{
			// 上
			double t = (z_min_ - startPiont.z) / direction.z;
			x = startPiont.x + t * direction.x;
			if (x < x_max_ && x > x_min_)
			{
				y = startPiont.y + t * direction.y;
				if (y < y_max_ && y > y_min_)
				{
					inter_point.push_back(Vector3(x, y, z_min_));
				}
			}
			// 下
			t = (z_max_ - startPiont.z ) / direction.z;
			x = startPiont.x + t * direction.x;
			if (x < x_max_ && x > x_min_)
			{
				y = startPiont.y + t * direction.y;
				if (y < y_max_ && y > y_min_)
				{
					inter_point.push_back(Vector3(x, y, z_max_));
				}
			}
		}
		if (inter_point.size() == 2) {
			p1 = inter_point[0];
			p2 = inter_point[1];
			return true;
		}
		return false;
	}

	void RayTracingSTLIndex::FindAllGrid(
		const Vector3 & p1, const Vector3 & p2, std::set<Index>& grid_set)
	{
		int x_begin = (p1.x - x_min_) / x_gap_;
		int x_end = (p2.x - x_min_) / x_gap_;
		if (x_begin > x_end) std::swap(x_begin, x_end);

		int y_begin = (p1.y - y_min_) / y_gap_;
		int y_end = (p2.y - y_min_) / y_gap_;
		if (y_begin > y_end) std::swap(y_begin, y_end);

		int z_begin = (p1.z - z_min_) / z_gap_;
		int z_end = (p2.z - z_min_) / z_gap_;
		if (z_begin > z_end) std::swap(z_begin, z_end);

		double x = 0.0;
		double y = 0.0;
		double z = 0.0;

		double p2p1x = p2.x - p1.x;
		double p2p1y = p2.y - p1.y;
		double p2p1z = p2.z - p1.z;
		for (size_t i = x_begin; i < x_end; i++) {
			double t = (i * x_gap_ - p1.x) / p2p1x;
			if (t > 1.0) continue;
			y = p1.y + p2p1y * t;
			z = p1.z + p2p1z * t;
			InsertIndex(i, (y - y_min_) / y_gap_, (z - z_min_) / z_gap_, grid_set);
		}

		for (size_t i = y_begin; i < y_end; i++) {
			double t = (i * y_gap_ + y_min_ - p1.y) / p2p1y;
			if (t > 1.0) continue;
			x = p1.x + p2p1x * t;
			z = p1.z + p2p1z * t;
			InsertIndex((x - x_min_) / x_gap_, i, (z - z_min_) / z_gap_, grid_set);
		}

		for (size_t i = z_begin; i < z_end; i++) {
			double t = (i * z_gap_ - p1.z) / p2p1z;
			if (t > 1.0) continue;
			x = p1.x + p2p1x * t;
			y = p1.y + p2p1y * t;
			InsertIndex((x - x_min_) / x_gap_, (y - y_min_) / y_gap_, i, grid_set);
		}
	}
	void RayTracingSTLIndex::InsertIndex(size_t x, size_t y, size_t z, std::set<Index>& grid_set)
	{
		if (x >= x_num_) x = x_num_ - 1;
		if (y >= y_num_) y = y_num_ - 1;
		if (z >= z_num_) z = z_num_ - 1;
		grid_set.insert(Index(x,y,z));
	}
}
