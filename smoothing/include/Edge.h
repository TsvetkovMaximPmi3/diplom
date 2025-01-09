#pragma once
#include <vector>

#include <mb_cart_point3d.h>

struct Edge {
	// вернуть первую точку ребра
	int pt1() const;
	// вернуть последнюю точку ребра
	int pt2() const;
	// id ячеек в которые включено ребро
	std::vector<int> includedCells;
	// флаг показывающий является ли ребро диагональным
	bool diag;
	// id точек включенных в ребро
	std::vector<int> idPointsIn;
	// "перевернуть" ребро
	void reverseEdge();
	Edge(int _pt1, int _pt2);
	// флаг показывающий пересекается ли ребро
	bool intersection;
	// напряжение на ребре(зависит от длины векторов перемещения точек ребра)
	double napryazhenie;
	MbVector3D tmpVecPt1;
	MbVector3D tmpVecPt2;

	Edge() {}
};