#include "ForceSmoothing.h"

MbVector3D CalculateLinePointVec(Grid* grid, int index1, int index2, double distance) {

	MbVector3D vec;
	vec = grid->aPoints[index2].pt - grid->aPoints[index1].pt;
	vec.Normalize();
	vec *= distance;
	vec /= 2;

	return vec;
}


std::pair<MbVector3D, MbVector3D> CalculateLinePoint(Grid* grid, Edge& e, double distance) {
	MbVector3D vecPt1, vecPt2;
	//пружинки

	vecPt2 = CalculateLinePointVec(grid, e.pt2(), e.idPointsIn[e.idPointsIn.size() - 2], distance);
	vecPt1 = CalculateLinePointVec(grid, e.pt1(), e.idPointsIn[1], distance);

	if (e.tmpVecPt1 == MbVector3D(0, 0, 0) && e.tmpVecPt2 == MbVector3D(0, 0, 0)) {
		e.tmpVecPt1 = vecPt1;
		e.tmpVecPt2 = vecPt2;
	} else {
		e.tmpVecPt1 += vecPt1;
		e.tmpVecPt2 += vecPt2;
	}
	return std::pair<MbVector3D, MbVector3D>(vecPt1, vecPt2);
}


void CalculateMoveVecForPoint(Grid* grid, Edge& edge, double idealEdge) {
	double distanceToPoint;
	double lenght = 0;

	lenght = GetLenghtEdge(grid->aPoints, edge);
	if (lenght == 0)
		throw - 1;
	distanceToPoint = lenght / (edge.idPointsIn.size() - 1);

	//если distanceToPoint = 0 то это треугольник, если distanceToPoint = diagOrEdge то мы достигли идельного ребра или диагонали
	if ((distanceToPoint != idealEdge) && (distanceToPoint != 0)) {
		double distance = distanceToPoint - idealEdge;
		if (!edge.diag) distance /= 2;
		//edge.napryazhenie = distance;
		//считаем расстояние на прямой построенной через точки ptInd1,ptInd2 на которое надо сдвинуть обе точки
		std::pair<MbVector3D, MbVector3D> vecs = CalculateLinePoint(grid, edge, distance);

		if (!(grid->aPoints[edge.pt1()].flagFix)) {
			grid->aPoints[edge.pt1()].moveVect += vecs.first;

		}
		if (!(grid->aPoints[edge.pt2()].flagFix)) {
			grid->aPoints[edge.pt2()].moveVect += vecs.second;
		}
	}
}

void CalculateMoveVect(Grid* grid) {
	size_t aCellsSize = grid->aCells.size();


	for (size_t i = 0; i < aCellsSize; i++) {

		//подаем в функцию все варианты связей узлов в ячейке по id
		for (int j = 0; j < grid->aCells[i].edges.size(); j++) {
			CalculateMoveVecForPoint(grid, grid->edges[grid->aCells[i].edges[j]], grid->aCells[i].m_idealEdge);
		}
		for (int j = 0; j < grid->aCells[i].edgesDiag.size(); j++) {
			CalculateMoveVecForPoint(grid, grid->edges[grid->aCells[i].edgesDiag[j]], grid->aCells[i].m_idealDiag);
		}

		for (auto edge : grid->aCells[i].edges) {
			for (int i = 0; i < grid->edges[edge].includedCells.size(); i++) {
				edgeIntersection2(grid, grid->aCells[grid->edges[edge].includedCells[i]], edge);
			}

		}
	}
	SweepingLine(grid);

	for (int i = 0; i < grid->edges.size(); i++) {
		LineMorphing(grid, grid->edges[i]);
	}
	grid->energy = 0;

	for (std::map<int, Knot>::iterator it = grid->aPoints.begin(); it != grid->aPoints.end(); ++it) {
		grid->energy += grid->aPoints[it->first].moveVect.Length();
	}
}