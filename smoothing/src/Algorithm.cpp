#include <queue>
#include<cmath>
#include<fstream>
#include<iostream>
#include <iomanip>
#include <model.h>
#include <surf_spline_surface.h>
#include<mb_vector3d.h>
#include "Algorithm.h"
#include "Knot.h"
#include "Cell.h"
#include "RWFile.h"
#include "Grid.h"
#include "QDecFunc.h"
int iteration = 0;
int forOpenFile = 0;
struct Knot;

void edgeIntersection(Grid* grid, Cell& cell) {
	for (auto indEdge : cell.edges) {
		Edge edge = grid->edges[indEdge];
		for (int i = 0; i < edge.idPointsIn.size() - 1; i++) {
			MbCartPoint r11 = { grid->aPoints[edge.idPointsIn[i]].pt.x + grid->aPoints[edge.idPointsIn[i]].moveVect.x
				,grid->aPoints[edge.idPointsIn[i]].pt.z + grid->aPoints[edge.idPointsIn[i]].moveVect.z };
			MbCartPoint r12 = { grid->aPoints[edge.idPointsIn[i + 1]].pt.x + grid->aPoints[edge.idPointsIn[i + 1]].moveVect.x,
				grid->aPoints[edge.idPointsIn[i + 1]].pt.z + grid->aPoints[edge.idPointsIn[i + 1]].moveVect.z };
			for (auto ind2Edge : cell.edges) {
				//int ind2Edge = Inc(indEdge, 2, 4);
				if (indEdge == ind2Edge)
					continue;

				Edge edge1 = grid->edges[ind2Edge];
				for (int j = 0; j < edge1.idPointsIn.size() - 1; j++) {
					MbCartPoint r21 = { grid->aPoints[edge1.idPointsIn[j]].pt.x + grid->aPoints[edge1.idPointsIn[j]].moveVect.x,
						grid->aPoints[edge1.idPointsIn[j]].pt.z + +grid->aPoints[edge1.idPointsIn[j]].moveVect.z };
					MbCartPoint r22 = { grid->aPoints[edge1.idPointsIn[j + 1]].pt.x + grid->aPoints[edge1.idPointsIn[j + 1]].moveVect.x,
						grid->aPoints[edge1.idPointsIn[j + 1]].pt.z + grid->aPoints[edge1.idPointsIn[j + 1]].moveVect.z };
					double detA = (r11.x - r12.x)*(r22.y - r21.y) - (r11.y - r12.y)*(r22.x - r21.x);
					double detX = ((r22.y - r21.y)*(r22.x - r12.x) - (r22.x - r21.x)*(r22.y - r12.y));
					double detY = (r11.x - r12.x)* (r22.y - r12.y) - (r22.x - r12.x)*(r11.y - r12.y);

					double betta = detY / detA;
					double alpha = detX / detA;

					if (alpha > 0 && betta > 0 && alpha < 1 && betta < 1) {
						if (r11 == r21 || r11 == r22 || r12 == r22 || r12 == r21)continue;
						grid->edges[indEdge].intersection = true;
						grid->aPoints[grid->edges[indEdge].idPointsIn[i]].peresech = true;
						grid->aPoints[grid->edges[indEdge].idPointsIn[i + 1]].peresech = true;
						grid->aPoints[grid->edges[ind2Edge].idPointsIn[j]].peresech = true;
						grid->aPoints[grid->edges[ind2Edge].idPointsIn[j + 1]].peresech = true;
					}



				}
			}
		}
	}
}

int InsertInMapInEnd(Grid* grid, Knot pt) {
	auto it = grid->aPoints.end();
	it--;
	int element = it->first + 1;
	grid->aPoints.insert({ element,pt });
	return element;
}
MbVector3D CalculateLinePointVec(Grid* grid, int index1, int index2, double distance) {

	MbVector3D vec;
	vec = grid->aPoints[index2].pt - grid->aPoints[index1].pt;
	vec.Normalize();
	vec *= distance;
	vec /= 2;

	return vec;
}

size_t Inc(size_t ind, int delta, size_t size) {
	int j = (ind + delta) % size;
	return j < 0 ? (size + j) : j;
}


/**
* \brief Вычислить точку на прямой на определенной дистанции
* \param [in] pt1 Первая точка прямой
* \param [in] pt2 Вторая точка прямой
* \param [in] distance Дистанция на которой надо найти точку
* \return вектор до найденной точки
*/
std::pair<MbVector3D, MbVector3D> CalculateLinePoint(Grid* grid, Edge& e, double distance) {
	MbVector3D vecPt1, vecPt2;
	//пружинки
	//distance *= e.idPointsIn.size() - 1;

	vecPt2 = CalculateLinePointVec(grid, e.pt2(), e.idPointsIn[e.idPointsIn.size() - 2], distance);
	vecPt1 = CalculateLinePointVec(grid, e.pt1(), e.idPointsIn[1], distance);
	//if (e.diag)
	//{
	//	vecPt2 /= e.idPointsIn.size() - 1;//GetLenghtEdge(grid->aPoints, e);
	//	vecPt1 /= e.idPointsIn.size() - 1; //GetLenghtEdge(grid->aPoints,e);
	//}
	if (e.tmpVecPt1 == MbVector3D(0, 0, 0) && e.tmpVecPt2 == MbVector3D(0, 0, 0)) {
		e.tmpVecPt1 = vecPt1;
		e.tmpVecPt2 = vecPt2;
	} else {
		e.tmpVecPt1 += vecPt1;
		e.tmpVecPt2 += vecPt2;
	}
	return std::pair<MbVector3D, MbVector3D>(vecPt1, vecPt2);
}

double GetLenghtEdge(std::map<int, Knot>& aPoints, const Edge& edge) {
	double lenght = 0;
	for (int i = 0; i < edge.idPointsIn.size() - 1; i++) {
		MbVector3D vec;
		vec = aPoints[edge.idPointsIn[i]].pt - aPoints[edge.idPointsIn[i + 1]].pt;
		lenght += vec.Length();
	}
	return lenght;
}

/**
* \brief Вычислить вектор перемещения для конкретных узлов
* \param [in] grid Сетка
* \param [in] pt1Ind Индекс точки в ячейке
* \param [in] pt2Ind Индекс точки в ячейке c которой соединена первая
* \param [in] cellInd Индекс ячейки
*/
void CalculateMoveVecForPoint(Grid* grid, Edge& edge, double idealEdge) {
	double distanceToPoint;
	double lenght = 0;

	lenght = GetLenghtEdge(grid->aPoints, edge);
	if (lenght == 0)
		throw - 1;
	distanceToPoint = lenght / (edge.idPointsIn.size() - 1);
	//idealEdge *= edge.idPointsIn.size() - 1;

//если distanceToPoint = 0 то это треугольник, если distanceToPoint = diagOrEdge то мы достигли идельного ребра или диагонали
	if ((distanceToPoint != idealEdge) && (distanceToPoint != 0)) {
		double distance = distanceToPoint - idealEdge;
		if (!edge.diag) distance /= 2;
		edge.napryazhenie = distance;
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

void CheckEversion(Grid* grid, const Edge& edge) {
	MbVector3D a(grid->aPoints[edge.pt2()].pt - grid->aPoints[edge.pt1()].pt),
		F1(grid->aPoints[edge.pt2()].moveVect),
		F2(grid->aPoints[edge.pt1()].moveVect);
	//MbVector3D a(2, 0, 0);
	//MbVector3D F1(1, 1, 0);
	//MbVector3D F2(3, 3, 0);

	double k = KCRIT;

	double F1Pr = a * F1 / a.Length();
	double F2Pr = a * F2 / a.Length();

	double aPr = a.Length() + F1Pr - F2Pr;

	if (aPr / a.Length() > k)return;

	double f1 = (F1Pr + F2Pr + (k - 1)*a.Length()) / 2;
	double f2 = F1Pr + F2Pr - f1;

	MbVector3D f1New = F1*f1 / F1Pr;
	MbVector3D f2New = F2*f2 / F2Pr;
	grid->aPoints[edge.pt1()].pole = true;
	grid->aPoints[edge.pt2()].pole = true;
	//grid->aPoints[edge.pt1()].newMoveVec = new MbVector3D(f2New);
	//grid->aPoints[edge.pt2()].newMoveVec = new MbVector3D(f1New);
	if (grid->aPoints[edge.pt1()].newMoveVec == nullptr)
		grid->aPoints[edge.pt1()].newMoveVec = new MbVector3D(f2New);
	else {
		MbVector3D* vec = new MbVector3D(f2New);
		grid->aPoints[edge.pt1()].newMoveVec = vec->Length() < grid->aPoints[edge.pt1()].newMoveVec->Length() ? vec : grid->aPoints[edge.pt1()].newMoveVec;
	}
	if (grid->aPoints[edge.pt2()].newMoveVec == nullptr)
		grid->aPoints[edge.pt2()].newMoveVec = new MbVector3D(f1New);
	else {
		MbVector3D* vec = new MbVector3D(f1New);
		grid->aPoints[edge.pt2()].newMoveVec = vec->Length() < grid->aPoints[edge.pt2()].newMoveVec->Length() ? vec : grid->aPoints[edge.pt2()].newMoveVec;
	}

}

void ChangeMoveVecs(Grid* grid) {
	for (auto e : grid->edges) {
		CheckEversion(grid, e);
	}
	for (auto pt : grid->aPoints) {
		if (pt.second.newMoveVec == nullptr)continue;
		pt.second.moveVect = *pt.second.newMoveVec;
		pt.second.newMoveVec = nullptr;
	}

}
/**
* \brief Вычислить вектор перемещения для каждого узла
* \param [in, out] grid Сетка
*/
void CalculateMoveVect(Grid* grid) {
	size_t aCellsSize = grid->aCells.size();

	for (size_t i = 0; i < aCellsSize; i++) {
		//if (iteration == 1) {
		//	grid->aCells[i].m_idealDiag = (GetLenghtEdge(grid->aPoints, grid->edges[grid->aCells[i].edgesDiag[0]]) +
		//		GetLenghtEdge(grid->aPoints, grid->edges[grid->aCells[i].edgesDiag[1]])) / 2;
		//	grid->aCells[i].m_idealEdge = (GetLenghtEdge(grid->aPoints, grid->edges[grid->aCells[i].edges[0]]) +
		//		GetLenghtEdge(grid->aPoints, grid->edges[grid->aCells[i].edges[1]]) + GetLenghtEdge(grid->aPoints, grid->edges[grid->aCells[i].edges[2]]) +
		//		GetLenghtEdge(grid->aPoints, grid->edges[grid->aCells[i].edges[3]])) / 4;
		//}
		//подаем в функцию все варианты связей узлов в ячейке по id
		for (int j = 0; j < grid->aCells[i].edges.size(); j++) {
			CalculateMoveVecForPoint(grid, grid->edges[grid->aCells[i].edges[j]], grid->aCells[i].m_idealEdge);
			//LineMorphing(grid, grid->edges[grid->aCells[i]->edges[j]]);
		}
		for (int j = 0; j < grid->aCells[i].edgesDiag.size(); j++) {
			CalculateMoveVecForPoint(grid, grid->edges[grid->aCells[i].edgesDiag[j]], grid->aCells[i].m_idealDiag);
			//LineMorphing(grid, grid->edges[grid->aCells[i]->edgesDiag[j]]);
		}

		edgeIntersection(grid, grid->aCells[i]);
		for (auto edge : grid->aCells[i].edges) {
			bool flag = false;
			for (auto ind : grid->edges[edge].idPointsIn) {
				if (grid->aPoints[ind].peresech) {
					flag = true;
				}
			}
			if (flag) {
				for (auto ind : grid->edges[edge].idPointsIn) {
					grid->aPoints[ind].moveVect = MbVector3D(0, 0, 0);
					grid->aPoints[ind].peresech = false;
				}
			}
		}
	}

	//ChangeMoveVecs(grid);
	for (int i = 0; i < grid->edges.size(); i++) {
		LineMorphing(grid, grid->edges[i]);
	}
	grid->energy = 0;

	for (std::map<int, Knot>::iterator it = grid->aPoints.begin(); it != grid->aPoints.end(); ++it) {
		grid->energy += grid->aPoints[it->first].moveVect.Length();
	}
}

/**
* \brief Спроецировать 3D точки в 2D
* \param [in, out] grid Сетка
* \param [in] srfNurbs Поверхность
*/
void ProectionOn2D(Grid* grid, MbSplineSurface* srfNurbs) {

	MbCartPoint tch;

	double uMin = srfNurbs->GetUMin(),
		uMax = srfNurbs->GetUMax(),
		vMin = srfNurbs->GetVMin(),
		vMax = srfNurbs->GetVMax();

	MbCartPoint3D pt11;
	srfNurbs->PointOn(uMin, vMin, pt11);
	for (std::map<int, Knot>::iterator it = grid->aPoints.begin(); it != grid->aPoints.end(); ++it) {
		srfNurbs->NearPointProjection(it->second.pt, tch.x, tch.y, false);

		grid->aPoints2D.insert({ it->first, Knot(tch.x, tch.y) });
	}
}

/**
* \brief Вычислить новые координаты узлов в 3D
* \param [in, out] grid Сетка
* \param [in] srfNurbs Поверхность
*/
void CalculateMoveKnots(Grid* grid, MbSplineSurface* srfNurbs) {

	for (std::map<int, Knot>::iterator it = grid->aPoints.begin(); it != grid->aPoints.end(); ++it) {
		if (grid->aPoints[it->first].flagFix || grid->aPoints[it->first].moveVect == MbVector3D(0, 0, 0)) {
			continue;
		}

		MbVector3D vec = grid->speed*0.5*grid->aPoints[it->first].moveVect,
			vecU,
			vecV;

		srfNurbs->DeriveU(grid->aPoints2D[it->first].pt.x, grid->aPoints2D[it->first].pt.y, vecU);
		srfNurbs->DeriveV(grid->aPoints2D[it->first].pt.x, grid->aPoints2D[it->first].pt.y, vecV);

		//проекция вектора перемещения на вектора U V
		double u = vecU * vec / vecU.Length2();
		double v = vecV * vec / vecV.Length2();

		grid->aPoints2D[it->first].pt.x += u;
		grid->aPoints2D[it->first].pt.y += v;


		srfNurbs->PointOn(grid->aPoints2D[it->first].pt.x, grid->aPoints2D[it->first].pt.y,
						  grid->aPoints[it->first].pt);

		if (iteration != 200)

			grid->aPoints[it->first].moveVect = MbVector3D(0, 0, 0);



	}

	for (auto& cell : grid->aCells) {
		edgeIntersection(grid, cell);
		cell.InitIntervals(grid);

	}
	if (iteration != 150) {
		for (auto& edge : grid->edges) {
			edge.tmpVecPt1 = MbVector3D(0, 0, 0);
			edge.tmpVecPt2 = MbVector3D(0, 0, 0);
		}
	}
	int a = 0;
}

//ненужно
void initEdges(Grid* grid) {
	// создаю список ребер в grid 
	for (size_t i = 0; i < grid->aCells.size(); i++) {

		Cell cell = grid->aCells[i];

		for (size_t j = 0; j < cell.m_aPointsId.size(); j++) {
			int ind = cell.m_aPointsId[j];

			Knot knot = grid->aPoints[ind];
			Knot knot2D = grid->aPoints2D[ind];
			int indNext = cell.m_aPointsId[Inc(j, 1, 4)],
				indPreview = cell.m_aPointsId[Inc(j, -1, 4)];

			Knot knotNext = grid->aPoints[indNext],
				knotPreview = grid->aPoints[indPreview];

			knot.incCellsId.push_back(i);
			knot2D.incCellsId.push_back(i);

			if (knot.incEdgesId.empty()) {
				Edge edge = Edge(ind, indNext);

				int indEdge = grid->edges.size();
				grid->edges.push_back(edge);

				knot.incEdgesId.push_back(indEdge);
				knot2D.incEdgesId.push_back(indEdge);
				knotNext.incEdgesId.push_back(indEdge);
				cell.edges.push_back(indEdge);
				edge.includedCells.push_back(i);

				edge = Edge(ind, indPreview);

				indEdge = grid->edges.size();
				grid->edges.push_back(edge);

				knot.incEdgesId.push_back(indEdge);
				knot2D.incEdgesId.push_back(indEdge);
				knotPreview.incEdgesId.push_back(indEdge);
				cell.edges.push_back(indEdge);
				edge.includedCells.push_back(i);


				continue;
			}

			int isNeedPreviewEdge = -1,
				isNeedNextEdge = -1;

			for (int w = 0; w < knot.incEdgesId.size(); w++) {
				if (ind != indNext) {
					if (grid->edges[knot.incEdgesId[w]].pt1() == indNext ||
						grid->edges[knot.incEdgesId[w]].pt2() == indNext) {
						isNeedNextEdge = w;
					}
				}
				if (ind != indPreview) {
					if (grid->edges[knot.incEdgesId[w]].pt1() == indPreview ||
						grid->edges[knot.incEdgesId[w]].pt2() == indPreview) {
						isNeedPreviewEdge = w;
					}
				}
			}

			if (isNeedNextEdge == -1) {
				Edge edge = Edge(ind, indNext);
				int indEdge = grid->edges.size();
				grid->edges.push_back(edge);

				knot.incEdgesId.push_back(indEdge);
				knot2D.incEdgesId.push_back(indEdge);
				knotNext.incEdgesId.push_back(indEdge);
				cell.edges.push_back(indEdge);

				edge.includedCells.push_back(i);

			} else {
				Edge edge = Edge(ind, indNext);
				for (int q = 0; q < grid->edges.size(); q++) {
					if ((edge.pt1() == grid->edges[q].pt1()) && (edge.pt2() == grid->edges[q].pt2()) ||
						(edge.pt2() == grid->edges[q].pt1()) && (edge.pt1() == grid->edges[q].pt2())) {
						if (std::find(cell.edges.begin(), cell.edges.end(), q) == cell.edges.end()) {
							cell.edges.push_back(q);
						}
					}
				}

				if (std::find(grid->edges[knot.incEdgesId[isNeedNextEdge]].includedCells.begin(),
					grid->edges[knot.incEdgesId[isNeedNextEdge]].includedCells.end(),
					i) == grid->edges[knot.incEdgesId[isNeedNextEdge]].includedCells.end()) {
					grid->edges[knot.incEdgesId[isNeedNextEdge]].includedCells.push_back(i);

				}
			}

			if (isNeedPreviewEdge == -1) {
				Edge edge = Edge(ind, indPreview);
				int indEdge = grid->edges.size();
				grid->edges.push_back(edge);

				knot.incEdgesId.push_back(indEdge);
				knot2D.incEdgesId.push_back(indEdge);
				knotPreview.incEdgesId.push_back(indEdge);


				cell.edges.push_back(indEdge);

				edge.includedCells.push_back(i);

			} else {
				if (std::find(grid->edges[knot.incEdgesId[isNeedPreviewEdge]].includedCells.begin(),
					grid->edges[knot.incEdgesId[isNeedPreviewEdge]].includedCells.end(),
					i) == grid->edges[knot.incEdgesId[isNeedPreviewEdge]].includedCells.end()) {
					grid->edges[knot.incEdgesId[isNeedPreviewEdge]].includedCells.push_back(i);

				}
			}
		}

		for (int j = 0; j < 2; j++) {
			Edge e = Edge(cell.m_aPointsId[j], cell.m_aPointsId[j + 2]);
			e.diag = true;
			grid->edges.push_back(e);
			grid->aCells[i].edgesDiag.push_back(grid->edges.size() - 1);

		}
	}


}

void initEdgesInCellsMethod(Grid* grid) {
	for (int i = 0; i < grid->aCells.size(); i++) {
		for (int j = 0; j < 4; j++) {
			int pt1 = grid->aCells[i].m_aPointsId[j],
				pt2 = grid->aCells[i].m_aPointsId[Inc(j, 1, 4)];
			Edge edge(pt1, pt2);
			edge.idPointsIn.push_back(pt1);
			edge.idPointsIn.push_back(pt2);

			int count = 0;

			bool flag = true;

			for (auto& edgeG : grid->edges) {
				if ((edgeG.pt1() == edge.pt1() && edgeG.pt2() == edge.pt2()) || (edgeG.pt2() == edge.pt1() && edgeG.pt1() == edge.pt2())) {
					edgeG.includedCells.push_back(i);
					grid->aCells[i].edges.push_back(count);
					grid->aPoints[pt1].incCellsId.push_back(i);

					if (std::find(grid->aPoints[pt1].incEdgesId.begin(), grid->aPoints[pt1].incEdgesId.end(), count) == std::end(grid->aPoints[pt1].incEdgesId)) {
						grid->aPoints[pt1].incEdgesId.push_back(count);
					}
					if (std::find(grid->aPoints[pt2].incEdgesId.begin(), grid->aPoints[pt2].incEdgesId.end(), count) == std::end(grid->aPoints[pt2].incEdgesId)) {
						grid->aPoints[pt1].incEdgesId.push_back(count);
					}

					flag = false;
				}
				count++;

			}
			if (flag) {
				edge.includedCells.push_back(i);
				grid->aCells[i].edges.push_back(grid->edges.size());
				grid->aPoints[pt1].incCellsId.push_back(i);
				//grid->aPoints[pt2].incCellsId.push_back(i);
				grid->aPoints[pt1].incEdgesId.push_back(grid->edges.size());
				grid->aPoints[pt2].incEdgesId.push_back(grid->edges.size());
				grid->edges.push_back(edge);
			}

		}
	}
}

Grid* ConvetQDecFromGrid(QDec* qdec) {
	std::map<int, Knot> aPoints;
	std::vector<Cell> aCells;
	std::vector<Edge> edges;

	for (int i = 0; i < qdec->boundsApprox.size(); i++) {
		Knot knot = Knot(qdec->boundsApprox[i].x, qdec->boundsApprox[i].y, qdec->boundsApprox[i].z);
		knot.flagFix = true;
		aPoints.insert({ i, knot });
	}
	for (int i = 0; i < qdec->inPoints.size(); i++) {
		Knot knot = Knot(qdec->inPoints[i].x, qdec->inPoints[i].y, qdec->inPoints[i].z);
		aPoints.insert({ i + qdec->boundsApprox.size(), knot });
	}

	for (int i = 0; i < qdec->quads.size(); i++) {
		std::vector<int> pointsIdForCell;
		std::vector<int> edgesIdForCell;
		double idealEdge = 0;
		double interv = 0;
		for (int j = 0; j < 4; j++) {
			int start = qdec->quads[i].edges[j][0];
			int finish = qdec->quads[i].edges[j][1];

			// проверка есть ли вектор в массиве
			/*
			int count = 0;
			bool flag = false;
			for (auto edge : edges) {
				if ((edge.pt1() == start && edge.pt2() == finish) || (edge.pt2() == start && edge.pt1() == finish)) {
					edgesIdForCell.push_back(count);
					aPoints[qdec->edgesPointIndices[start]].incEdgesId.push_back(count);
					aPoints[qdec->edgesPointIndices[finish]].incEdgesId.push_back(count);

					aPoints[qdec->edgesPointIndices[start]].flagEdgePt1Pt2 = true;
					aPoints[qdec->edgesPointIndices[finish]].flagEdgePt1Pt2 = true;

					pointsIdForCell.push_back(qdec->edgesPointIndices[start]);
					interv += edge.idPointsIn.size() - 1;
					idealEdge += GetLenghtEdge(aPoints, edge);
					flag = true;
				}
				count++;
			}

			if (flag)continue;*/
			Edge e = Edge(qdec->edgesPointIndices[start], qdec->edgesPointIndices[finish]);

			if (start > finish) {
				std::swap(start, finish);
			}



			for (int i = start; i <= finish; i++) {
				e.idPointsIn.push_back(qdec->edgesPointIndices[i]);
			}
			edges.push_back(e);
			edgesIdForCell.push_back(edges.size() - 1);
			aPoints[qdec->edgesPointIndices[start]].incEdgesId.push_back(edges.size() - 1);
			aPoints[qdec->edgesPointIndices[finish]].incEdgesId.push_back(edges.size() - 1);

			aPoints[qdec->edgesPointIndices[start]].flagEdgePt1Pt2 = true;
			aPoints[qdec->edgesPointIndices[finish]].flagEdgePt1Pt2 = true;

			pointsIdForCell.push_back(qdec->edgesPointIndices[start]);
			interv += e.idPointsIn.size() - 1;
			idealEdge += GetLenghtEdge(aPoints, e);
		}

		Cell cell = Cell(pointsIdForCell);

		idealEdge /= interv;
		cell.m_idealEdge = idealEdge;
		cell.edges = edgesIdForCell;

		for (int i = 0; i < pointsIdForCell.size(); i++) {
			if (!(std::find(aPoints[pointsIdForCell[i]].incCellsId.begin(),
				aPoints[pointsIdForCell[i]].incCellsId.end(), aCells.size())
				!= aPoints[pointsIdForCell[i]].incCellsId.end())) {
				aPoints[pointsIdForCell[i]].incCellsId.push_back(aCells.size());
			} else {
				continue;
			}

		}

		aCells.push_back(cell);
	}


	Grid* grid = new Grid(aPoints, aCells, edges);

	return grid;
}

void initIncCell(Grid* grid) {
	auto findElem = [&](std::vector<int>& vec, int elem) {
		auto find{ std::find(vec.begin(), vec.end(), elem) };
		if (find == vec.end()) {
			vec.push_back(elem);
		}
	};
	for (int i = 0; i < grid->aCells.size(); i++) {
		for (auto idEdge : grid->aCells[i].edges) {
			grid->edges[idEdge].includedCells.push_back(i);
			for (auto idKnot : grid->edges[idEdge].idPointsIn) {
				findElem(grid->aPoints[idKnot].incCellsId, i);
				findElem(grid->aPoints[idKnot].incEdgesId, idEdge);
			}
		}
		for (auto idEdge : grid->aCells[i].edgesDiag) {
			grid->edges[idEdge].includedCells.push_back(i);
			for (auto idKnot : grid->edges[idEdge].idPointsIn) {
				findElem(grid->aPoints[idKnot].incCellsId, i);
				findElem(grid->aPoints[idKnot].incEdgesId, idEdge);
			}
		}
	}

	for (int i = 0; i < grid->edges.size(); i++) {
		for (int j = 0; j < grid->edges.size(); j++) {
			if (i == j)continue;
			if (grid->edges[i].pt1() == grid->edges[j].pt1()) {
				if (grid->edges[i].pt2() == grid->edges[j].pt2()) {
					for (int w = 0; w < grid->edges[j].includedCells.size(); w++) {
						findElem(grid->edges[i].includedCells, grid->edges[j].includedCells[w]);
					}
					if (i < j) {
						for (auto knot : grid->edges[j].idPointsIn) {
							grid->aPoints[knot].incEdgesId.erase(
								std::remove(grid->aPoints[knot].incEdgesId.begin(),
								grid->aPoints[knot].incEdgesId.end(), j),
								grid->aPoints[knot].incEdgesId.end());
						}
					}
				}
			}
		}
	}


	//tests
	/*
	for (auto Knot : grid->aPoints)
	{
		if (!Knot.second.flagFix)
		{
			if (Knot.second.diag)
			{
				if (Knot.second.incEdgesId.size() != 1)
				{
					grid->aPoints[Knot.first].pole = true;
				}

				if (Knot.second.incCellsId.size() != 1)
				{
					grid->aPoints[Knot.first].pole = true;

				}
				else continue;


			}
			if (Knot.second.flagEdgePt1Pt2)
			{
				if (Knot.second.incEdgesId.size() != 8)
				{
					grid->aPoints[Knot.first].pole = true;
				}
				if (Knot.second.incCellsId.size() != 4)
				{
					grid->aPoints[Knot.first].pole = true;
				}
			}
			else
			{
				if (Knot.second.incEdgesId.size() != 1)
				{
					grid->aPoints[Knot.first].pole = true;
				}
				if (Knot.second.incCellsId.size() != 2)
				{
					grid->aPoints[Knot.first].pole = true;
				}
			}

			//WriteFileQDec(grid, false);
		}

	}
	for (auto edge : grid->edges)
	{
		if (grid->aPoints[edge.pt1()].flagFix && grid->aPoints[edge.pt2()].flagFix) continue;
		if (edge.diag)
		{
			if (edge.includedCells.size() != 1)
			{
				for (auto knot : edge.idPointsIn)
				{
					grid->aPoints[knot].pole = true;
				}
			}
		}
		else
		{
			if (edge.includedCells.size() != 2)
			{
				for (auto knot : edge.idPointsIn)
				{
					grid->aPoints[knot].pole = true;
				}
			}
		}
	}

	WriteFileQDec(grid, false);
	int a = 0;
	*/
}
//углы
//обход очередью
std::vector<int> QueueByPass(Grid* grid) {

	std::vector<int> res;
	//очередь точек
	std::queue<int> queueIdPoints;
	//обработанные точки
	std::set<int> donePoints;
	//первую берем первой из точек
	int firstPoint = grid->aPoints.begin()->first;

	queueIdPoints.push(firstPoint);
	while (!queueIdPoints.empty()) {
		int firstInQueue = queueIdPoints.front();
		if (donePoints.count(firstInQueue) != 0) {
			queueIdPoints.pop();
			continue;
		}

		queueIdPoints.pop();
		donePoints.insert({ firstInQueue });

		for (int i = 0; i < grid->aPoints[firstInQueue].incEdgesId.size(); i++) {
			int idSecondPoint = grid->edges[grid->aPoints[firstInQueue].incEdgesId[i]].pt1();;
			if (idSecondPoint == firstPoint) {
				idSecondPoint = grid->edges[grid->aPoints[firstInQueue].incEdgesId[i]].pt2();
			}
			if (donePoints.count(idSecondPoint) == 0) {
				if (grid->aPoints[idSecondPoint].flagFix == false) {
					queueIdPoints.push(idSecondPoint);
				}
			}
		}


		if (grid->aPoints[firstInQueue].flagFix == true) {
			continue;
		}
		res.push_back(firstPoint);
	}
	return res;
}

//обход от границы
std::vector<int> BorderByPass(Grid* grid) {

	std::vector<int> res;

	std::queue<int> queueIdCells;
	std::set<int> processedIdCells;
	std::set<int> processedIdKnots;
	for (int i = 0; i < grid->aCells.size(); i++) {
		if (grid->aCells[i].GetKnots()[0].flagFix == true ||
			grid->aCells[i].GetKnots()[1].flagFix == true ||
			grid->aCells[i].GetKnots()[2].flagFix == true ||
			grid->aCells[i].GetKnots()[3].flagFix == true) {
			queueIdCells.push(i);
		}
	}

	while (!queueIdCells.empty()) {
		int idCell = queueIdCells.front();

		if (processedIdCells.count(idCell) != 0) {
			queueIdCells.pop();
			continue;
		}
		queueIdCells.pop();
		processedIdCells.insert({ idCell });
		Cell  cell = grid->aCells[idCell];
		for (int j = 0; j < cell.edges.size(); j++) {
			for (int g = 0; g < grid->edges[cell.edges[j]].includedCells.size(); g++) {
				std::vector<int> knotsId = grid->aCells[grid->edges[cell.edges[j]].includedCells[g]].m_aPointsId;

				if (processedIdCells.count(grid->edges[cell.edges[j]].includedCells[g]) == 0) {
					if (grid->aPoints[knotsId[0]].flagFix != true &&
						grid->aPoints[knotsId[1]].flagFix != true &&
						grid->aPoints[knotsId[2]].flagFix != true &&
						grid->aPoints[knotsId[3]].flagFix != true) {
						queueIdCells.push(grid->edges[cell.edges[j]].includedCells[g]);
					}

				}
			}
		}

		std::vector<int> knotsId = cell.m_aPointsId;
		for (int j = 0; j < knotsId.size(); j++) {
			if (grid->aPoints[knotsId[j]].flagFix == true || processedIdKnots.count(knotsId[j]) != 0) {
				continue;
			}
			res.push_back(knotsId[j]);
		}

		for (int j = 0; j < 4; j++) {
			processedIdKnots.insert({ cell.m_aPointsId[j] });
		}
	}

	return res;
}

//обход рандом
std::vector<int> RandomByPass(Grid* grid) {
	std::vector<int> res;
	for (auto knot : grid->aPoints) {
		if (knot.second.flagFix)continue;
		if (knot.second.incEdgesId.size() > 2) {
			res.push_back(knot.first);
		}
	}
	return res;
}

Knot CalculateMoveVecForPointAngle(Grid* grid, int idMoveKnot, std::vector<Edge>& edges, bool addTargetAngel) {
	MbCartPoint3D moveKnot, secKnot;
	//edges[0].pt1() == idMoveKnot ? moveKnot = grid->aPoints2D[edges[0].pt1()].pt : moveKnot = grid->aPoints2D[edges[0].pt2()].pt;
	//edges[0].pt1() == idMoveKnot ? secKnot = grid->aPoints2D[edges[0].pt2()].pt : secKnot = grid->aPoints2D[edges[0].pt1()].pt;
	edges[0].pt1() == idMoveKnot ? moveKnot = grid->aPoints2D[edges[0].idPointsIn[edges[0].idPointsIn.size() - 2]].pt
		: moveKnot = grid->aPoints2D[edges[0].idPointsIn[1]].pt;
	edges[0].pt1() == idMoveKnot ? secKnot = grid->aPoints2D[edges[0].pt2()].pt : secKnot = grid->aPoints2D[edges[0].pt1()].pt;

	int idKnotCenter;
	edges[0].pt1() == idMoveKnot ? idKnotCenter = edges[0].pt2() : idKnotCenter = edges[0].pt1();
	MbCartPoint3D leftKnot, rightKnot;
	//grid->aPoints2D[edges[1].pt1()].pt == secKnot ? leftKnot = grid->aPoints2D[edges[1].pt2()].pt : leftKnot = grid->aPoints2D[edges[1].pt1()].pt;
	//grid->aPoints2D[edges[2].pt1()].pt == secKnot ? rightKnot = grid->aPoints2D[edges[2].pt2()].pt : rightKnot = grid->aPoints2D[edges[2].pt1()].pt;
	grid->aPoints2D[edges[1].pt1()].pt == secKnot ? leftKnot = grid->aPoints2D[edges[1].idPointsIn[1]].pt
		: leftKnot = grid->aPoints2D[edges[1].idPointsIn[edges[1].idPointsIn.size() - 2]].pt;
	grid->aPoints2D[edges[2].pt1()].pt == secKnot ? rightKnot = grid->aPoints2D[edges[2].idPointsIn[1]].pt
		: rightKnot = grid->aPoints2D[edges[2].idPointsIn[edges[2].idPointsIn.size() - 2]].pt;

	//вычисляем какая точка правая, а какая левая
	bool swap = false;
	for (auto incCell : grid->aPoints[idMoveKnot].incCellsId) {
		std::vector<int> pointsId = grid->aCells[incCell].m_aPointsId;
		for (int j = 0; j < pointsId.size(); j++) {
			if (idMoveKnot == pointsId[j]) {
				if (leftKnot == grid->aPoints2D[pointsId[Inc(j, -1, 4)]].pt || rightKnot == grid->aPoints2D[pointsId[Inc(j, 1, 4)]].pt) {
					swap = true;
				}
			}
		}

	}
	if (swap) {
		std::swap(leftKnot, rightKnot);
	}

	//строим вектора 
	MbVector3D left = leftKnot - secKnot;
	MbVector3D right = rightKnot - secKnot;
	MbVector3D main = moveKnot - secKnot;


	//ищем угол
	double alphaRight = acos((main * right) / (main.Length() * right.Length()));
	double alphaLeft = acos((main*left) / (main.Length() * left.Length()));
	double beta = (alphaRight - alphaLeft) / 2;

	if (!edges[0].diag && addTargetAngel) {

		if (alphaLeft >= M_PI)
			throw "alphaLeft >= M_PI";
		grid->aPoints[idKnotCenter].targetAngle.push_back(alphaLeft);

		//grid->aPoints[idKnotCenter].targetAngle.push_back(alphaRight);
	}
	//вычисляем новые координаты
	//moveKnot = grid->aPoints2D[idMoveKnot].pt;
	double newX = secKnot.x + (moveKnot.x - secKnot.x)*cos(beta) - (moveKnot.y - secKnot.y)*sin(beta);
	double newY = secKnot.y + (moveKnot.x - secKnot.x)*sin(beta) + (moveKnot.y - secKnot.y)*cos(beta);
	newX -= moveKnot.x;
	newY -= moveKnot.y;
	newX = grid->aPoints2D[idMoveKnot].pt.x + newX;
	newY = grid->aPoints2D[idMoveKnot].pt.y + newY;
	return Knot(newX, newY);
}

void CalculateMoveVecForPointsAngle(Grid* grid, const std::vector<int>& queueIdKnots, MbSplineSurface* srfNurbs, bool addTargetAngel = true) {

	for (auto idKnot : queueIdKnots) {
		if (grid->aPoints[idKnot].flagFix) {
			continue;
		}
		std::vector<Knot> newPlaceKnot;
		for (auto incEdge : grid->aPoints[idKnot].incEdgesId) {
			std::vector<Edge> edges;
			Edge edge = grid->edges[incEdge];
			edges.push_back(edge);

			std::vector<int> tmp;
			tmp.push_back(incEdge);

			int idKnot2;
			grid->edges[incEdge].pt1() == idKnot ? idKnot2 = grid->edges[incEdge].pt2() : idKnot2 = grid->edges[incEdge].pt1();

			for (auto incCellinEdge : grid->edges[incEdge].includedCells) {
				int a = incCellinEdge;
				for (auto incEdgeinCell : grid->aCells[incCellinEdge].edges) {
					grid->edges[incEdgeinCell].napryazhenie = 0;
					if (grid->edges[incEdgeinCell].pt1() == idKnot2 || grid->edges[incEdgeinCell].pt2() == idKnot2) {
						if (grid->edges[incEdgeinCell].pt1() != idKnot && grid->edges[incEdgeinCell].pt2() != idKnot) {
							bool flag = true;
							for (int i = 0; i < edges.size(); i++) {
								if ((grid->edges[incEdgeinCell].pt1() == edges[i].pt1() && grid->edges[incEdgeinCell].pt2() == edges[i].pt2())
									|| (grid->edges[incEdgeinCell].pt1() == edges[i].pt2() && grid->edges[incEdgeinCell].pt2() == edges[i].pt1())) {
									flag = false;
								}
							}
							if (flag) {
								edges.push_back(grid->edges[incEdgeinCell]);
								tmp.push_back(incEdgeinCell);
							}
						}
					}
				}
			}
			//for (int i = 0; i < edges.size(); i++)
			//{
			//	grid->edges[tmp[i]].napryazhenie = -1;
			//	for (auto id : edges[i].idPointsIn)
			//	{
			//		grid->aPoints[id].pole = true;
			//	}
			//}
			if (edges.size() < 3) {
				throw - 1;
			}
			if ((edges[1].pt1() == edges[2].pt1() && edges[1].pt2() == edges[2].pt2()) || (edges[1].pt2() == edges[2].pt1() && edges[1].pt1() == edges[2].pt2())) {

				continue;
			}
			newPlaceKnot.push_back(CalculateMoveVecForPointAngle(grid, idKnot, edges,addTargetAngel));

		}

		MbCartPoint3D moveVec(0, 0, 0);
		for (auto knot : newPlaceKnot) {
			moveVec += knot.pt;
		}

		moveVec /= newPlaceKnot.size();
		MbCartPoint3D pt;
		srfNurbs->PointOn(moveVec.x, moveVec.y, pt);

		MbCartPoint3D Knot = grid->aPoints[idKnot].pt;
		MbCartPoint3D Knot2d = grid->aPoints2D[idKnot].pt;
		grid->aPoints2D[idKnot].pt = moveVec;
		grid->aPoints[idKnot].pt = pt;

	}
}

void MakeVecMoveFromGrid(Grid* oldGrid, Grid* newGrid) {
	for (auto knot : newGrid->aPoints) {
		oldGrid->aPoints[knot.first].moveVect += knot.second.pt - oldGrid->aPoints[knot.first].pt;
	}
}

void CalculateMoveVectAngle(Grid* grid, MbSplineSurface* srfNurbs) {
	//создаем новую сетку
	Grid* newGrid = new Grid(grid);
	std::vector<int> queueIdKnots;
	//высчитываем очередь обхода точек
	queueIdKnots = RandomByPass(newGrid);
	//высчитываем новые координаты точек
	CalculateMoveVecForPointsAngle(newGrid, queueIdKnots, srfNurbs);
	//делаем вектор перемещения из новых координат новой сетки в старую сетку
	for (auto knot : newGrid->aPoints) {
		grid->aPoints[knot.first].targetAngle = knot.second.targetAngle;
	}
	MakeVecMoveFromGrid(grid, newGrid);
	for (int i = 0; i < grid->edges.size(); i++) {
		LineMorphing(grid, grid->edges[i]);
	}
	//перемещаем как в алгоритме с пружинками
	//CalculateMoveKnots(grid, srfNurbs);
}


// градиентный спуск
double DiffTargetAnglesAndIdeal(const Knot& knot) {
	if (knot.targetAngle.size() == 0) {

		return 0;
	}
	double idealAngle = 2 * M_PI / knot.targetAngle.size();
	double res = 0;
	for (int i = 0; i < knot.targetAngle.size(); i++) {
		res += std::abs(knot.targetAngle[i] - idealAngle);
	}
	return res;

}
void RefreshTargetAngles(Knot& knot) {
	for (int i = 0; i < knot.targetAngle.size(); i++) {
		knot.targetAngle.pop_back();
	}
}
void ReCalcAngle(Grid* grid, MbSplineSurface* srfNurbs, int idKnot, Knot& knot) {
	std::vector<int> queueKnots;
	for (auto edge : knot.incEdgesId) {
		if (grid->edges[edge].diag)continue;
		int idKnotMod = grid->edges[edge].pt1() == idKnot ? grid->edges[edge].pt2() : grid->edges[edge].pt1();
		queueKnots.push_back(idKnotMod);

	}
	CalculateMoveVecForPointsAngle(grid, queueKnots, srfNurbs, false);
}
double CalcDx(Grid* grid) {
	double min = GetLenghtEdge(grid->aPoints, grid->edges[0]);
	for (auto edge : grid->edges) {
		double lenght = GetLenghtEdge(grid->aPoints, edge);
		if (min > lenght) {
			min = lenght;
		}
	}

	return min / 1000;
}
double CalcFxy(Grid* grid, double fxy = 0, MbSplineSurface* srfNurbs = nullptr, int* indexDf = nullptr, int* xOry = nullptr) {
	double res = 0;
	if (!indexDf) {
		for (auto knot : grid->aPoints) {
			if (!knot.second.flagFix) {
				res += DiffTargetAnglesAndIdeal(knot.second);
				//RefreshTargetAngles(knot.second);
			}
		}
	} else {
		for (auto& knot : grid->aPoints) {
			if (!knot.second.flagFix) {
				if (knot.first == *indexDf) {
					if (*indexDf == 24) {
						int a = 0;
					}
					MbCartPoint3D pt = knot.second.pt;
					xOry == 0 ? grid->aPoints2D[knot.first].pt.x += grid->dx : grid->aPoints2D[knot.first].pt.y += grid->dx;

					srfNurbs->PointOn(grid->aPoints2D[knot.first].pt.x, grid->aPoints2D[knot.first].pt.y, knot.second.pt);

					std::vector<double> oldAngles = knot.second.targetAngle;
					while (knot.second.targetAngle.size() != 0) {
						knot.second.targetAngle.pop_back();
					}
					//делаем пересчет углов
					ReCalcAngle(grid, srfNurbs, knot.first, knot.second);

					res +=DiffTargetAnglesAndIdeal(knot.second);
					while (knot.second.targetAngle.size() != 0) {
						knot.second.targetAngle.pop_back();
					}
					knot.second.targetAngle = oldAngles;
					knot.second.pt = pt;

				} else {
					res += DiffTargetAnglesAndIdeal(knot.second);
				}
			}
		}
	}
	if (fxy == 0)
		return res;
	else
		return (res - fxy)/grid->dx;
}
std::vector<double> CalcDF(Grid* grid, MbSplineSurface* srfNurbs, double fxy) {
	std::vector<double> DF;
	for (auto knot : grid->aPoints) {
		if (!knot.second.flagFix) {
			DF.push_back(CalcFxy(grid,fxy, srfNurbs, new int(knot.first), new int(0)));
			DF.push_back(CalcFxy(grid,fxy, srfNurbs, new int(knot.first), new int(1)));
		}
	}

	return DF;
}
std::vector <double> GetFVector(Grid* grid) {
	std::vector<double> F;
	for (auto knot : grid->aPoints2D) {
		if (!grid->aPoints[knot.first].flagFix) {
			F.push_back(knot.second.pt.x);
			F.push_back(knot.second.pt.y);
		}
	}
	return F;
}
void NormalizeDir(std::vector<double>& dir) {
	double max = 0;
	for (int i = 0; i < dir.size(); i++) {
		std::abs(dir[i]) > max ? max = std::abs(dir[i]) : max = max;
	}
	for (int i = 0; i < dir.size(); i++) {
		dir[i] /= max;
	}
}
std::vector<double> CalcR(const std::vector<double>& r, const std::vector<double>& dir, const double& step) {
	std::vector<double> res;
	for (int i = 0; i < r.size(); i++) {
		res.push_back(r[i] - dir[i] * step);
	}
	return res;
}
double GetDirDir(std::vector<double> dirPred, std::vector<double> dir) {
	double res = 0;
	for (int i = 0; i < dir.size(); i++) {
		res += dirPred[i] * dir[i];
	}
	return res;
}


void CalcMoveVec(Grid* grid, MbSplineSurface* srfNurbs, std::vector<double> r) {
	//double tmp = r[i] - aPoint2d[i].pt.x 
	// pointOn
	// aPoint2d[i].pt = pt;
	std::map<int, Knot> Knots;
	int count = 0;
	for (int i = 0; i < r.size(); i += 2) {
		count++;
		Knot pt(MbCartPoint3D(r[i], r[i + 1], 0));
		Knots.insert({ count,pt });
	}
	count = 0;
	for (auto& knot : grid->aPoints2D) {
		count++;
		double newKoordx = Knots[count].pt.x;
		double newKoordy = Knots[count].pt.x;
		knot.second.pt = MbCartPoint3D(newKoordx, newKoordy, 0);
		srfNurbs->PointOn(newKoordx, newKoordy, grid->aPoints[knot.first].pt);
	}
}
void Gradient(Grid* grid, MbSplineSurface* srfNurbs) {

	CalculateMoveVectAngle(grid, srfNurbs);
	double step = grid->step;

	double fxy = CalcFxy(grid);

	std::vector<double> dF = CalcDF(grid, srfNurbs, fxy);
	std::vector<double> fVec = GetFVector(grid);

	NormalizeDir(dF);

	if (!grid->dfPred.empty()) {
		std::vector<double> dfPred = grid->dfPred;
		grid->step = GetDirDir(dfPred, dF) < 0 ? step / 2 : step;
	}

	std::vector<double> r = CalcR(fVec, dF, grid->step);

	Grid* newGrid = new Grid(grid);
	CalcMoveVec(newGrid, srfNurbs, r);
	MakeVecMoveFromGrid(grid, newGrid);

	for (int i = 0; i < grid->edges.size(); i++) {
		LineMorphing(grid, grid->edges[i]);
	}

	for (auto& knot : grid->aPoints) {
		RefreshTargetAngles(knot.second);
	}
	grid->dfPred = dF;
}


void TmpFixKnots(Grid* grid) {
	for (auto edge : grid->edges) {
		if (!edge.diag) {
			if (edge.includedCells.size() == 1) {
				for (auto knot : edge.idPointsIn) {
					grid->aPoints[knot].flagFix = true;
				}
			}
		}
	}
}
/**
* \brief Запустить итерации по перемещению узлов
* \param [in, out] grid Сетка
* \param [in] srfNurbs Поверхность
*/
void StartIterations(Grid* grid, MbSplineSurface* srfNurbs) {

	// решить проблему с ребрами
	// возможно ввести проверку чтобы не создавать дубликатов
	initEdgesInCellsMethod(grid);
	MorphDiag(grid, true);

	//возможно уже не надо (проверка показала что ничего нового не привносит после initEdgesInCellsMethod)
	// надо подумать
	initIncCell(grid);

	ProectionOn2D(grid, srfNurbs);
	grid->dx = CalcDx(grid);
	grid->step = 0.1;
	while (true) {
		iteration++;
		double oldEn = grid->energy;
		//CalculateMoveVect(grid);
		//CalculateMoveVectAngle(grid, srfNurbs);
		Gradient(grid, srfNurbs);

		CalculateMoveKnots(grid, srfNurbs);
		//WriteFileQDec(grid,false);
		if (oldEn != 0) {
			if (oldEn / grid->energy < 1 + EPSILON) {
				//break;
			}
		}
		std::cout << "Interation : " << iteration << std::endl;
		std::cout << "Energy change ratio : " << oldEn / grid->energy << std::endl;
		if (iteration > 0)break;
	}
}


