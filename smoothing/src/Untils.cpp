#include "Untils.h"
int InsertInMapInEnd(Grid* grid, Knot pt) {
	auto it = grid->aPoints.end();
	it--;
	int element = it->first + 1;
	grid->aPoints.insert({ element,pt });
	return element;
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

size_t Inc(size_t ind, int delta, size_t size) {
	int j = (ind + delta) % size;
	return j < 0 ? (size + j) : j;
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
			if (pt1 == pt2)continue;
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
}

bool SwapLR(Grid* grid, int cellId) {

	MbVector3D first2D = grid->aPoints2D[grid->aCells[cellId].m_aPointsId[1]].pt - grid->aPoints2D[grid->aCells[cellId].m_aPointsId[0]].pt;
	MbVector3D second2D = grid->aPoints2D[grid->aCells[cellId].m_aPointsId[2]].pt - grid->aPoints2D[grid->aCells[cellId].m_aPointsId[1]].pt;
	MbVector3D third2D = grid->aPoints2D[grid->aCells[cellId].m_aPointsId[3]].pt - grid->aPoints2D[grid->aCells[cellId].m_aPointsId[2]].pt;
	MbVector3D fourth2D = grid->aPoints2D[grid->aCells[cellId].m_aPointsId[1]].pt - grid->aPoints2D[grid->aCells[cellId].m_aPointsId[3]].pt;

	double z = MbVector3D((first2D | second2D) + (third2D | fourth2D)).z;

	return z > 0;
}


void TmpFixKnots(Grid* grid) {
	// смотрим на включение ребер в ячейки и в зависимости от этого выставляем фиксацию точек ребра
	for (auto& edge : grid->edges) {
		if (!edge.diag) {
			if (grid->aPoints[edge.pt1()].incCellsId.size() <= 2) {
				grid->aPoints[edge.pt1()].flagFix = true;
			}
			if (grid->aPoints[edge.pt2()].incCellsId.size() <= 2) {
				grid->aPoints[edge.pt2()].flagFix = true;
			}
			if (edge.includedCells.size() == 1) {
				grid->aPoints[edge.pt1()].flagFix = true;
				grid->aPoints[edge.pt2()].flagFix = true;
			}
		}

	}
}