#include "AngleSmoothing.h"



Knot CalculateMoveVecForPointAngle(Grid* grid, int idMoveKnot, std::vector<Edge>& edges, bool addTargetAngel) {
	MbCartPoint3D moveKnot, secKnot;

	edges[0].pt1() == idMoveKnot ? moveKnot = grid->aPoints2D[edges[0].idPointsIn[edges[0].idPointsIn.size() - 2]].pt
		: moveKnot = grid->aPoints2D[edges[0].idPointsIn[1]].pt;
	edges[0].pt1() == idMoveKnot ? secKnot = grid->aPoints2D[edges[0].pt2()].pt : secKnot = grid->aPoints2D[edges[0].pt1()].pt;

	int idKnotCenter;
	edges[0].pt1() == idMoveKnot ? idKnotCenter = edges[0].pt2() : idKnotCenter = edges[0].pt1();
	MbCartPoint3D leftKnot, rightKnot;
	leftKnot = grid->aPoints2D[edges[1].pt1()].pt == secKnot ? grid->aPoints2D[edges[1].idPointsIn[1]].pt
		: grid->aPoints2D[edges[1].idPointsIn[edges[1].idPointsIn.size() - 2]].pt;
	rightKnot = grid->aPoints2D[edges[2].pt1()].pt == secKnot ? grid->aPoints2D[edges[2].idPointsIn[1]].pt
		: grid->aPoints2D[edges[2].idPointsIn[edges[2].idPointsIn.size() - 2]].pt;

	//вычисляем какая точка правая, а какая левая
	// анализировать через площать ячейки
	bool swap = false;
	for (auto incCell : grid->aPoints[idMoveKnot].incCellsId) {
		std::vector<int> pointsId = grid->aCells[incCell].m_aPointsId;
		for (int j = 0; j < pointsId.size(); j++) {
			if (idKnotCenter == pointsId[j]) {
				if (!grid->swap) {
					if (leftKnot == grid->aPoints2D[pointsId[Inc(j, 1, 4)]].pt || rightKnot == grid->aPoints2D[pointsId[Inc(j, -1, 4)]].pt) {
						swap = true;
					}
				} else {
					if (leftKnot == grid->aPoints2D[pointsId[Inc(j, -1, 4)]].pt || rightKnot == grid->aPoints2D[pointsId[Inc(j, 1, 4)]].pt) {
						swap = true;
					}
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

	MbVector3D norm = right | main;
	MbVector3D norm1 = main | left;
	MbVector3D norm2 = left | right;

	if (norm * norm1 < 0 && norm1 * norm2 > 0) {
		norm = norm1;
	}
	if (norm * norm2 < 0 && norm1 * norm2 > 0) {
		norm = norm2;
	}

	norm.Normalize();
	double alphaRight = atan2(((right | main)*norm) / (right.Length() * main.Length()), (right * main) / (right.Length() * main.Length()));
	double alphaLeft = atan2(((main | left)*norm) / (left.Length() * main.Length()), (left * main) / (left.Length() * main.Length()));

	alphaRight = alphaRight < 0 ? M_PI2 + alphaRight : alphaRight;
	alphaLeft = alphaLeft < 0 ? M_PI2 + alphaLeft : alphaLeft;
	double beta = (alphaRight - alphaLeft) / 2;

	if (!edges[0].diag && addTargetAngel) {
		grid->aPoints[idKnotCenter].targetAngle.push_back(alphaLeft);
	}
	//вычисляем новые координаты
	double newX = secKnot.x + (moveKnot.x - secKnot.x)*cos(beta) - (moveKnot.y - secKnot.y)*sin(beta);
	double newY = secKnot.y + (moveKnot.x - secKnot.x)*sin(beta) + (moveKnot.y - secKnot.y)*cos(beta);
	newX -= moveKnot.x;
	newY -= moveKnot.y;
	newX = grid->aPoints2D[idMoveKnot].pt.x + newX;
	newY = grid->aPoints2D[idMoveKnot].pt.y + newY;
	return Knot(newX, newY);
}

void CalculateMoveVecForPointsAngle(Grid* grid, const std::vector<int>& queueIdKnots, MbSplineSurface* srfNurbs, bool addTargetAngel = true, bool MoveKnot = true) {

	for (auto idKnot : queueIdKnots) {

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

			if (edges.size() < 3) {

				continue;
			}
			if ((edges[1].pt1() == edges[2].pt1() && edges[1].pt2() == edges[2].pt2()) || (edges[1].pt2() == edges[2].pt1() && edges[1].pt1() == edges[2].pt2())) {

				continue;
			}
			newPlaceKnot.push_back(CalculateMoveVecForPointAngle(grid, idKnot, edges, addTargetAngel));

		}
		if (MoveKnot) {
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
}

void MakeVecMoveFromGrid(Grid* oldGrid, Grid* newGrid) {
	for (auto knot : newGrid->aPoints) {

		oldGrid->aPoints[knot.first].moveVect += knot.second.pt - oldGrid->aPoints[knot.first].pt;
		oldGrid->aPoints[knot.first].pole = knot.second.pole;
	}
}

void CalculateMoveVectAngle(Grid* grid, MbSplineSurface* srfNurbs, bool MoveKnot = true) {
	//создаем новую сетку
	Grid* newGrid = new Grid(grid);
	std::vector<int> queueIdKnots;
	//высчитываем очередь обхода точек
	queueIdKnots = RandomByPass(newGrid, MoveKnot);
	//queueIdKnots = BorderByPass(newGrid);
	//высчитываем новые координаты точек
	CalculateMoveVecForPointsAngle(newGrid, queueIdKnots, srfNurbs, true, MoveKnot);
	//делаем вектор перемещения из новых координат новой сетки в старую сетку
	for (auto knot : newGrid->aPoints) {
		grid->aPoints[knot.first].targetAngle = knot.second.targetAngle;
	}
	MakeVecMoveFromGrid(grid, newGrid);

	for (int i = 0; i < grid->aCells.size(); i++) {
		for (auto edge : grid->aCells[i].edges) {
			bool flag = false;
			for (auto ind : grid->edges[edge].idPointsIn) {
				if (grid->aPoints[ind].peresech) {
					flag = true;
				}
			}
			while (flag) {

				bool end = false;
				for (auto ind : grid->edges[edge].idPointsIn) {
					for (auto edge1 : grid->aCells[i].edges) {
						grid->aPoints[grid->edges[edge1].pt1()].moveVect = MbVector3D(0, 0, 0);;
						grid->aPoints[grid->edges[edge1].pt2()].moveVect = MbVector3D(0, 0, 0);;
					}
					grid->aPoints[ind].peresech = false;
				}
				for (int i = 0; i < grid->edges[edge].includedCells.size(); i++) {
					edgeIntersection2(grid, grid->aCells[grid->edges[edge].includedCells[i]], edge);
				}
				for (auto ind : grid->edges[edge].idPointsIn) {
					end = end || grid->aPoints[ind].peresech;
				}
				flag = end;
			}

		}
	}
	for (int i = 0; i < grid->edges.size(); i++) {
		LineMorphing(grid, grid->edges[i]);
	}

}

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
std::vector<int> RandomByPass(Grid* grid, bool flagfix = true) {
	std::vector<int> res;
	for (auto knot : grid->aPoints) {
		if (flagfix) {
			if (knot.second.flagFix)continue;
		}

		if (knot.second.incEdgesId.size() > 2) {
			res.push_back(knot.first);
		}
	}
	return res;
}

