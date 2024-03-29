#include "QDecFunc.h"

void LineMorphing(Grid * grid, const Edge& edge) {
	MbVector3D vecPt1 = grid->aPoints[edge.pt1()].moveVect;
	MbVector3D vecPt2 = grid->aPoints[edge.pt2()].moveVect;
	int size = edge.idPointsIn.size();
	double intervals = 1 / ((double)size - 1);
	for (int i = 1; i < size - 1; i++) {

		grid->aPoints[edge.idPointsIn[i]].moveVect = (vecPt1*(size - i - 1) + vecPt2*(i))*intervals;

	}
}


MbCartPoint3D getPointInMtrxForIndexPt(MbCartPoint3D A, MbCartPoint3D B, MbCartPoint3D D,
									   MbCartPoint3D F, MbCartPoint3D H1, MbCartPoint3D H2, MbCartPoint3D H3, MbCartPoint3D H4, int indexI, int indexJ, int nSize, int mSize) {
	MbCartPoint3D res;
	double in = (double)indexJ / (double)mSize;
	double jm = (double)indexI / (double)nSize;
	double ijnm = (double)(indexI * indexJ) / (double)(nSize*mSize);
	res = A*(1 - in) + B *in + D*jm + F*(1 - jm) - H2*(jm - ijnm) - H3*(ijnm)-H1*(1 - jm - in + ijnm) - H4*(in - ijnm);
	return res;
}

MbCartPoint3D  getPointInMtrxForIndexEdge(Grid* grid, const Edge& e1, const Edge& e2, const Edge& e3, const Edge& e4, int indexI, int indexJ) {
	MbCartPoint3D res;

	res = getPointInMtrxForIndexPt(grid->aPoints[e1.idPointsIn[indexI]].pt,
								   grid->aPoints[e3.idPointsIn[e3.idPointsIn.size() - 1 - indexI]].pt,
								   grid->aPoints[e2.idPointsIn[indexJ]].pt,
								   grid->aPoints[e4.idPointsIn[e4.idPointsIn.size() - 1 - indexJ]].pt,
								   grid->aPoints[e1.pt1()].pt,
								   grid->aPoints[e2.pt1()].pt,
								   grid->aPoints[e3.pt1()].pt,
								   grid->aPoints[e4.pt1()].pt,
								   indexI,
								   indexJ,
								   e1.idPointsIn.size() - 1,
								   e2.idPointsIn.size() - 1
	);
	return res;
}

std::vector<std::vector<int>> addPointMatrx(Grid* grid, std::vector<Edge>& edges, bool create) {
	//если точки уже есть то удаляем их и потом сохраняем новые(нужно только для geomview двигать стороны,можно удалить)
	if (!create) {
		std::vector<int> del;
		for (auto el : grid->aPoints) {
			if (el.second.diag) {
				del.push_back(el.first);
			}
		}
		for (int i = 0; i < del.size(); i++) {
			grid->aPoints.erase(del[i]);
		}
	}

	//даелаем правильную последовательность точек в ребре
	std::vector<bool> check = { false,false,false,false };
	for (int i = 0; i < edges.size(); i++) {

		if (edges[i].pt2() != edges[Inc(i, 1, edges.size())].pt1() &&
			!(edges[i].pt1() == edges[Inc(i, -1, edges.size())].pt2() || edges[i].pt1() == edges[Inc(i, -1, edges.size())].pt1())
			&& !check[i]) {
			edges[i].reverseEdge();
			check[i] = true;
		}
	}

	std::vector<std::vector<int>> res;

	auto addPt{
		[](std::vector<std::vector<int>>& res,Grid* grid,std::vector<Edge>& edges,int ind1,int ind2) {
			MbCartPoint3D pt = getPointInMtrxForIndexEdge(grid, edges[0], edges[1], edges[2], edges[3], ind1, ind2);
			Knot newKnot = Knot(pt);
			newKnot.diag = true;
			std::vector<int>tmp;
			tmp.push_back(InsertInMapInEnd(grid, newKnot));
			res.push_back(tmp);
		}
	};

	for (int i = 1; i < edges[0].idPointsIn.size() - 1; i++) {

		addPt(res, grid, edges, i, i);
		addPt(res, grid, edges, i, edges[0].idPointsIn.size() - i - 1);
	}
	return res;
}

void ReSplitEdge(Grid* grid, Edge& e, int intervals) {
	std::vector<int> aPoints;
	if (e.idPointsIn.size() == intervals + 1)return;
	SArray<MbCartPoint3D> pts;
	for (auto j : e.idPointsIn) {
		pts.push_back(grid->aPoints[j].pt);
	}
	MbHermit3D* hermit = MbHermit3D::Create(pts, false);
	double min = 0;
	double max = hermit->GetTMax() - hermit->GetTMin();

	aPoints.push_back(e.pt1());
	for (int i = 1; i < intervals; i++) {

		double k = (i * max) / intervals;

		MbCartPoint3D pt;
		hermit->PointOn(k, pt);

		aPoints.push_back(InsertInMapInEnd(grid, Knot(pt)));
	}
	aPoints.push_back(e.pt2());
	e.idPointsIn = aPoints;
}

void MorphDiag(Grid * grid, bool flag) {
	for (int i = 0; i < grid->aCells.size(); i++) {
		std::vector<Edge> e;

		std::map<int, Knot> Knots;
		std::vector<Cell> cell{ grid->aCells[i] };
		for (int j = 0; j < grid->aCells[i].edges.size(); j++) {
			e.push_back(grid->edges[grid->aCells[i].edges[j]]);
			for (int w = 0; w < grid->edges[grid->aCells[i].edges[j]].idPointsIn.size(); w++) {

				Knots.insert({ grid->edges[grid->aCells[i].edges[j]].idPointsIn[w], grid->aPoints[grid->edges[grid->aCells[i].edges[j]].idPointsIn[w]] });
			}
		}

		Grid g = Grid(Knots, cell, e);
		if (e[0].idPointsIn.size() < e[1].idPointsIn.size()) {
			ReSplitEdge(&g, g.edges[0], e[1].idPointsIn.size() - 1);
			ReSplitEdge(&g, g.edges[2], e[1].idPointsIn.size() - 1);
		} else {
			ReSplitEdge(&g, g.edges[1], e[0].idPointsIn.size() - 1);
			ReSplitEdge(&g, g.edges[3], e[0].idPointsIn.size() - 1);
		}
		for (int j = 0; j < e.size(); j++) {
			e[j] = g.edges[j];
		}
		std::vector<std::vector<int>> res = addPointMatrx(&g, e, flag);
		Edge diag1, diag2;

		diag1.idPointsIn.push_back(e[0].pt1());
		diag2.idPointsIn.push_back(e[2].pt2());

		for (int j = 0; j < res.size(); j++) {

			g.aPoints[res[j][0]].diag = true;
			int tmp = InsertInMapInEnd(grid, g.aPoints[res[j][0]]);

			if (j % 2 == 0) {
				diag1.idPointsIn.push_back(tmp);
			} else {
				diag2.idPointsIn.push_back(tmp);
			}
		}
		diag1.idPointsIn.push_back(e[1].pt2());
		diag2.idPointsIn.push_back(e[1].pt1());

		diag1.diag = true;
		diag2.diag = true;
		if (diag1.idPointsIn.size() != 0) {
			grid->aCells[i].edgesDiag.push_back(grid->edges.size());
			for (auto idknot : diag1.idPointsIn) {
				grid->aPoints[idknot].incEdgesId.push_back(grid->edges.size());
			}
			grid->edges.push_back(diag1);
		}
		if (diag2.idPointsIn.size() != 0) {
			grid->aCells[i].edgesDiag.push_back(grid->edges.size());
			for (auto idknot : diag2.idPointsIn) {
				grid->aPoints[idknot].incEdgesId.push_back(grid->edges.size());
			}
			grid->edges.push_back(diag2);
		}
		//WriteFileQDec(grid, false);

		int a = 0;
	}

	//иницализация идеальной диагонали
	for (auto& cell : grid->aCells) {
		cell.InitIntervals(grid);
	}

}
