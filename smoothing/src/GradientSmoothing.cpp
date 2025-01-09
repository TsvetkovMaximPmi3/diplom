#include "GradientSmoothing.h"

double DiffTargetAnglesAndIdeal(const Knot& knot) {
	if (knot.targetAngle.size() == 0) {

		return 0;
	}
	double idealAngle = 2 * M_PI / knot.targetAngle.size();
	double res = 0;
	double check2Pi = 0;

	for (int i = 0; i < knot.targetAngle.size(); i++) {
		check2Pi += knot.targetAngle[i];
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

	}
	CalculateMoveVecForPointsAngle(grid, queueKnots, srfNurbs, true, false);
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


double  CalcFxyN(Grid* grid, MbSplineSurface* srfNurbs, int indexKnot, int xOry) {
	double res = 0;
	for (auto& knot : grid->aPoints) {

		if (knot.first == indexKnot) {

			MbCartPoint3D pt = knot.second.pt;
			MbCartPoint3D pt1 = grid->aPoints2D[knot.first].pt;
			if (xOry == 0) {
				grid->aPoints2D[knot.first].pt.x += grid->dx;
			} else {
				grid->aPoints2D[knot.first].pt.y += grid->dx;
			}

			srfNurbs->PointOn(grid->aPoints2D[knot.first].pt.x, grid->aPoints2D[knot.first].pt.y, knot.second.pt);

			std::vector<double> oldAngles = knot.second.targetAngle;

			while (knot.second.targetAngle.size() != 0) {
				knot.second.targetAngle.pop_back();
			}
			//делаем пересчет углов
			ReCalcAngle(grid, srfNurbs, knot.first, knot.second);

			res += DiffTargetAnglesAndIdeal(knot.second);
			while (knot.second.targetAngle.size() != 0) {
				knot.second.targetAngle.pop_back();
			}
			knot.second.targetAngle = oldAngles;
			knot.second.pt = pt;
			grid->aPoints2D[knot.first].pt = pt1;

		} else {
			res += DiffTargetAnglesAndIdeal(knot.second);
		}

	}
	return res;
}

double CalcFxy(Grid* grid, double fxy = 0, MbSplineSurface* srfNurbs = nullptr, int* indexDf = nullptr, int* xOry = nullptr) {
	double res = 0;
	if (!indexDf) {
		for (auto knot : grid->aPoints) {
			res += DiffTargetAnglesAndIdeal(knot.second);

		}
	} /*else {
	  for (auto& knot : grid->aPoints) {
	  //if (!knot.second.flagFix) {
	  if (knot.first == *indexDf) {

	  MbCartPoint3D pt = knot.second.pt;
	  MbCartPoint3D pt1 = grid->aPoints2D[knot.first].pt;
	  if (xOry == 0) {
	  grid->aPoints2D[knot.first].pt.x += grid->dx;
	  } else {
	  grid->aPoints2D[knot.first].pt.y += grid->dx;
	  }

	  srfNurbs->PointOn(grid->aPoints2D[knot.first].pt.x, grid->aPoints2D[knot.first].pt.y, knot.second.pt);

	  std::vector<double> oldAngles = knot.second.targetAngle;

	  while (knot.second.targetAngle.size() != 0) {
	  knot.second.targetAngle.pop_back();
	  }
	  //делаем пересчет углов
	  ReCalcAngle(grid, srfNurbs, knot.first, knot.second);

	  res += DiffTargetAnglesAndIdeal(knot.second);
	  while (knot.second.targetAngle.size() != 0) {
	  knot.second.targetAngle.pop_back();
	  }
	  knot.second.targetAngle = oldAngles;
	  knot.second.pt = pt;
	  grid->aPoints2D[knot.first].pt = pt1;

	  } else {
	  res += DiffTargetAnglesAndIdeal(knot.second);
	  }
	  //}
	  }
	  }*/
	if (fxy == 0)
		return res;
	else
		return (res - fxy) / grid->dx;
}

std::vector<double> CalcDF(Grid* grid, MbSplineSurface* srfNurbs, double fxy) {
	std::vector<double> DF;
	for (auto knot : grid->aPoints) {
		DF.push_back(CalcFxyN(grid, srfNurbs, knot.first, 0));
		DF.push_back(CalcFxyN(grid, srfNurbs, knot.first, 1));

	}

	return DF;
}

std::vector <double> GetFVector(Grid* grid) {
	std::vector<double> F;
	for (auto knot : grid->aPoints2D) {
		F.push_back(knot.second.pt.x);
		F.push_back(knot.second.pt.y);
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
		if (grid->aPoints[knot.first].flagFix)continue;
		double newKoordx = Knots[count].pt.x;
		double newKoordy = Knots[count].pt.y;
		knot.second.pt = MbCartPoint3D(newKoordx, newKoordy, 0);
		srfNurbs->PointOn(newKoordx, newKoordy, grid->aPoints[knot.first].pt);
	}
}

void Gradient(Grid* grid, MbSplineSurface* srfNurbs) {

	CalculateMoveVectAngle(grid, srfNurbs, false);


	double step = grid->step;

	double fxy = CalcFxy(grid);
	std::vector<double> dF = CalcDF(grid, srfNurbs, fxy);
	std::vector<double> fVec = GetFVector(grid);

	double a = grid->dx * 1000;

	NormalizeDir(dF);

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