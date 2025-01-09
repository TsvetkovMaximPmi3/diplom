#pragma once
#include <mb_vector3d.h>
#include <surf_spline_surface.h>

#include "Knot.h"
#include "Grid.h"
#include "Edge.h"
#include "Cell.h"

double DiffTargetAnglesAndIdeal(const Knot& knot);
void RefreshTargetAngles(Knot& knot);
void ReCalcAngle(Grid* grid, MbSplineSurface* srfNurbs, int idKnot, Knot& knot);
double CalcDx(Grid* grid);
double  CalcFxyN(Grid* grid, MbSplineSurface* srfNurbs, int indexKnot, int xOry);
double CalcFxy(Grid* grid, double fxy = 0, MbSplineSurface* srfNurbs = nullptr, int* indexDf = nullptr, int* xOry = nullptr);
std::vector<double> CalcDF(Grid* grid, MbSplineSurface* srfNurbs, double fxy);
std::vector <double> GetFVector(Grid* grid);
void NormalizeDir(std::vector<double>& dir);
std::vector<double> CalcR(const std::vector<double>& r, const std::vector<double>& dir, const double& step);
double GetDirDir(std::vector<double> dirPred, std::vector<double> dir);
void CalcMoveVec(Grid* grid, MbSplineSurface* srfNurbs, std::vector<double> r);
void Gradient(Grid* grid, MbSplineSurface* srfNurbs);