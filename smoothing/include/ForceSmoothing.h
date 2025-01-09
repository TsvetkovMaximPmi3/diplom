#pragma once
#include <mb_vector3d.h>
#include <surf_spline_surface.h>

#include "Knot.h"
#include "Grid.h"
#include "Edge.h"
#include "Cell.h"

MbVector3D CalculateLinePointVec(Grid* grid, int index1, int index2, double distance);
std::pair<MbVector3D, MbVector3D> CalculateLinePoint(Grid* grid, Edge& e, double distance);
void CalculateMoveVecForPoint(Grid* grid, Edge& edge, double idealEdge);
void CalculateMoveVect(Grid* grid);