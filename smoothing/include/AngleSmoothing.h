#include <vector>

#include <queue>

#include <surf_spline_surface.h>

#include "Knot.h"
#include "Grid.h"
#include "Edge.h"
#include "Cell.h"

Knot CalculateMoveVecForPointAngle(Grid* grid, int idMoveKnot, std::vector<Edge>& edges, bool addTargetAngel);

void CalculateMoveVecForPointsAngle(Grid* grid, const std::vector<int>& queueIdKnots, MbSplineSurface* srfNurbs, bool addTargetAngel = true, bool MoveKnot = true);

void MakeVecMoveFromGrid(Grid* oldGrid, Grid* newGrid);

void CalculateMoveVectAngle(Grid* grid, MbSplineSurface* srfNurbs, bool MoveKnot = true);

std::vector<int> QueueByPass(Grid* grid);

std::vector<int> BorderByPass(Grid* grid);

std::vector<int> RandomByPass(Grid* grid, bool flagfix = true);