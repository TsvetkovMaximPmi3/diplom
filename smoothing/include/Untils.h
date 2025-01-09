#pragma once
#include "Knot.h"
#include "Grid.h"
#include "Edge.h"
#include "Cell.h"

int InsertInMapInEnd(Grid* grid, Knot pt);
double GetLenghtEdge(std::map<int, Knot>& aPoints, const Edge& edge);
size_t Inc(size_t ind, int delta, size_t size);
void initEdges(Grid* grid);
void initEdgesInCellsMethod(Grid* grid);

void initIncCell(Grid* grid);
bool SwapLR(Grid* grid, int cellId);
void TmpFixKnots(Grid* grid);