#include "Grid.h"
#include <cur_hermit3d.h>
#include <action_curve3d.h>
#include"Algorithm.h"
struct Edge;

void LineMorphing(Grid* grid,const Edge& edge);

void MorphDiag(Grid* grid,bool flag);
//std::vector<std::vector<int>> createMatrixPoints(Grid* grid, std::vector<Edge*> edges, bool create);

std::vector<std::vector<int>> addPointMatrx(Grid* grid,  std::vector<Edge>& edges, bool create);
void ReSplitEdge(Grid* grid, Edge& e, int intervals);