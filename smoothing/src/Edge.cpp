#include "Edge.h"

int Edge::pt1() const {
	if (idPointsIn.size() == 0)throw - 1;
	return idPointsIn[0];
}

int Edge::pt2() const {
	if (idPointsIn.size() == 0)throw - 1;
	return idPointsIn[idPointsIn.size() - 1];
}

void Edge::reverseEdge() {
	std::reverse(idPointsIn.begin(), idPointsIn.end());

}

Edge::Edge(int _pt1, int _pt2) {
	diag = false;
	intersection = false;
}