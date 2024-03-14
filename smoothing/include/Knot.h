#ifndef KNOT_H
#define KNOT_H

#include <map>

#include <mb_cart_point3d.h>

struct Edge;
struct Cell;

/**
* @struct Knot
* \brief Cтруктура для узла сетки
*/
struct Knot
{
	Knot();
	Knot(double _x, double _y);
	Knot(double _x, double _y, double _z);

	Knot(MbCartPoint3D point);
	MbCartPoint3D pt;
	MbVector3D moveVect;
	// каким ячейкам принадлежит точка
	//std::map<int, Cell* > includedCells;
	// каким ребрам принадлежит точка
	//std::map<int, Edge* > includedEdges;
	std::vector<int> incEdgesId;
	std::vector<int> incCellsId;
	bool flagFix;
	bool duble;
	bool pole;
	int idDuble;
	bool flagEdgePt1Pt2;
	std::vector<double> targetAngle;
	//int targetAngleCount;
	bool diag;

	std::vector<double> angles;

	MbVector3D* newMoveVec;

	bool peresech;

private:
	void Init() { moveVect = MbVector3D(0, 0, 0);  duble = false; pole = false; flagEdgePt1Pt2 = false;
	diag = false; newMoveVec = nullptr, peresech = false; }

};
#endif //KNOT_H