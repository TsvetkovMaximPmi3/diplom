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
	// координаты точки
	MbCartPoint3D pt;
	// вектор перемещения точки
	MbVector3D moveVect;
	// id ребер в которые включена точка
	std::vector<int> incEdgesId;
	// id ячеек в которые включена точек
	std::vector<int> incCellsId;
	// флаг означающий фиксацию точки
	bool flagFix;
	// есть ли у точки дублированная в переходе в 2D (к примеру верхушка конуса)
	bool duble;
	bool pole;
	// id дублированной точки
	int idDuble;
	// флаг показывающий является ли точка одним из концов ребра
	bool flagEdgePt1Pt2;
	// углы образованные ребрами которые соеденены в данной точке
	std::vector<double> targetAngle;

	// флаг показывающий лежит ли точка на диагонали ячейки
	bool diag;
	// флаг показывающий пересекает ли ребро содержащее эту точку другое ребро
	bool peresech;

private:
	void Init() { moveVect = MbVector3D(0, 0, 0);  duble = false; pole = false; flagEdgePt1Pt2 = false;
	diag = false; peresech = false; }

};
#endif //KNOT_H