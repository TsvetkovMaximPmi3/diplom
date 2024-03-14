#ifndef CELL_H
#define CELL_H

#include <set>
#include <map>
#include <vector>

struct Knot;
struct Grid;
struct Edge;
/**
* @struct Cell
* \brief Cтруктура ячейки
*/
struct Cell
{
	Cell(std::vector<int> aPointsInd);

	/**
	* \brief Вернуть вектор координат точек ячейки
	* \return Вектор точек
	*/
	std::vector<Knot> GetKnots();

	/**
	* \brief Инициализировать "идеальные" ребро и диагональ
	*/
	void InitIdealEdgeAndDiag();

	void InitIntervals(Grid* grid);
	
	std::vector<int> m_aPointsId;
	double m_idealEdge;
	double m_idealDiag;
	Grid *m_grid;
	bool processed;
	bool isTriangel;
	std::vector<int> edges;
	std::vector<int> edgesDiag;

	//std::vector<Edge*> edgesForQDec;
	//std::vector<Edge*> edgesDiagForQDec;
	double P;
};
#endif // CELL_H
