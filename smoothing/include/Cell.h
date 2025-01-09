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

	/**
	* \brief Инициализировать "идеальные" ребро и диагональ при криволиненых ребрах
	*/
	void InitIntervals(Grid* grid);
	
	// id угловых точек ячейки
	std::vector<int> m_aPointsId;
	// "идеальная" диагональ
	double m_idealEdge;
	// "идеальное" ребро
	double m_idealDiag;
	// ссылка на сетку
	Grid *m_grid;
	// флаг показывающий обработана ли ячейка(расставлены ли у точек вектора перемещения)
	bool processed;
	// флаг показывающий является ли ячейка треугольником
	bool isTriangel;
	// id боковых ребер ячейки
	std::vector<int> edges;
	// id даигоналей ячейки
	std::vector<int> edgesDiag;
};
#endif // CELL_H
