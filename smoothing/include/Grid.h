#ifndef GRID_H
#define GRID_H

#include <vector>
#include <map>

struct Knot;
struct Cell;
struct Edge;

/**
* @struct Grid
* \brief Cтруктура сетки
*/
struct Grid
{
	Grid(const std::map<int, Knot>& points, const std::vector<Cell>& cells);
	Grid(const std::map<int, Knot>& points, const  std::vector<Cell>& cells, const  std::vector<Edge>& _edges);
	Grid(const std::map<int, Knot>& points, const std::vector<Cell>& cells, const std::vector<Edge>& _edges,
		std::map<int, Knot> _aPoints2D, std::vector<int> _fixPoint, double _speed, double _energy);
	Grid(Grid* _grid);
	// набор точек сетки
	std::map<int, Knot> aPoints;
	// набор спроецированных точек сетки
	std::map<int, Knot> aPoints2D;
	std::vector<Cell> aCells;

	//
	
	// скорость передвижения узлов
	double speed,
		// энергия сетки
		energy,
		// шаг для градиентного спуска
		dx,
		// функция показывающая состояние системы для градиентного спуска
		Fxy;

	std::vector<int> fixPoint;

	std::vector<Edge> edges;

	Grid operator=(Grid* grid);
};


#endif // GRID_H

