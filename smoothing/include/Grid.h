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
struct Grid {
	Grid(const std::map<int, Knot>& points, const std::vector<Cell>& cells);
	Grid(const std::map<int, Knot>& points, const  std::vector<Cell>& cells, const  std::vector<Edge>& _edges);
	Grid(const std::map<int, Knot>& points, const std::vector<Cell>& cells, const std::vector<Edge>& _edges,
		 std::map<int, Knot> _aPoints2D, std::vector<int> _fixPoint, double _speed, double _energy);
	Grid(Grid* _grid);
	// набор точек сетки
	std::map<int, Knot> aPoints;
	// набор спроецированных точек сетки
	std::map<int, Knot> aPoints2D;
	// набор ячеек
	std::vector<Cell> aCells;

	// скорость передвижения узлов
	double speed;
	// энергия сетки
	double	energy;
	// шаг для градиентного спуска
	double	dx;
	// функция показывающая состояние системы для градиентного спуска
	double Fxy;
	// шаг для перемещения (для градиентного спуска)
	double step;
	// 
	std::vector<double> dfPred;
	// id фиксированных узлов
	std::vector<int> fixPoint;
	// набор ребер
	std::vector<Edge> edges;
	// флаг показывающий надо ли менять направление обхода ячейки при переходе в 2D
	bool swap = false;

	Grid operator=(Grid* grid);
};


#endif // GRID_H

