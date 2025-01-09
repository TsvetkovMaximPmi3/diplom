#include "Grid.h"
#include "Cell.h"
#include "Algorithm.h"

Grid::Grid(const std::map<int, Knot>& points, const std::vector<Cell>& cells)
{
	aPoints = points;
	aCells = cells;
	speed = 0.1;
	energy = 0;
	Fxy = 0;
	size_t cellsSize = cells.size();
	for (size_t i = 0; i < cellsSize; i++)
	{
		aCells[i].m_grid = this;
		aCells[i].InitIdealEdgeAndDiag();
	}
}

Grid::Grid(const std::map<int, Knot>& points, const  std::vector<Cell>& cells, const std::vector<Edge>& _edges)
{

	aPoints = points;
	aCells = cells;
	speed = 0.1;
	energy = 0;
	Fxy = 0;

	size_t cellsSize = cells.size();
	for (size_t i = 0; i < cellsSize; i++)
	{
		aCells[i].m_grid = this;
		bool initEdgeAndDiag = true;
		//for (int j = 0; j < aCells[i].edges.size(); j++)
		//{
		//	
		//	if (_edges[aCells[i].edges[j]].idPointsIn.size() > 2)
		//	{
		//		initEdgeAndDiag = false;
		//		break;
		//	}
		//}
		//if (initEdgeAndDiag)
		//{
		//	aCells[i].InitIdealEdgeAndDiag();
		//}
	}
	edges = _edges;
}

Grid::Grid(const std::map<int, Knot>& points, const std::vector<Cell>& cells, const std::vector<Edge>& _edges,
	std::map<int, Knot> _aPoints2D, std::vector<int> _fixPoint, double _speed, double _energy)
{
	 aPoints = points;
	 aPoints2D = _aPoints2D;
	 aCells = cells;
	 edges = _edges;
	 fixPoint = _fixPoint;
	 speed = _speed;
	energy = _energy;
	Fxy = 0;
}
Grid::Grid(Grid * grid)
{
	 aPoints = grid->aPoints;
	aPoints2D = grid->aPoints2D;
	aCells = grid->aCells;
	 edges = grid->edges;
	 fixPoint = grid->fixPoint;
	speed = grid->speed;
	energy = grid->energy;
	Fxy = 0;
	swap = grid->swap;
	
}
Grid Grid::operator=(Grid* grid)
{
	std::map<int, Knot> aPoints = grid->aPoints;
	std::map<int, Knot> aPoints2D = grid->aPoints2D;
	std::vector<Cell> aCells = grid->aCells;
	std::vector<Edge> edges = grid->edges;
	std::vector<int> fixPoint = grid->fixPoint;
	double speed = grid->speed;
	energy = grid->energy;
	Fxy = 0;
	return Grid(aPoints, aCells, edges, aPoints2D, fixPoint, speed, energy);
}