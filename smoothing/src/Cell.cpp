#include "Cell.h"
#include "Grid.h"
#include "Knot.h"
#include"Algorithm.h"
#include "Edge.h"

/**
* \brief Вернуть вектор координат точек ячейки
* \return Вектор точек
*/
std::vector<Knot> Cell::GetKnots()
{
	std::vector<Knot> res;
	size_t m_aPointsIdSize = m_aPointsId.size();
	for (size_t i = 0; i < m_aPointsIdSize; i++)
	{
		res.push_back(m_grid->aPoints[m_aPointsId[i]]);
	}
	
	return res;
}

Cell::Cell(std::vector<int> aPointsInd) 
{
	m_aPointsId = aPointsInd;

	if((m_aPointsId[0] == m_aPointsId[1]) || (m_aPointsId[0] == m_aPointsId[2]) ||
		(m_aPointsId[0] == m_aPointsId[3]) || (m_aPointsId[1] == m_aPointsId[2]) ||
		(m_aPointsId[1] == m_aPointsId[3]) || (m_aPointsId[2] == m_aPointsId[3]))
	{
		isTriangel = true;
	}
	else
	{
		isTriangel = false;
	}
	processed = false;
}

/**
* \brief Установить "идеальные" ребро и диагональ
*/
void Cell::InitIdealEdgeAndDiag() 
{
	std::vector<Knot> res = GetKnots();
	m_idealDiag = 0;
	m_idealEdge = 0;

	
	m_idealEdge += sqrt(pow(res[0].pt.x - res[1].pt.x, 2) + 
		pow(res[0].pt.y - res[1].pt.y, 2) + pow(res[0].pt.z - res[1].pt.z, 2));
	m_idealEdge += sqrt(pow(res[1].pt.x - res[2].pt.x, 2) + 
		pow(res[1].pt.y - res[2].pt.y, 2) + pow(res[1].pt.z - res[2].pt.z, 2));
	m_idealEdge += sqrt(pow(res[2].pt.x - res[3].pt.x, 2) + 
		pow(res[2].pt.y - res[3].pt.y, 2) + pow(res[2].pt.z - res[3].pt.z, 2));
	m_idealEdge += sqrt(pow(res[3].pt.x - res[0].pt.x, 2) + 
		pow(res[3].pt.y - res[0].pt.y, 2) + pow(res[3].pt.z - res[0].pt.z, 2));
	m_idealEdge /= 4;
	

	if ((res[2].pt.x == res[3].pt.x) && (res[2].pt.y == res[3].pt.y) && 
		(res[2].pt.z == res[3].pt.z)) 
	{
		return;
	}
	
	m_idealDiag += sqrt(pow(res[0].pt.x - res[2].pt.x, 2) + 
		pow(res[0].pt.y - res[2].pt.y, 2) + pow(res[0].pt.z - res[2].pt.z, 2));
	m_idealDiag += sqrt(pow(res[1].pt.x - res[3].pt.x, 2) + 
		pow(res[1].pt.y - res[3].pt.y, 2) + pow(res[1].pt.z - res[3].pt.z, 2));
	m_idealDiag /= 2;
	
}

void Cell::InitIntervals(Grid* grid)
{
	double lenght = 0;
	double intervals = 0;
	
	for (auto edge : edgesDiag)
	{
		lenght += GetLenghtEdge(grid->aPoints, grid->edges[edge]);
		intervals += grid->edges[edge].idPointsIn.size() - 1;
	}
	m_idealDiag = lenght / intervals;
	lenght = 0;
	intervals = 0;
	for (auto edge : edges)
	{
		lenght += GetLenghtEdge(grid->aPoints, grid->edges[edge]);
		intervals += grid->edges[edge].idPointsIn.size() - 1;
	}

	m_idealEdge = lenght / intervals;
}
