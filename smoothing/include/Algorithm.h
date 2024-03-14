#ifndef ALGORITHM_H
#define ALGORITHM_H

class MbSplineSurface;


#include "Grid.h"
#include "RWFile.h"


#define EPSILON  0.00001

#define KCRIT 0.5

void edgeIntersection(Grid* grid, Cell& cell);
/**
* \brief Вычислить вектор перемещения для каждого узла
* \param [in, out] grid Сетка
*/
void CalculateMoveVect(Grid* grid);

/**
* \brief Вычислить точку на прямой на определенной дистанции
* \param [in] pt1 Первая точка прямой
* \param [in] pt2 Вторая точка прямой
* \param [in] distance Дистанция на которой надо найти точку
* \return вектор до найденной точки
*/
std::pair<MbVector3D, MbVector3D> CalculateLinePoint(Grid* grid,  Edge& e, double distance);

/**
* \brief Вычислить вектор перемещения для конкретных узлов
* \param [in] grid Сетка
* \param [in] pt1Ind Индекс точки в ячейке
* \param [in] pt2Ind Индекс точки в ячейке c которой соединена первая
* \param [in] cellInd Индекс ячейки
*/
void CalculateMoveVecForPoint(Grid* grid,  Edge& edge, double idealEdge);

/**
* \brief Спроецировать 3D точки в 2D
* \param [in, out] grid Сетка
* \param [in] srfNurbs Поверхность
*/
void ProectionOn2D(Grid* grid, MbSplineSurface* srfNurbs);

/**
* \brief Вычислить новые координаты узлов в 3D
* \param [in, out] grid Сетка
* \param [in] srfNurbs Поверхность
*/
void CalculateMoveKnots(Grid* grid, MbSplineSurface* srfNurbs);

/**
* \brief Запустить итерации по перемещению узлов
* \param [in, out] grid Сетка
* \param [in] srfNurbs Поверхность
*/
void StartIterations(Grid* grid, MbSplineSurface* srfNurbs);

MbVector3D CalculateLinePointVec(Grid* grid, int index1, int index2, double distance);
void AngleSmoothing(Grid* grid, MbSplineSurface* srfNurbs);

size_t Inc(size_t ind, int delta, size_t size);

Grid* ConvetQDecFromGrid(QDec* qdec);
double GetLenghtEdge(std::map<int, Knot>& aPoints, const Edge& edge);
int InsertInMapInEnd(Grid* grid, Knot pt);
struct Edge
{
	int pt1() const
	{
		if (idPointsIn.size() == 0)throw - 1;
		return idPointsIn[0];
	}
	int pt2() const
	{
		if (idPointsIn.size() == 0)throw - 1;
		return idPointsIn[idPointsIn.size()-1];
	}
	std::vector<int> includedCells;
	double dx;
	bool diag;
	std::vector<int> idPointsIn;

	void reverseEdge()
	{
		std::reverse(idPointsIn.begin(), idPointsIn.end());	
		
	}
	Edge(int _pt1, int _pt2)
	{
		//idPointsIn.insert(idPointsIn.begin(), _pt1);
		//idPointsIn.push_back(_pt2);
		diag = false;
		intersection = false;
	}

	bool intersection;
	double napryazhenie;
	MbVector3D tmpVecPt1;
	MbVector3D tmpVecPt2;

	Edge() {}
};

#endif //ALGORITHM_H