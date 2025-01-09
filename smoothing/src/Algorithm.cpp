#include <queue>
#include<cmath>
#include <math.h>
#include<fstream>
#include<iostream>
#include <iomanip>
#include <model.h>
#include <surf_spline_surface.h>
#include<mb_vector3d.h>
#include "Algorithm.h"
#include "Knot.h"
#include "Cell.h"
#include "RWFile.h"
#include "Grid.h"
#include "QDecFunc.h"
#include "Edge.h"
#include "AngleSmoothing.h"

int iteration = 0;
int forOpenFile = 0;
const double EPS = 1E-9;
struct Knot;


/**
* \brief Спроецировать 3D точки в 2D
* \param [in, out] grid Сетка
* \param [in] srfNurbs Поверхность
*/
void ProectionOn2D(Grid* grid, MbSplineSurface* srfNurbs) {

	MbCartPoint tch;

	double uMin = srfNurbs->GetUMin(),
		uMax = srfNurbs->GetUMax(),
		vMin = srfNurbs->GetVMin(),
		vMax = srfNurbs->GetVMax();

	MbCartPoint3D pt11;
	srfNurbs->PointOn(uMin, vMin, pt11);
	for (std::map<int, Knot>::iterator it = grid->aPoints.begin(); it != grid->aPoints.end(); ++it) {
		srfNurbs->NearPointProjection(it->second.pt, tch.x, tch.y, false);

		grid->aPoints2D.insert({ it->first, Knot(tch.x, tch.y) });
	}
}

/**
* \brief Вычислить новые координаты узлов в 3D
* \param [in, out] grid Сетка
* \param [in] srfNurbs Поверхность
*/
void CalculateMoveKnots(Grid* grid, MbSplineSurface* srfNurbs) {

	for (std::map<int, Knot>::iterator it = grid->aPoints.begin(); it != grid->aPoints.end(); ++it) {
		if (grid->aPoints[it->first].flagFix || grid->aPoints[it->first].moveVect == MbVector3D(0, 0, 0)) {
			continue;
		}

		MbVector3D vec = grid->speed*0.5*grid->aPoints[it->first].moveVect,
			vecU,
			vecV;

		srfNurbs->DeriveU(grid->aPoints2D[it->first].pt.x, grid->aPoints2D[it->first].pt.y, vecU);
		srfNurbs->DeriveV(grid->aPoints2D[it->first].pt.x, grid->aPoints2D[it->first].pt.y, vecV);

		//проекция вектора перемещения на вектора U V
		double u = vecU * vec / vecU.Length2();
		double v = vecV * vec / vecV.Length2();

		grid->aPoints2D[it->first].pt.x += u;
		grid->aPoints2D[it->first].pt.y += v;


		srfNurbs->PointOn(grid->aPoints2D[it->first].pt.x, grid->aPoints2D[it->first].pt.y,
						  grid->aPoints[it->first].pt);


		grid->aPoints[it->first].moveVect = MbVector3D(0, 0, 0);



	}

	for (auto& cell : grid->aCells) {
		//edgeIntersection(grid, cell);
		cell.InitIntervals(grid);

	}
}






/**
* \brief Запустить итерации по перемещению узлов
* \param [in, out] grid Сетка
* \param [in] srfNurbs Поверхность
*/
void StartIterations(Grid* grid, MbSplineSurface* srfNurbs) {

	// создание диагоналей для ячейки
	MorphDiag(grid, true);

	// тестовая функция для решения проблем на редких моделях где некотрые точки не фиксировались
	// причины не фиксирования точек не вяснены
	TmpFixKnots(grid);
	// проекция сетки на 2D плоскость
	ProectionOn2D(grid, srfNurbs);
	// dx - переменная для расчета производной в алгоритме градиентного спуска
	grid->dx = CalcDx(grid);

	// step - переменная для шага в алгоритме градиентного спуска
	grid->step = grid->dx * 30;

	//проверяем надо ли менять направление обхода ячеек в сетке (достаточно проверить одну ячейку)
	grid->swap = SwapLR(grid, 0);

	// запускаем цикл итераций алгоритма. Выход из цикла пока обеспечивается количеством итераций,
	// можно добавить другие критерии выхода
	while (true) {

		iteration++;
		// силовой алгоритм сглаживания 
		CalculateMoveVect(grid);

		// алгоритм сглаживания по углам
		//CalculateMoveVectAngle(grid, srfNurbs);

		// градиентный спуск (ахтунг: не работает)
		//Gradient(grid, srfNurbs);

		// алгоритмы расставили вектора перемещений для углов, далее в функции CalculateMoveKnots
		// происходит перемещение узлов
		CalculateMoveKnots(grid, srfNurbs);

		std::cout << "Interation : " << iteration << std::endl;

		if (iteration > 0)
			break;
	}
}


