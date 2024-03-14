#pragma once
#include<vector>
#include<mb_cart_point3d.h>
#include <array>


struct QDec
{


	//Глобальные индексы точек ребра в обьединённом массиве {boundsApprox, inPoints} 
	//*EDGENODES сюда
	std::vector<size_t> edgesPointIndices;

	struct Quad
	{

		//*ELEMENTS сюда
		std::array<std::array<size_t, 2>, 4> edges; // 4 пары индексов {начало, конец} рёбер в массиве edgesPointIndices

	};
	//Координаты внутренних узлов многоугольника.
	//остаток точек
	std::vector<MbCartPoint3D> inPoints;
	//Список четырехугольников, покрывающих многоугольник.
	std::vector<Quad> quads;
	//Новая аппроксимация границы грани.
	//первые n Точек
	std::vector<MbCartPoint3D> boundsApprox;



};
