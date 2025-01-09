#include "Grid.h"
#include <cur_hermit3d.h>
#include <action_curve3d.h>
#include"Algorithm.h"
struct Edge;

/**
* \brief Билинейный морфинг ребра
* \param [in] grid Сетка
* \param [in] Edge ребро
*/
void LineMorphing(Grid* grid,const Edge& edge);

/**
* \brief Билинейный морфинг ребра
* \param [in] grid Сетка
* \param [in] flag true - если надо пересоздать диагонали, false - если надо подвинуть существующие
*/
void MorphDiag(Grid* grid,bool flag);

/**
* \brief Билинейный морфинг ребра
* \param [in] grid Сетка
* \param [in] edges ребра ячейки
* \param [in] flag true - если надо пересоздать диагонали, false - если надо подвинуть существующие
* \return [in] vector<vector<int>> точки диагонали ячейки
*/
std::vector<std::vector<int>> addPointMatrx(Grid* grid,  std::vector<Edge>& edges, bool create);

/**
* \brief Переразбиение ребра на |intervals| количество точек
* \param [in] grid Сетка
* \param [in] edge ребро 
* \param [in] intervals количество интервалов
*/
void ReSplitEdge(Grid* grid, Edge& e, int intervals);