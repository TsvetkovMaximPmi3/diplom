#include "Grid.h"
#include <cur_hermit3d.h>
#include <action_curve3d.h>
#include"Algorithm.h"
struct Edge;

/**
* \brief ���������� ������� �����
* \param [in] grid �����
* \param [in] Edge �����
*/
void LineMorphing(Grid* grid,const Edge& edge);

/**
* \brief ���������� ������� �����
* \param [in] grid �����
* \param [in] flag true - ���� ���� ����������� ���������, false - ���� ���� ��������� ������������
*/
void MorphDiag(Grid* grid,bool flag);

/**
* \brief ���������� ������� �����
* \param [in] grid �����
* \param [in] edges ����� ������
* \param [in] flag true - ���� ���� ����������� ���������, false - ���� ���� ��������� ������������
* \return [in] vector<vector<int>> ����� ��������� ������
*/
std::vector<std::vector<int>> addPointMatrx(Grid* grid,  std::vector<Edge>& edges, bool create);

/**
* \brief ������������� ����� �� |intervals| ���������� �����
* \param [in] grid �����
* \param [in] edge ����� 
* \param [in] intervals ���������� ����������
*/
void ReSplitEdge(Grid* grid, Edge& e, int intervals);