#pragma once
#include<vector>
#include<mb_cart_point3d.h>
#include <array>


struct QDec
{


	//���������� ������� ����� ����� � ����������� ������� {boundsApprox, inPoints} 
	//*EDGENODES ����
	std::vector<size_t> edgesPointIndices;

	struct Quad
	{

		//*ELEMENTS ����
		std::array<std::array<size_t, 2>, 4> edges; // 4 ���� �������� {������, �����} ���� � ������� edgesPointIndices

	};
	//���������� ���������� ����� ��������������.
	//������� �����
	std::vector<MbCartPoint3D> inPoints;
	//������ �����������������, ����������� �������������.
	std::vector<Quad> quads;
	//����� ������������� ������� �����.
	//������ n �����
	std::vector<MbCartPoint3D> boundsApprox;



};
