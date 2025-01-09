#pragma once
#include <vector>

#include <mb_cart_point3d.h>

struct Edge {
	// ������� ������ ����� �����
	int pt1() const;
	// ������� ��������� ����� �����
	int pt2() const;
	// id ����� � ������� �������� �����
	std::vector<int> includedCells;
	// ���� ������������ �������� �� ����� ������������
	bool diag;
	// id ����� ���������� � �����
	std::vector<int> idPointsIn;
	// "�����������" �����
	void reverseEdge();
	Edge(int _pt1, int _pt2);
	// ���� ������������ ������������ �� �����
	bool intersection;
	// ���������� �� �����(������� �� ����� �������� ����������� ����� �����)
	double napryazhenie;
	MbVector3D tmpVecPt1;
	MbVector3D tmpVecPt2;

	Edge() {}
};