#include <surf_spline_surface.h>
#include "QDec.h"
#include "Grid.h"
#include "Algorithm.h"
#include "ReadModel.h"
#include "RWFile.h"
#include"QDecFunc.h"

#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>

// ���� ��� ���������� �����
const std::string inputFile =  "../../../qdec_2D_halfquad_with_circlehole.txt";
//const std::string inputFile = "../../../konus_down_diplom.k";

// ���� ��� ������ �����
//const std::string outputFile = "../../../qdec_2D_halfquad_with_circlehole_res.txt";
const std::string outputFile = "C:/myprogram/geomView/install/bin/qdec_halfquad_with_circlehole_res.txt";

void main() 
{

	Grid* grid;
	// ���� ��� ���������� ������
	const char* modelDir = "../../../2D-halfquad_with_circlehole.step";
	//const char* modelDir = "../../../konus.c3d";
	// ���� ���� � ����������� txt, �� ���������������, ��� ��� ���� �� �������� QDec
	if (inputFile[inputFile.length() - 1] == 't')
	{
		// ������ ��������� QDec
		QDec* qdec = ReadQDec();
		// ����������� �� ��������� QDec � Grid
		grid = ConvetQDecFromGrid(qdec);
	}
	else
	{
		// ������ ����� �� �����
		grid = ReadFile();
	}

	// ��������� MbSplineSurface �� ������ 
	MbSplineSurface* srfNurbs = GetSrfNurbs(modelDir, grid);
	
	// ����� ��������� �����������
	StartIterations(grid, srfNurbs);

	// ������� ����� � ����
	WriteFileQDec(grid, false);

	delete grid;
}

