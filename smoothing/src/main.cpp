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

const std::string inputFile =  "../../../konus_down_diplom.k";
//const std::string outputFile = "../../../qdec_2D_halfquad_with_circlehole_res.txt";
const std::string outputFile = "C:/myprogram/geomView/install/bin/qdec_halfquad_with_circlehole_res.txt";
void main() 
{

	Grid* grid;

	const char* modelDir = "../../../konus.c3d";

	//сделать переменную define стрингом
	if (inputFile[inputFile.length() - 1] == 't')
	{
		QDec* qdec = ReadQDec();
		grid = ConvetQDecFromGrid(qdec);
	}
	else
	{
		grid = ReadFile();
	}
	//WriteFileQDec(grid,false);
	MbSplineSurface* srfNurbs = GetSrfNurbs(modelDir, grid);
	//MorphDiag(grid, true);
	StartIterations(grid, srfNurbs);
	//AngleSmoothing(grid, srfNurbs);
	WriteFileQDec(grid, false);

	int a = 0;
	delete grid;

}

