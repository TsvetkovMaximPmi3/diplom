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

// файл для считывания сетки
const std::string inputFile =  "../../../qdec_2D_halfquad_with_circlehole.txt";
//const std::string inputFile = "../../../konus_down_diplom.k";

// файл для записи сетки
//const std::string outputFile = "../../../qdec_2D_halfquad_with_circlehole_res.txt";
const std::string outputFile = "C:/myprogram/geomView/install/bin/qdec_halfquad_with_circlehole_res.txt";

void main() 
{

	Grid* grid;
	// файл для считывания модели
	const char* modelDir = "../../../2D-halfquad_with_circlehole.step";
	//const char* modelDir = "../../../konus.c3d";
	// если файл с разрешением txt, то подразумевается, что это файл со структой QDec
	if (inputFile[inputFile.length() - 1] == 't')
	{
		// чтение структуры QDec
		QDec* qdec = ReadQDec();
		// конвертация из структуры QDec в Grid
		grid = ConvetQDecFromGrid(qdec);
	}
	else
	{
		// чтение сетки из файла
		grid = ReadFile();
	}

	// получение MbSplineSurface из модели 
	MbSplineSurface* srfNurbs = GetSrfNurbs(modelDir, grid);
	
	// старт алгоритма сглаживания
	StartIterations(grid, srfNurbs);

	// записть сетки в файл
	WriteFileQDec(grid, false);

	delete grid;
}

