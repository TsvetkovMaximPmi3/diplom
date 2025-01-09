#ifndef RWFILE_H
#define RWFILE_H

#include <fstream>

#include "Grid.h"
#include "Cell.h"
#include "Convert.h"
#include "Knot.h"
#include"QDec.h"

const extern std::string inputFile;
const extern std::string outputFile;

/**
* \brief Cчитать .k файл и сформировать сетку Grid
* \return grid сетка
*/
Grid* ReadFile();

/**
* \brief Записать в файл .k id вершин, координаты вершин и порядок их соединения ячейки
* \param [in] grid Сетка
*/
void WriteFile(Grid* grid);

/**
* \brief Записать в файл .k id вершин, координаты вершин и порядок их соединения ячейки для структуры QDec
* \param [in] grid Сетка
*/
void WriteFileQDec(Grid* grid, bool App);

/**
* \brief Cчитать .k файл и сформировать структуру QDec
* \return QDec сетка
*/
QDec* ReadQDec();
#endif //RWFILE_H