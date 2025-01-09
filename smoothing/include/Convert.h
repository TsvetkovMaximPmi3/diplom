#ifndef CONVERT_H
#define CONVERT_H

#include <string>

#include "Knot.h"
#include "Grid.h"
#include "Edge.h"
#include "Cell.h"

/**
* \brief Проверить, является ли символ числом
* \param [in] str Cтрока 
* \return true-символ это число, false - символ не число
*/
bool IsNumber(const std::string& str);

/**
* \brief Перевод строки в double
* \param [in] str Cтрока
* \return double
*/
double StringToDouble(const std::string& str);

Grid* ConvetQDecFromGrid(QDec* qdec);
#endif // CONVERT_H
