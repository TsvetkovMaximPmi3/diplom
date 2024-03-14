#ifndef READMODEL_H
#define READMODEL_H

#include <string>
#include <sstream>

#include <conv_exchange_settings.h>
#include <conv_model_exchange.h>
#include <tool_cstring.h>
#include <io_memory_buffer.h>
#include <io_tape.h>
#include <conv_exchange_settings.h>
#include <tool_enabler.h>

#include <solid.h>
#include <model.h>
#include <mb_cart_point3d.h>
#include <curve3d.h>

#include "Grid.h"
#include "Knot.h"

/**
* \brief Перевести строку в нижний регистр
* \param[in] str Строка для перевода
*/ 
static void ToLower(std::string& str);

/**
* \brief Сравнить расширенение файла с заданным
* \param[in] fullName Строка, которую сравниваем
* \param[in] extName Строка с расширением
*/
bool SameExtension(const char* fullName, const char* extName);

/**
* \brief Инициализировать библиотеку C3D
*/
void initC3D();

/**
* \brief Считать модель
* \param[in] fileName Имя модели
* \param[in] model Модель
* \return true - успешное считывание, false- не успешное считывание
*/
bool ReadModel(const char* fileName, MbModel* model);

/**
* \brief Вернуть NURBS поверхность в модели
* \param [in] ModelDir Дириктория модели
* \param [in, out] grid Сетка
* \return NURBS поверхность 
*/
MbSplineSurface* GetSrfNurbs(const char* ModelDir, Grid* grid);

/**
* \brief Вернуть набор фиксированных узлов
* \param [in] face Грань
* \param [in] grid Сетка
* \return Набор фиксированных узлов
*/
std::map<int, bool> GetFixKnots(const MbFace* face, Grid* grid, const MbSurface* srf);
#endif //READMODEL_H