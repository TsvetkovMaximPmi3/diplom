#ifndef CONVERT_H
#define CONVERT_H

#include <string>

/**
* \brief Проверить, является ли символ числом
* \param [in] str Cтрока 
* \return true-символ это число, false - символ не число
*/
bool IsNumber(const std::string& str);

double StringToDouble(const std::string& str);
#endif // CONVERT_H
