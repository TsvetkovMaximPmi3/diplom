#include <sstream>

#include "Convert.h"

/**
* \brief Проверить, является ли числом символ
* \param [in] str Cтрока
* \return true или false
*/
bool IsNumber(const std::string& str)
{
    std::istringstream ist(str);
    int number = 0;
    //Если вывод из потока в число прошел успешно и в потоке нет символов кроме пробелов - возвращаем true.
    return (ist >> number) && (ist >> std::ws).eof();
}

/**
* \brief Перевести string в double
* \param [in] str Cтрока
* \return Значение double
*/
double StringToDouble(const std::string& str)
{
    std::istringstream i(str);
    double number;
    if (!(i >> number))
    {
        return 0;
    }
    return number;
}