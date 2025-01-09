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

Grid* ConvetQDecFromGrid(QDec* qdec) {
	// создаем необходимые для Grid компоненты
	std::map<int, Knot> aPoints;
	std::vector<Cell> aCells;
	std::vector<Edge> edges;

	// в boundsApprox лежат точки расположенные на границе сетки, те фиксированные
	for (int i = 0; i < qdec->boundsApprox.size(); i++) {
		Knot knot = Knot(qdec->boundsApprox[i].x, qdec->boundsApprox[i].y, qdec->boundsApprox[i].z);
		knot.flagFix = true;
		aPoints.insert({ i, knot });
	}
	// в inPoints лежат все остальные точки
	for (int i = 0; i < qdec->inPoints.size(); i++) {
		Knot knot = Knot(qdec->inPoints[i].x, qdec->inPoints[i].y, qdec->inPoints[i].z);
		aPoints.insert({ i + qdec->boundsApprox.size(), knot });
	}
	// формирование ячеек и ребер
	for (int i = 0; i < qdec->quads.size(); i++) {
		std::vector<int> pointsIdForCell;
		std::vector<int> edgesIdForCell;
		double idealEdge = 0;
		double interv = 0;
		for (int j = 0; j < 4; j++) {
			int start = qdec->quads[i].edges[j][0];
			int finish = qdec->quads[i].edges[j][1];

			Edge e = Edge(qdec->edgesPointIndices[start], qdec->edgesPointIndices[finish]);

			if (start > finish) {
				std::swap(start, finish);
			}



			for (int i = start; i <= finish; i++) {
				e.idPointsIn.push_back(qdec->edgesPointIndices[i]);
			}
			edges.push_back(e);
			edgesIdForCell.push_back(edges.size() - 1);
			aPoints[qdec->edgesPointIndices[start]].incEdgesId.push_back(edges.size() - 1);
			aPoints[qdec->edgesPointIndices[finish]].incEdgesId.push_back(edges.size() - 1);

			aPoints[qdec->edgesPointIndices[start]].flagEdgePt1Pt2 = true;
			aPoints[qdec->edgesPointIndices[finish]].flagEdgePt1Pt2 = true;

			pointsIdForCell.push_back(qdec->edgesPointIndices[start]);
			interv += e.idPointsIn.size() - 1;
			idealEdge += GetLenghtEdge(aPoints, e);
		}

		Cell cell = Cell(pointsIdForCell);

		idealEdge /= interv;
		cell.m_idealEdge = idealEdge;
		cell.edges = edgesIdForCell;

		for (int i = 0; i < pointsIdForCell.size(); i++) {
			if (!(std::find(aPoints[pointsIdForCell[i]].incCellsId.begin(),
				aPoints[pointsIdForCell[i]].incCellsId.end(), aCells.size())
				!= aPoints[pointsIdForCell[i]].incCellsId.end())) {
				aPoints[pointsIdForCell[i]].incCellsId.push_back(aCells.size());
			} else {
				continue;
			}

		}

		aCells.push_back(cell);
	}


	Grid* grid = new Grid(aPoints, aCells, edges);

	return grid;
}