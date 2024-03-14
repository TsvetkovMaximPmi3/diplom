#include "RWFile.h"
#include "Algorithm.h"
#include "QDec.h"
#include <algorithm>
#include <vector>

/**
* \brief Cчитать .k файл и сформировать сетку Grid
* \return сетка
*/
Grid* ReadFile() 
{
   
    fin.open(inputFile.c_str());
    char buff[25];
    std::map<int, Knot> aPoints;
    std::vector<Cell> masCell;

    if (fin.is_open())
    {
        while (true)
        {
            fin >> buff;
            if (buff[1] == 'N')
            {
                continue;
            }
            if ((buff[1] == 'E') || (buff[0] == '#'))
            {
                break;
            }

            int id = atoi(buff);

            fin >> buff;
            double x = StringToDouble(buff);

            fin >> buff;
            double y = StringToDouble(buff);

            fin >> buff;
            double z = StringToDouble(buff);

            Knot pt =  Knot(x, y, z);
            aPoints.insert({id, pt});
        }

        while (true)
        {
            bool isError = false;

            std::vector<int> aPointsInd(4);
            fin >> buff;
            fin >> buff;
            for (size_t i = 0; i < 4; i++)
            {
                if (!(fin >> buff) || (buff[0] == '#') || (buff[1] == 'E'))
                {
                    isError = true;

                    break;
                }
                aPointsInd[i] = atoi(buff);
            }

            if (isError)
            {
                break;
            }

            Cell cell =  Cell(aPointsInd);

            masCell.push_back(cell);
        }
    }

    Grid *grid = new Grid(aPoints, masCell);
    return grid;
}

/**
* \brief Записать в файл .k id вершин, координаты вершин и порядок их соединения ячейки
* \param [in] grid Сетка
*/
void WriteFile(Grid* grid) 
{
    std::ofstream out;
    out.open(outputFile.c_str());
    if (out.is_open()) 
    {
        out << "*NODE" << std::endl;
       
    	for (std::map<int, Knot>::iterator it = grid->aPoints.begin(); it != grid->aPoints.end(); ++it)
        {	
			//if (it->second == nullptr)continue;
            out << it->first << ", " << grid->aPoints[it->first].pt.x << ", " << 
				grid->aPoints[it->first].pt.y << ", " << grid->aPoints[it->first].pt.z << std::endl;
			
        }
        
    	out << "*ELEMENT_SHELL" << std::endl;
        
    	size_t aCellsSize = grid->aCells.size();
        for (size_t i = 0; i < aCellsSize; i++)
		{
			
            out << i+1 << ", " << "0, " << grid->aCells[i].m_aPointsId[0] << ", " << 
				grid->aCells[i].m_aPointsId[1] << ", " << grid->aCells[i].m_aPointsId[2] << 
				", " << grid->aCells[i].m_aPointsId[3] << std::endl;
        }
        out << "*END" << std::endl;
    }
}




QDec* ReadQDec()
{
	std::ifstream fin;
	fin.open(inputFile.c_str());
	char buff[25];
	QDec* qdec = new QDec();
	/*
	std::map<int, Knot*> aPoints;
	std::vector<Cell*> masCell;
	std::vector<int> edgesIdPoints;
	std::vector<Edge*> edgesAll;
	if (fin.is_open())
	{
		int id = 0;
		fin >> buff;
		int fix = StringToDouble(buff);

		while (true)
		{
			fin >> buff;
			if (buff[1] == 'N')
			{
				continue;
			}
			if (buff[2] == 'D')
			{
				break;
			}

			//fin >> buff;
			double x = StringToDouble(buff);
			fin >> buff;
			double y = StringToDouble(buff);
			fin >> buff;
			double z = StringToDouble(buff);

			Knot* pt = new Knot(x, y, z);

			if (id <= fix)
			{
				pt.flagFix = true;
			}
			aPoints.insert({ id,pt });
			id++;

		}
	
		while (true)
		{

			fin >> buff;
			if (buff[1] == 'E')
			{
				break;
			}
			
			int point = StringToDouble(buff);

			edgesIdPoints.push_back(point);


		}
		while (true)
		{
			std::vector<std::string> buffs;
			std::vector<Edge*> edges;
			fin >> buff;

			if (buff[1] == 'E')
			{
				break;
			}
			std::vector<int> aPointsInd;

			int b = StringToDouble(buff);
			aPointsInd.push_back(edgesIdPoints[b]);
			aPoints[edgesIdPoints[b]]->flagQDec = true;

			
			fin >> buff;
			int c = StringToDouble(buff);
			aPointsInd.push_back(edgesIdPoints[edgesIdPoints[c]]);
			aPoints[edgesIdPoints[c]]->flagQDec = true;
			//aPoints[c]->idNext = b;
			idNextP(b, c, aPoints, edgesIdPoints,edges, edgesAll);

			fin >> buff;
			int r =StringToDouble(buff);
			//aPoints[r]->idNext = c;
			idNextP(c, r, aPoints,edgesIdPoints,edges, edgesAll);

			fin >> buff;
			int e = StringToDouble(buff);
			aPointsInd.push_back(edgesIdPoints[e]);
			aPoints[edgesIdPoints[e]]->flagQDec = true;
			//aPoints[e]->idNext = r;
			idNextP(r, e, aPoints, edgesIdPoints,edges, edgesAll);

			fin >> buff;
			int t = StringToDouble(buff);
			//aPoints[t]->idNext = e;
			idNextP(e, t, aPoints, edgesIdPoints,edges, edgesAll);

			fin >> buff;
			int q = StringToDouble(buff);
			//aPoints[q]->idNext = t;
			idNextP(t, q, aPoints, edgesIdPoints,edges, edgesAll);
			aPointsInd.push_back(edgesIdPoints[q]);
			aPoints[edgesIdPoints[q]]->flagQDec = true;


			fin >> buff;
			int w = StringToDouble(buff);
			//aPoints[w]->idNext = q;
			idNextP(q, w, aPoints, edgesIdPoints,edges, edgesAll);

			fin >> buff;
			int s = StringToDouble(buff);
			//aPoints[s]->idNext = w;
			idNextP(w, s, aPoints, edgesIdPoints, edges, edgesAll);
			Cell* cell = new Cell(aPointsInd);
			for (int i = 0; i < edges.size(); i++)
			{
				cell->edgesForQDec.push_back(edges[i]);

			}
			masCell.push_back(cell);

		}
	}

	Grid* grid = new Grid(aPoints, masCell,edgesAll);
	return grid;
	*/
	
	if (fin.is_open())
	{
		fin >> buff;
		int fixKnots = StringToDouble(buff);
		
		fin >> buff;
		if (buff[1] != 'N')throw - 1;
		
		int iter = 0;

		while (true)
		{
			fin >> buff;
			if (buff[2] == 'D')break;

			double x = StringToDouble(buff);
			fin >> buff;
			double y = StringToDouble(buff);
			fin >> buff;
			double z = StringToDouble(buff);
			MbCartPoint3D point(x, y, z);
			if (iter < fixKnots)
			{
				qdec->boundsApprox.push_back(point);
			}
			else
			{
				qdec->inPoints.push_back(point);
			}
			iter++;
		}
		while (true)
		{
			fin >> buff;
			if (buff[1] == 'E')break;
			qdec->edgesPointIndices.push_back((int)StringToDouble(buff));
		}

		while (true)
		{
			fin >> buff;
			if (buff[1] == 'E')break;
			std::array<std::array<size_t, 2>, 4> edges;
			edges[0][0] = (int)StringToDouble(buff);

			fin >> buff;
			edges[0][1] = (int)StringToDouble(buff);

			fin >> buff;
			edges[1][0] = (int)StringToDouble(buff);

			fin >> buff;
			edges[1][1] = (int)StringToDouble(buff);

			fin >> buff;
			edges[2][0] = (int)StringToDouble(buff);

			fin >> buff;
			edges[2][1] = (int)StringToDouble(buff);

			fin >> buff;
			edges[3][0] = (int)StringToDouble(buff);

			fin >> buff;
			edges[3][1] = (int)StringToDouble(buff);

			QDec::Quad q;
			q.edges = edges;
			qdec->quads.push_back(q);
		}
	}
	return qdec;
}

void WriteFileQDec(Grid* grid,bool App)
{

	//рисуем напряжение 
	double max = 0;
	for (auto edge : grid->edges)
	{
		max = std::max(std::abs(edge.napryazhenie), max);

	}
	for (auto edge : grid->edges)
	{
		edge.napryazhenie /= max;
	}

	std::ofstream out;
	static int count = 0;
	count++;
	out.open(outputFile.c_str(), App ? std::ios_base::app : std::ios_base::out);
	if (out.is_open())
	{
		out << "points : All" << std::endl;
		/*
		for (int i = 0; i < grid->edges.size(); i++)
		{
			if (!grid->edges[i].diag)
			{
				out << "(" << grid->aPoints[grid->edges[i].pt1()].pt.x << ", " 
					<< grid->aPoints[grid->edges[i].pt1()].pt.y << ", " 
					<< grid->aPoints[grid->edges[i].pt1()].pt.z << ")";

				if (grid->aPoints[grid->edges[i].pt1()].flagFix)
				{
					
					out << " 4 (0, 1, 0)" << std::endl;
				}
				else
				{
					
					out << " 4 (0, 0, 1)" << std::endl;
				}



				out << "(" << grid->aPoints[grid->edges[i].pt2()].pt.x << ", "
					<< grid->aPoints[grid->edges[i].pt2()].pt.y << ", "
					<< grid->aPoints[grid->edges[i].pt2()].pt.z << ")";

				if (grid->aPoints[grid->edges[i].pt2()].flagFix)
				{
					out << " 4 (0, 1, 0)" << std::endl;
				}
				else
				{

					out << " 4 (1, 0, 0)" << std::endl;
				}

			}
		}
		*/
		for (std::map<int, Knot>::iterator it = grid->aPoints.begin(); it != grid->aPoints.end(); ++it)
		{
			//if (!it->second.flagFix)continue;
			if(!it->second.diag)
			//if (it->first < 1389 || it->first > 1389)continue;
			//if (it->first == 1389 || it->first == 46 || it->first == 1533 || it->first == 1416) 
			{
				out << "(" << it->second.pt.x << ", " << it->second.pt.y << ", " << it->second.pt.z << ")";

				if (it->second.flagFix)
				{
					
					out << " 1 (0, 1, 0)" << std::endl;
					
				}
				else
				{
					if (it->second.peresech)
					{
						out << " 1 (0, 0, 1)" << std::endl;
					}
					else
					{
						if (it->second.pole)
						{
							out << " 5 (0, 1, 0)" << std::endl;
						}
						else
							out << " 1 (1, 0, 0)" << std::endl;
					}
				}

			}
			
		}
		
		//for (auto cell : grid->aCells)
		//{
		//	for (auto edge : cell.edges)
		//	{
		//		//grid->edges[edge].
		//	}
		//}

		out << "lines : All" << std::endl;
		
		for (int i = 0; i < grid->edges.size();i++)
		{
			//grid->edges[i].napryazhenie = 0;
			if (grid->edges[i].diag)continue;
			//if (grid->edges[i].diag)continue;
			//if (grid->edges[i].napryazhenie != -1)continue;
			if (grid->edges[i].idPointsIn.size() == 0)
			{
				out << "(" << grid->aPoints[grid->edges[i].pt1()].pt.x << ", "
					<< grid->aPoints[grid->edges[i].pt1()].pt.y << ", "
					<< grid->aPoints[grid->edges[i].pt1()].pt.z << ") ";
				out << "(" << grid->aPoints[grid->edges[i].pt2()].pt.x << ", "
					<< grid->aPoints[grid->edges[i].pt2()].pt.y << ", "
					<< grid->aPoints[grid->edges[i].pt2()].pt.z << ") ";
				//if (!grid->edges[i].intersection)
				//	out << "(0, 0, 1)" << std::endl;
				//else
				//	out << "(0, 1, 0)" << std::endl;
				if (grid->edges[i].napryazhenie > 0)
					out << "(" << std::pow(grid->edges[i].napryazhenie,0.33) << ", " << 1 - std::pow(std::abs(grid->edges[i].napryazhenie),0.33) << ", 0)" << std::endl;
				else
					out << "(0, " << 1 - std::pow(std::abs(grid->edges[i].napryazhenie),0.33) << ", " << std::pow(std::abs(grid->edges[i].napryazhenie),0.33) << ")" << std::endl;
			}
			else
			{
				for (int j = 0; j < grid->edges[i].idPointsIn.size() - 1; j++)
				{
					//if (grid->edges[i].diag)continue;
					out << "(" << grid->aPoints[grid->edges[i].idPointsIn[j]].pt.x << ", "
						<< grid->aPoints[grid->edges[i].idPointsIn[j]].pt.y << ", "
						<< grid->aPoints[grid->edges[i].idPointsIn[j]].pt.z << ") ";
					out << "(" << grid->aPoints[grid->edges[i].idPointsIn[j + 1]].pt.x << ", "
						<< grid->aPoints[grid->edges[i].idPointsIn[j + 1]].pt.y << ", "
						<< grid->aPoints[grid->edges[i].idPointsIn[j + 1]].pt.z << ") ";
					if (grid->edges[i].diag)
					{
						//out << "(0, 0.5, 0.5)" << std::endl;
						if (grid->edges[i].napryazhenie > 0)
							out << "(" << std::pow(grid->edges[i].napryazhenie,0.33) << ", " << 1 - std::pow(std::abs(grid->edges[i].napryazhenie),0.33) << ", 0)" << std::endl;
						else
							out << "(0, " << 1 - std::pow(std::abs(grid->edges[i].napryazhenie),0.33) << ", " << std::pow(std::abs(grid->edges[i].napryazhenie),0.33) << ")" << std::endl;
						
					}
					else
					{
						//if (!grid->edges[i].intersection)
						//	out << "(0, 0, 1)" << std::endl;
						//else
						//	out << "(0, 1, 0)" << std::endl;
						if (grid->edges[i].napryazhenie > 0)
							out << "(" << std::pow(grid->edges[i].napryazhenie,0.33) << ", " << 1 - std::pow(std::abs(grid->edges[i].napryazhenie),0.33) << ", 0)" << std::endl;
						else
							out << "(0, " << 1 - std::pow(std::abs(grid->edges[i].napryazhenie),0.33) << ", " << std::pow(std::abs( grid->edges[i].napryazhenie),0.33) << ")" << std::endl;
					}
					

					//out << it->first << "-" << it->second->idNext[i] << std::endl;
				}
			}

		}


		//for (int i = 0; i < aCellsSize; i++)
		//{
		//	if (i == 75) {
		//		out << "points : cell" << i << std::endl;
		//		for (int j = 0; j < 4; j++)
		//		{
		//			out << "(" << grid->aCells[i].GetKnots()[j].pt.x << ", " << grid->aCells[i].GetKnots()[j].pt.y << ", " << grid->aCells[i].GetKnots()[j].pt.z << ")";
		//
		//			out << " 8 (0, 1, 1)" << std::endl;
		//
		//
		//		}
		//	}
		//}
		
		out << "vectors : All" << std::endl;

		for (std::map<int, Knot>::iterator it = grid->aPoints.begin(); it != grid->aPoints.end(); ++it)
		{
		
			if (it->second.flagEdgePt1Pt2) {
				out << "(" << it->second.pt.x << ", "
					<< it->second.pt.y << ", "
					<< it->second.pt.z << ") ";
				out << "(" << it->second.pt.x + it->second.moveVect.x << ", "
					<< it->second.pt.y + it->second.moveVect.y << ", "
					<< it->second.pt.z + it->second.moveVect.z << ") ";
				out << "(0, 1, 0)" << std::endl;
			}

				//out << it->first << "-" << it->second->idNext[i] << std::endl;
		
		}

		/*
		for (auto e : grid->edges)
		{
			out << "(" << grid->aPoints[e.pt1()].pt.x << ", "
				<< grid->aPoints[e.pt1()].pt.y << ", "
				<< grid->aPoints[e.pt1()].pt.z << ") ";
			out << "(" << grid->aPoints[e.pt1()].pt.x + grid->aPoints[e.pt1()].moveVect.x << ", "
				<< grid->aPoints[e.pt1()].pt.y + grid->aPoints[e.pt1()].moveVect.y << ", "
				<< grid->aPoints[e.pt1()].pt.z + grid->aPoints[e.pt1()].moveVect.z<< ") ";
			if (e.napryazhenie > 0)
				out << "(" << std::pow(e.napryazhenie, 0.33) << ", " << 1 - std::pow(std::abs(e.napryazhenie), 0.33) << ", 0)" << std::endl;
			else
				out << "(0, " << 1 - std::pow(std::abs(e.napryazhenie), 0.33) << ", " << std::pow(std::abs(e.napryazhenie), 0.33) << ")" << std::endl;

			out << "(" << grid->aPoints[e.pt2()].pt.x << ", "
				<< grid->aPoints[e.pt2()].pt.y << ", "
				<< grid->aPoints[e.pt2()].pt.z << ") ";
			out << "(" << grid->aPoints[e.pt2()].pt.x + grid->aPoints[e.pt2()].moveVect.x << ", "
				<< grid->aPoints[e.pt2()].pt.y + 10* grid->aPoints[e.pt2()].moveVect.y << ", "
				<< grid->aPoints[e.pt2()].pt.z + 10* grid->aPoints[e.pt2()].moveVect.z << ") ";
			if (e.napryazhenie > 0)
				out << "(" << std::pow(e.napryazhenie, 0.33) << ", " << 1 - std::pow(std::abs(e.napryazhenie), 0.33) << ", 0)" << std::endl;
			else
				out << "(0, " << 1 - std::pow(std::abs(e.napryazhenie), 0.33) << ", " << std::pow(std::abs(e.napryazhenie), 0.33) << ")" << std::endl;

		}
		
		int a = 0;*/
	}

}