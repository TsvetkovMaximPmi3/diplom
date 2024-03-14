#include "ReadModel.h"
#include "Cell.h"
#include "Algorithm.h"
#include <surf_spline_surface.h>
#include <assembly.h>
#define C3D_EXT   _T("c3d")

/**
* \brief Перевести строку в нижний регистр
* \param[in] str Строка для перевода
*/
static void ToLower(std::string& str)
{
	size_t strSize = str.size();
	for (size_t i = 0; i < strSize; i++)
	{
		if (str[i] >= 'A' && str[i] <= 'Z')
		{
			str[i] = 'a' + (str[i] - 'A');
		}
	}
}

/**
* \brief Сравнить расширенение файла с заданным
* \param[in] fullName Строка которую сравниваем
* \param[in] extName Строка с расширением
*/
bool SameExtension(const char* fullName, const char* extName)
{
	std::vector<std::string> tokens;
	std::stringstream ss(fullName);
	std::string item;

	while (getline(ss, item, '.'))
	{
		tokens.push_back(item);
	}
	if (tokens.size() < 2)
	{
		return false;
	}
	std::string fileExt(tokens.back());
	ToLower(fileExt);
	return _tcscmp(fileExt.c_str(), extName) == 0;
}

/**
* \brief Инициализировать библиотеку C3D
*/
void initC3D()
{
	// инициализация лицензии библиотеки
	std::string str0;
	std::string str1;
#ifndef _WIN32
	str0 = "RFNC-VNIIEF-Linux..[cnv][mdl][slv]";
	str1 = "oQ87pGPIKogbPdXjYpBx9EZ31hFDuOKtXDreJvn7J6FN+1sH7jXF9ehxWtFBrev1lTD2pTY6b2/ryBajxcTCaw==";
#else
	str0 = "RFNC-VNIIEF..[cnv][mdl][slv]";
	str1 = "FIExwoN9mosyHyjBCGejLJ8lkYhqdJMqcgIxiRyyESqlcfjjXVQNDyvqKxVxQpZv7X09FhpqpmVqRmLWWVuuvg==";
#endif
	EnableMathModules(str0.c_str(), (int)str0.length(), str1.c_str(), (int)str1.length());
}

/**
* \brief Считать модель
* \param[in] fileName Имя модели
* \param[in] model Модель
* \return true - модель считана успешно, false - не успешно
*/
bool ReadModel(const char* fileName, MbModel* model)
{
	bool result = false;

	if (SameExtension(fileName, C3D_EXT))
	{
		reader inputFile(::createiobuf(fileName), io::in);
		if (inputFile.good())
		{
			result = ::ReadModelItems(inputFile, *model);
		}
	}
	else
	{
		ConvConvertorProperty3D prop;
		prop.fileName = fileName;
		prop.enableAutostitch = false;
		prop.joinSimilarFaces = false;
		c3d::path_string path = fileName;
		MbeConvResType ioResult = c3d::ImportFromFile(*model, path, &prop, nullptr);
		result = (ioResult == cnv_Success);
	}

	return result;
}


/**
* \brief Вернуть набор граничных узлов
* \param [in] face Грань
* \param [in] grid Сетка
* \return Набор граничных узлов
*/
std::map <int, bool> GetFixKnots(const MbFace* face, Grid* grid, const MbSurface* srf)
{	//найти ребро длинны 0 и взять начальную точку это полюсная
	//grid->aPoints2D[149].pt.x = 0;
	//grid->aPoints2D[149].pt.y = 0;
	int periodicy = srf->Periodicity();
	//if (periodicy == 1)
	//{
	//	if (grid->aPoints2D[it->first].pt.x == srf->GetUMin())
	//	{
	//		grid->aPoints2D[it->first].pt.y = grid->aPoints2D[grid->aPoints[it->first]->idDuble].pt.y;
	//		grid->aPoints2D[it->first].pt.x = srf->GetUMax();
	//	}
	//	else
	//	{
	//		if (grid->aPoints2D[it->first].pt.y == srf->GetUMin())
	//		{
	//			grid->aPoints2D[it->first].pt.x = grid->aPoints2D[grid->aPoints[it->first]->idDuble].pt.y;
	//			grid->aPoints2D[it->first].pt.y = srf->GetUMax();
	//		}
	//	}
	//}
	//u =x v =y
	std::map <int, bool> fixKnots;
	int countLoops = face->GetLoopsCount();
	int idPoles = 1;
	int sizePoints = grid->aPoints.size();
	int ind = 1;
	int sizePolePoints = 1;
	int sizePolePointsCheck = 1;
	for (std::map<int, Knot>::iterator it = grid->aPoints2D.begin(); it != grid->aPoints2D.end(); ++it)
	{
		if (sizePoints < ind)
		{
			continue;
		}
		MbCartPoint D(it->second.pt.x, it->second.pt.y);
		if (srf->IsPole(D))
		{
			fixKnots[it->first] = true;
			it->second.flagFix = true;
			it->second.pole = true;
			grid->aPoints[it->first].flagFix = true;
			grid->aPoints[it->first].pole = true;
			for (int i = 0; i < grid->aCells.size(); i++)
			{
				if (grid->aCells[i].m_aPointsId[0] == it->first)
				{
					sizePolePoints++;
				}

				if (grid->aCells[i].m_aPointsId[1] == it->first)
				{
					sizePolePoints++;
				}

				if (grid->aCells[i].m_aPointsId[2] == it->first)
				{
					sizePolePoints++;
				}

				if (grid->aCells[i].m_aPointsId[3] == it->first)
				{
					sizePolePoints++;
				}
			}

			for (int i = 0; i < grid->aCells.size(); i++)
			{
				if (grid->aCells[i].m_aPointsId[0] == it->first)
				{
					Knot pt;
					Knot pt2 = Knot(grid->aPoints[grid->aCells[i].m_aPointsId[0]].pt.x,
						grid->aPoints[grid->aCells[i].m_aPointsId[0]].pt.y,
						grid->aPoints[grid->aCells[i].m_aPointsId[0]].pt.z);
					if (srf->IsUPeriodic())
					{
						pt.pt.x = sizePolePointsCheck * srf->GetUMax() / sizePolePoints;//grid->aPoints2D[grid->aCells[i].m_aPointsId[3]].pt.x;
						pt.pt.y = grid->aPoints2D[grid->aCells[i].m_aPointsId[0]].pt.y;
						sizePolePointsCheck++;
						if (sizePolePointsCheck == 2)
						{
							pt.pt.x /= 2;
						}
						if (sizePolePointsCheck == sizePolePoints)
						{
							pt.pt.x += srf->GetUMax() / (2 * sizePolePoints);
						}
					}
					if (srf->IsVPeriodic())
					{
						pt.pt.x = grid->aPoints2D[grid->aCells[i].m_aPointsId[0]].pt.x;
						pt.pt.y = sizePolePointsCheck * srf->GetVMax() / sizePolePoints; //grid->aPoints2D[grid->aCells[i].m_aPointsId[3]].pt.y;
						sizePolePointsCheck++;
						if (sizePolePointsCheck == 2)
						{
							pt.pt.x /= 2;
						}
						if (sizePolePointsCheck == sizePolePoints)
						{
							pt.pt.x += srf->GetVMax() / (2 * sizePolePoints);
						}
					}
					pt.idDuble = grid->aCells[i].m_aPointsId[0];
					pt.duble = true;
					pt.pole = true;
					pt.flagFix = true;
					pt2.flagFix = true;
					int idNewPoint = idPoles + grid->aPoints2D.rbegin()->first;
					grid->aPoints2D[ idNewPoint] = pt;
					grid->aPoints[ idNewPoint] = pt2;
					grid->aPoints[ idNewPoint].idDuble = grid->aCells[i].m_aPointsId[0];
					grid->aPoints[ idNewPoint].duble = true;
					grid->aPoints[ idNewPoint].pole = true;
					grid->aCells[i].m_aPointsId[0] = idNewPoint;
					fixKnots[idNewPoint] = true;

				}
				if (grid->aCells[i].m_aPointsId[1] == it->first)
				{
					Knot pt;
					Knot pt2 = Knot(grid->aPoints[grid->aCells[i].m_aPointsId[1]].pt.x,
						grid->aPoints[grid->aCells[i].m_aPointsId[1]].pt.y,
						grid->aPoints[grid->aCells[i].m_aPointsId[1]].pt.z);
					if (srf->IsUPeriodic())
					{
						pt.pt.x = sizePolePointsCheck * srf->GetUMax() / sizePolePoints;// grid->aPoints2D[grid->aCells[i].m_aPointsId[0]].pt.x;
						pt.pt.y = grid->aPoints2D[grid->aCells[i].m_aPointsId[1]].pt.y;
						sizePolePointsCheck++;
						if (sizePolePointsCheck == 2)
						{
							pt.pt.x /= 2;
						}
						if (sizePolePointsCheck == sizePolePoints)
						{
							pt.pt.x += srf->GetUMax() / (2 * sizePolePoints);
						}
					}
					if (srf->IsVPeriodic())
					{
						pt.pt.x = grid->aPoints2D[grid->aCells[i].m_aPointsId[1]].pt.x;
						pt.pt.y = sizePolePointsCheck * srf->GetVMax() / sizePolePoints;//grid->aPoints2D[grid->aCells[i].m_aPointsId[0]].pt.y;
						sizePolePointsCheck++;
						if (sizePolePointsCheck == 2)
						{
							pt.pt.x /= 2;
						}
						if (sizePolePointsCheck == sizePolePoints)
						{
							pt.pt.x += srf->GetVMax() / (2 * sizePolePoints);
						}
					}
					pt.idDuble = grid->aCells[i].m_aPointsId[1];
					pt.duble = true;
					pt.pole = true;
					pt.flagFix = true;
					pt2.flagFix = true;
					int idNewPoint = idPoles + grid->aPoints2D.rbegin()->first;
					grid->aPoints2D[ idNewPoint] = pt;
					grid->aPoints[ idNewPoint] = pt2;
					grid->aPoints[ idNewPoint].idDuble = grid->aCells[i].m_aPointsId[1];
					grid->aPoints[ idNewPoint].duble = true;
					grid->aPoints[ idNewPoint].pole = true;
					grid->aCells[i].m_aPointsId[1] = idNewPoint;
					fixKnots[idNewPoint] = true;
				}
				if (grid->aCells[i].m_aPointsId[2] == it->first)
				{
					Knot pt;
					Knot pt2 = Knot(grid->aPoints[grid->aCells[i].m_aPointsId[2]].pt.x,
						grid->aPoints[grid->aCells[i].m_aPointsId[2]].pt.y,
						grid->aPoints[grid->aCells[i].m_aPointsId[2]].pt.z);
					if (srf->IsUPeriodic())
					{
						pt.pt.x = sizePolePointsCheck * srf->GetUMax() / sizePolePoints;//grid->aPoints2D[grid->aCells[i].m_aPointsId[1]].pt.x;
						pt.pt.y = grid->aPoints2D[grid->aCells[i].m_aPointsId[2]].pt.y;
						sizePolePointsCheck++;
						if (sizePolePointsCheck == 2)
						{
							pt.pt.x /= 2;
						}
						if (sizePolePointsCheck == sizePolePoints)
						{
							pt.pt.x += srf->GetUMax() / (2 * sizePolePoints);
						}
					}
					if (srf->IsVPeriodic())
					{
						pt.pt.x = grid->aPoints2D[grid->aCells[i].m_aPointsId[2]].pt.x;
						pt.pt.y = sizePolePointsCheck * srf->GetVMax() / sizePolePoints;//grid->aPoints2D[grid->aCells[i].m_aPointsId[1]].pt.y;
						sizePolePointsCheck++;
						if (sizePolePointsCheck == 2)
						{
							pt.pt.x /= 2;
						}
						if (sizePolePointsCheck == sizePolePoints)
						{
							pt.pt.x += srf->GetVMax() / (2 * sizePolePoints);
						}
					}
					pt.idDuble = grid->aCells[i].m_aPointsId[2];
					pt.duble = true;
					pt.pole = true;
					pt.flagFix = true;
					pt2.flagFix = true;
					int idNewPoint = idPoles + grid->aPoints2D.rbegin()->first;
					grid->aPoints2D[ idNewPoint] = pt;
					grid->aPoints[ idNewPoint] = pt2;
					grid->aPoints[ idNewPoint].idDuble = grid->aCells[i].m_aPointsId[2];
					grid->aPoints[ idNewPoint].duble = true;
					grid->aPoints[ idNewPoint].pole = true;
					grid->aCells[i].m_aPointsId[2] = idNewPoint;
					fixKnots[idNewPoint] = true;
				}
				if (grid->aCells[i].m_aPointsId[3] == it->first)
				{
					Knot pt;
					Knot  pt2 =  Knot(grid->aPoints[grid->aCells[i].m_aPointsId[3]].pt.x,
						grid->aPoints[grid->aCells[i].m_aPointsId[3]].pt.y,
						grid->aPoints[grid->aCells[i].m_aPointsId[3]].pt.z);
					if (srf->IsUPeriodic())
					{
						pt.pt.x = sizePolePointsCheck * srf->GetUMax() / sizePolePoints;//grid->aPoints2D[grid->aCells[i].m_aPointsId[2]].pt.x;
						pt.pt.y = grid->aPoints2D[grid->aCells[i].m_aPointsId[3]].pt.y;
						sizePolePointsCheck++;
						if (sizePolePointsCheck == 2)
						{
							pt.pt.x /= 2;
						}
						if (sizePolePointsCheck == sizePolePoints)
						{
							pt.pt.x += srf->GetUMax() / (2 * sizePolePoints);
						}
					}
					if (srf->IsVPeriodic())
					{
						pt.pt.x = grid->aPoints2D[grid->aCells[i].m_aPointsId[3]].pt.x;
						pt.pt.y = sizePolePointsCheck * srf->GetVMax() / sizePolePoints;//grid->aPoints2D[grid->aCells[i].m_aPointsId[2]].pt.y;
						sizePolePointsCheck++;
						if (sizePolePointsCheck == 2)
						{
							pt.pt.x /= 2;
						}
						if (sizePolePointsCheck == sizePolePoints)
						{
							pt.pt.x += srf->GetVMax() / (2 * sizePolePoints);
						}
					}
					pt.idDuble = grid->aCells[i].m_aPointsId[3];
					pt.duble = true;
					pt.pole = true;
					pt.flagFix = true;
					pt2.flagFix = true;
					int idNewPoint = idPoles + grid->aPoints2D.rbegin()->first;
					grid->aPoints2D[ idNewPoint] = pt;
					grid->aPoints[ idNewPoint] = pt2;
					grid->aPoints[ idNewPoint].idDuble = grid->aCells[i].m_aPointsId[3];
					grid->aPoints[ idNewPoint].duble = true;
					grid->aPoints[ idNewPoint].pole = true;
					grid->aCells[i].m_aPointsId[3] = idNewPoint;
					fixKnots[idNewPoint] = true;
				}

			}

		}
		ind++;
	}
	
	for (size_t i = 0; i < countLoops; i++)
	{
		MbLoop* loop = face->GetLoop(i);
		std::set<MbCurveEdge*> curveEdges;
		int countEdges = loop->GetEdgesCount();
		for (size_t j = 0; j < countEdges; j++)
		{
			MbOrientedEdge* edge = loop->GetOrientedEdge(j);
			
			const MbCurve3D& crv = edge->GetCurve();
			
			if (edge->IsSeam() && curveEdges.count(&edge->GetCurveEdge()) == 0)
			{
				curveEdges.insert({ &edge->GetCurveEdge()});
				//ищем полюсную точку и копируем ее 

				//пошли по шовному ребру
				int s = 1;
				std::map<int, Knot> knots;
				for (std::map<int, Knot>::iterator it = grid->aPoints.begin(); it != grid->aPoints.end(); ++it)
				{
					if (it->second.pole == true)
					{
						continue;
					}
					if (crv.IsPointOn(grid->aPoints[it->first].pt))
					{
						Knot knot =  Knot(it->second.pt.x, it->second.pt.y, it->second.pt.z);
						
						knots.insert({ s + grid->aPoints.rbegin()->first, knot });
						it->second.duble = true;
						double tmp = grid->aPoints.rbegin()->first;
						it->second.idDuble = s + grid->aPoints.rbegin()->first;
						knots[s + grid->aPoints.rbegin()->first].idDuble = it->first;
						knots[s + grid->aPoints.rbegin()->first].duble = true;

						s++;
					}
					
				}
				for (std::map<int, Knot>::iterator it = knots.begin(); it != knots.end(); ++it)
				{
					grid->aPoints[it->first] = it->second;
					//for (std::map<int, Knot>::iterator findDubl = grid->aPoints.begin(); findDubl != grid->aPoints.end(); ++findDubl)
					//{
					//	if (findDubl->second->idDuble == it->first)
					//	{
					//		grid->aPoints[it->first]->idDuble = findDubl->first;
					//	}
					//}
					MbCartPoint tch;
					srf->NearPointProjection(grid->aPoints[it->first].pt, tch.x, tch.y, false);
					Knot pt2D =  Knot(tch.x, tch.y);
					grid->aPoints2D.insert({ it->first, pt2D });
					if (periodicy == 1)
					{
						if (grid->aPoints2D[it->first].pt.x == srf->GetUMin())
						{
							grid->aPoints2D[it->first].pt.y = grid->aPoints2D[grid->aPoints[it->first].idDuble].pt.y;
							grid->aPoints2D[it->first].pt.x = srf->GetUMax();
						}
						else
						{
							if (grid->aPoints2D[it->first].pt.y == srf->GetUMin())
							{
								grid->aPoints2D[it->first].pt.x = grid->aPoints2D[grid->aPoints[it->first].idDuble].pt.y;
								grid->aPoints2D[it->first].pt.y = srf->GetUMax();
							}
						}
					}
					if (periodicy == 2)
					{
						if (grid->aPoints2D[it->first].pt.x == srf->GetVMin())
						{
							grid->aPoints2D[it->first].pt.y = grid->aPoints2D[grid->aPoints[it->first].idDuble].pt.y;
							grid->aPoints2D[it->first].pt.x = srf->GetVMax();
						}
						else
						{
							if (grid->aPoints2D[it->first].pt.y == srf->GetVMin())
							{
								grid->aPoints2D[it->first].pt.x = grid->aPoints2D[grid->aPoints[it->first].idDuble].pt.y;
								grid->aPoints2D[it->first].pt.y = srf->GetVMax() ;
							}
						}
					}
				
				}
				
				for (size_t w = 0; w < grid->aCells.size(); w++)
				{
					std::vector<int> pointsId = grid->aCells[w].m_aPointsId;
					for (int q = 0; q < 4; q++)
					{
						if (grid->aPoints[pointsId[q]].duble == true && grid->aPoints[pointsId[q]].pole == false)
						{
		
								int next = Inc(q, 1, 4);
								int preview = Inc(q, -1, 4);
								double x1 = grid->aPoints2D[pointsId[preview]].pt.x;
								double y1 = grid->aPoints2D[pointsId[preview]].pt.y;
								double x2 = grid->aPoints2D[pointsId[q]].pt.x;
								double y2 = grid->aPoints2D[pointsId[q]].pt.y;
								double x3 = grid->aPoints2D[pointsId[next]].pt.x;
								double y3 = grid->aPoints2D[pointsId[next]].pt.y;
								double S = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
								if (S < 0)
								{
									grid->aCells[w].m_aPointsId[q] = grid->aPoints[pointsId[q]].idDuble;
								}
						}
					
					}
				}

		
			}
	
	
			for (std::map<int, Knot>::iterator it = grid->aPoints.begin(); it != grid->aPoints.end(); ++it)
			{

				if (fixKnots[it->first] == true)
				{
					continue;
				}
			
				if (crv.IsPointOn(grid->aPoints[it->first].pt))
				{
					//fixKnots.insert({ it->first,true });
					fixKnots[it->first] = true;
				}
				else
				{
					//fixKnots.insert({ it->first,false });
					fixKnots[it->first] = false;
				}
			}
		}
	}


	return  fixKnots;
}

/**
* \brief Вернуть NURBS поверхность в модели и зафиксировать граничные точки
* \param [in] ModelDir Дириктория модели
* \param [in, out] grid Сетка
* \return NURBS поверхность
*/
MbSplineSurface* GetSrfNurbs(const char* modelDir,Grid* grid)
{
	initC3D();
	MbModel* model = new MbModel();
	ReadModel(modelDir, model);

	const MbSolid* solid = nullptr;
	size_t itemCount = model->ItemsCount();
	MbeSpaceType st = model->GetItem(0)->IsA();
	for (size_t i = 0; i < itemCount; i++)
	{
		if (model->GetItem(i)->IsA() == st_Solid)
		{
			solid = dynamic_cast<const MbSolid*>(model->GetItem(i));
		}
		if (model->GetItem(i)->IsA() == st_Assembly)
		{
			const MbAssembly* assembly;
			assembly = dynamic_cast<const MbAssembly*> (model->GetItem(0));
			//MbItem* item = assembly->SetItem(0);
			//st = item->IsA();
			const MbInstance* inst = dynamic_cast<const MbInstance*>(assembly->GetItem(0));
			solid = dynamic_cast<const MbSolid*>(inst->GetItem());
			int a = 0;

		}
	}
	const MbFace* face = solid->GetFace(1);
	//face = dynamic_cast<const MbFace*>(model->GetItem(0));
	//if (solid->GetFacesCount() == 1)
	//{
	//	face = solid->GetFace(0);
	//}
	//else 
	//{
	//	//throw - 1;
	//	face = solid->GetFace(1);
	//}
	MbSurface *srf = const_cast<MbSurface*>(&face->GetSurface());
	const MbSurface *srf1 = &srf->GetBasisSurface();
	//int a = srf1->Periodicity();
	//int b = srf->Periodicity();
	//MbeSpaceType s = srf1->Family();
	//MbeSpaceType s1 = srf1->Type();
	MbSplineSurface* srfNurbs = face->GetSurface().NurbsSurface(true);
	ProectionOn2D(grid, srfNurbs);
	if (grid->fixPoint.size() == 0)
	{
		std::map<int, bool> fixKnots = GetFixKnots(face, grid, srf1);
		for (std::map<int, Knot>::iterator it = grid->aPoints.begin(); it != grid->aPoints.end(); ++it)
		{
			if (grid->aPoints[it->first].flagFix)continue;
			grid->aPoints[it->first].flagFix = fixKnots[it->first];
		}
	}
	return srfNurbs;
}
