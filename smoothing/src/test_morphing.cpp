#include <surf_spline_surface.h>
#include "QDec.h"
#include "Grid.h"
#include "Algorithm.h"
#include "ReadModel.h"
#include "RWFile.h"
#include "QDecFunc.h"
#include <geom_view.h>
#include <cur_hermit3d.h>
#include <action_curve3d.h>
#include <chrono>
const std::string inputFile = "../../../qdecTestMorphing.txt";
const std::string outputFile = "../../../qdecTestMorphing_res.txt";
geom_view gv;

struct geo
{
	Grid* grid;
	std::vector<int> controlPt;
	std::vector<int> otherPt;
	
	void init(Grid* _grid)
	{
		grid = _grid;
		for (auto i : grid->edges)
		{
			int midle = i.idPointsIn.size() / 2;
			grid->aPoints[i.idPointsIn[midle]].flagEdgePt1Pt2 = true;
		}
		for (auto t : grid->aPoints)
		{
			if (t.second.flagFix)
			{
				controlPt.push_back(t.first);
			}
			else
			{
				if (t.second.flagEdgePt1Pt2)
				{
					controlPt.push_back(t.first);
				}
				else
				{
					otherPt.push_back(t.first);
				}
			}
		}
	}
};
void WriteTestFile(geo* g)
{
	std::ofstream of(outputFile);
	of << "control_points: p\n";
	for (auto i : g->controlPt)
	{
		
		of << "(" << g->grid->aPoints[i].pt.x << ", " << g->grid->aPoints[i].pt.y << ", " << g->grid->aPoints[i].pt.z << ")";
		if(!g->grid->aPoints[i].peresech) of<<" 6 (0, 1, 0)" << std::endl;
		else of << " 6 (1, 0, 0)" << std::endl;
	}
	of << "points: u\n";
	for (auto i : g->otherPt)
	{
		of << "(" << g->grid->aPoints[i].pt.x << ", " << g->grid->aPoints[i].pt.y << ", " << g->grid->aPoints[i].pt.z << ")";
		if (!g->grid->aPoints[i].peresech) of << " 6 (0, 0, 1)" << std::endl;
		else of << " 6 (1, 0, 0)" << std::endl;
	}
	//of << "control_points: Notfix\n";
	//for (std::map<int, Knot>::iterator it = grid->aPoints.begin(); it != grid->aPoints.end(); ++it)
	//{
	//	if (it->second->flagEdgePt1Pt2 && !it->second.flagFix)
	//		of << "(" << it->second.pt.x << ", " << it->second.pt.y << ", " << it->second.pt.z << ")" << " 6 (1, 0, 0)" << std::endl;
	//}
	//of << "control_points: NotfixEdge\n";
	//for (std::map<int, Knot>::iterator it = grid->aPoints.begin(); it != grid->aPoints.end(); ++it)
	//{
	//	if (!it->second->flagEdgePt1Pt2 && !it->second.flagFix)
	//		of << "(" << it->second.pt.x << ", " << it->second.pt.y << ", " << it->second.pt.z << ")" << " 4 (0, 0, 1)" << std::endl;
	//}
	//
	size_t aCellsSize = g->grid->aCells.size();
	of << "lines : All" << std::endl;

	for (auto i : g->grid->edges)
	{

		for (int j = 0; j < i.idPointsIn.size() - 1 ; j++)
		{
			of << "(" << g->grid->aPoints[i.idPointsIn[j]].pt.x << ", "
				<< g->grid->aPoints[i.idPointsIn[j]].pt.y << ", "
				<< g->grid->aPoints[i.idPointsIn[j]].pt.z << ") ";
			of << "(" << g->grid->aPoints[i.idPointsIn[j+1]].pt.x << ", "
				<< g->grid->aPoints[i.idPointsIn[j+1]].pt.y << ", "
				<< g->grid->aPoints[i.idPointsIn[j+1]].pt.z << ") ";
			if (!i.intersection)
			of << "(0, 0, 1)" << std::endl;
			else
			of << "(0, 1, 0)" << std::endl;
		}

	}
}

void reSplitE(Grid* g)
{
	if (g->edges[0].idPointsIn.size() > g->edges[1].idPointsIn.size())
	{
		ReSplitEdge(g, g->edges[1], g->edges[0].idPointsIn.size() - 1);
		ReSplitEdge(g, g->edges[3], g->edges[0].idPointsIn.size() - 1);
	}
	else
	{
		ReSplitEdge(g, g->edges[0], g->edges[1].idPointsIn.size() - 1);
		ReSplitEdge(g, g->edges[2], g->edges[1].idPointsIn.size() - 1);
	}
}
void edge1Intersection(Grid* grid, Cell& cell)
{
	for (auto indEdge : cell.edges)
	{
		Edge edge = grid->edges[indEdge];
		for (int i = 0; i<edge.idPointsIn.size() - 1; i++)
		{
			MbCartPoint r11 = { grid->aPoints[edge.idPointsIn[i]].pt.x
				,grid->aPoints[edge.idPointsIn[i]].pt.y };
			MbCartPoint r12 = { grid->aPoints[edge.idPointsIn[i + 1]].pt.x,
				grid->aPoints[edge.idPointsIn[i + 1]].pt.y };
			for (auto ind2Edge : cell.edges)
			{
				//int ind2Edge = Inc(indEdge, 2, 4);
				if (indEdge == ind2Edge)
					continue;

				Edge edge1 = grid->edges[ind2Edge];
				for (int j = 0; j < edge1.idPointsIn.size() - 1; j++)
				{
					MbCartPoint r21 = { grid->aPoints[edge1.idPointsIn[j]].pt.x,
						grid->aPoints[edge1.idPointsIn[j]].pt.y };
					MbCartPoint r22 = { grid->aPoints[edge1.idPointsIn[j + 1]].pt.x ,
						grid->aPoints[edge1.idPointsIn[j + 1]].pt.y };
					double detA = (r11.x - r12.x)*(r22.y - r21.y) - (r11.y - r12.y)*(r22.x - r21.x);
					double detX = ((r22.y - r21.y)*(r22.x - r12.x) - (r22.x - r21.x)*(r22.y - r12.y));
					double detY = (r11.x - r12.x)* (r22.y - r12.y) - (r22.x - r12.x)*(r11.y - r12.y);

					double betta = detY / detA;
					double alpha = detX / detA;

					if (alpha > 0 && betta > 0 && alpha < 1 && betta < 1)
					{
						if (r11 == r21 || r11 == r22 || r12 == r22 || r12 == r21)continue;
						grid->edges[indEdge].intersection = true;
						grid->aPoints[grid->edges[indEdge].idPointsIn[i]].peresech = true;
						grid->aPoints[grid->edges[indEdge].idPointsIn[i + 1]].peresech = true;
						grid->aPoints[grid->edges[ind2Edge].idPointsIn[j]].peresech = true;
						grid->aPoints[grid->edges[ind2Edge].idPointsIn[j + 1]].peresech = true;
					}



				}
			}
		}
	}
}

void moveControl(void* callback_data, std::vector<std::string>& sId, double x, double y, double z)
{



	auto data = static_cast<std::array<void*, 2>*>(callback_data);
	geo* g = static_cast<geo*>((*data)[1]);
	geom_view* gv = static_cast<geom_view*>((*data)[0]);

	int id = atoi(sId.back().c_str());
	if (sId.front() == "p")
	{
		id = g->controlPt[id];
	}
	else
	{
		id = g->otherPt[id];
	}
	MbCartPoint3D pt(x, y, z);
	g->grid->aPoints[id].pt = pt;


	std::vector<MbHermit3D*> hermits;
	for (auto i : g->grid->edges)
	{
		SArray<MbCartPoint3D> pts;
		for (auto j : i.idPointsIn)
		{
			if (g->grid->aPoints[j].flagEdgePt1Pt2)
			{
				pts.push_back(g->grid->aPoints[j].pt);
			}
		}
		MbHermit3D* hermit = MbHermit3D::Create(pts, false);
		hermits.push_back(hermit);
		int count = 0;
		for (auto j : i.idPointsIn)
		{
			double min = hermit->GetTMin();
			double max = hermit->GetTMax();
			max = max - min;
			min = 0;
			double k = (count* max) / (i.idPointsIn.size() - 1);
			count++;
			hermit->PointOn(k, g->grid->aPoints[j].pt);
		}


	}

	
	//std::vector<Edge> e;
	//for (int i = 0; i < 4; i++)
	//{
	//	e.push_back(g->grid->edges[i]);
	//}
	//
	//
	//std::vector<std::vector<int>> res = addPointMatrx(g->grid, e, false);// = createMatrixPoints(g->grid, e,false);
	//for (int i = 0; i < res.size(); i++)
	//{
	//	for (int j = 0; j < res[i].size(); j++)
	//	{
	//		g->otherPt.push_back(res[i][j]);
	//	}
	//}
	for (auto cell : g->grid->aCells)
	{
		edge1Intersection(g->grid,cell);
	}
	std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
	
	WriteTestFile(g);

	std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1) * 100;
	std::cout << "It took me " << time_span.count() << " seconds.\n";
	gv->reload();

	
}



void main() 
{
	
	Grid* grid;
	
	QDec* qdec = ReadQDec();
	grid = ConvetQDecFromGrid(qdec);




	//WriteFileQDec(g->grid, false);
	//reSplitE(grid);
	
	//диагонали
	//std::vector<Edge> e;
	//for (int i = 0; i < 4; i++)
	//{
	//	e.push_back(grid->edges[i]);
	//}
	//std::vector<std::vector<int>> res = addPointMatrx(grid,e,true);// = createMatrixPoints(g->grid, e, true);
	//
	//
	//Edge diag1, diag2;
	////проблема в первом элементе res всегда
	////чтобы запомнить границы надо бовать первые и последние элементы E
	//diag1.idPointsIn.push_back(e[0].pt1());
	//diag2.idPointsIn.push_back(e[2].pt2());
	//
	//for (int j = 1; j < res.size(); j++)
	//{
	//
	//	grid->aPoints[res[j][0]].diag = true;
	//	//int tmp = InsertInMapInEnd(grid, g.aPoints[res[j][0]]);
	//	//WriteFileQDec(grid, false);
	//	if (j % 2 == 0)
	//	{
	//		diag1.idPointsIn.push_back(res[j][0]);
	//	}
	//	else
	//	{
	//		diag2.idPointsIn.push_back(res[j][0]);
	//	}
	//}
	//diag1.idPointsIn.push_back(e[1].pt2());
	//diag2.idPointsIn.push_back(e[1].pt1());
	//
	//diag1.diag = true;
	//diag2.diag = true;
	//if (diag1.idPointsIn.size() != 0) {
	//	grid->edges.push_back(diag1);
	//}
	//if (diag2.idPointsIn.size() != 0) {
	//	grid->edges.push_back(diag2);
	//}
	//
	
	geo* g = new geo();
	g->init(grid);
	//for (int i = 0; i < res.size(); i++)
	//{
	//	for (int j = 0; j < res[i].size(); j++)
	//	{
	//		g->otherPt.push_back(res[i][j]);
	//	}
	//}
	//
	//
	//
	////MorphDiag(g->grid, g->grid->aCells[0]);
	////int size = g->controlPt.size() + g->otherPt.size() - 1;
	///*
	//if (size < g->grid->aPoints.size() - 1)
	//{
	//	int count = 0;
	//	for (std::map<int, Knot>::iterator it = g->grid->aPoints.begin(); it != g->grid->aPoints.end(); ++it)
	//	{
	//		if (count < size)
	//		{
	//			count++;
	//			continue;
	//		}
	//		
	//		g->otherPt.push_back(it->first);
	//	}
	//}
	//*/
	//
	///*
	//std::vector<MbHermit3D*> hermits;
	//for (auto i : g->grid->edges)
	//{
	//	SArray<MbCartPoint3D> pts;
	//	for (auto j : i.idPointsIn)
	//	{
	//		if (g->grid->aPoints[j]->flagEdgePt1Pt2)
	//		{
	//			pts.push_back(g->grid->aPoints[j].pt);
	//		}
	//	}
	//	MbHermit3D* hermit = MbHermit3D::Create(pts, false);
	//	hermits.push_back(hermit);
	//	int count = 0;
	//	for (auto j : i.idPointsIn)
	//	{
	//		double min = hermit->GetTMin();
	//		double max = hermit->GetTMax();
	//		max = max - min;
	//		min = 0;
	//		double k = (count* max) / (i.idPointsIn.size()-1);
	//		count++;
	//		hermit->PointOn(k, g->grid->aPoints[j].pt);
	//	}
	//
	//	
	//}
	//*/
	for (auto cell : g->grid->aCells)
	{
		edgeIntersection(g->grid, cell);
	}
	
	WriteTestFile(g);
	gv.init(outputFile);
	
	std::array <void*, 2> callback_data{ (void*)&gv,(void*) g };

	gv.setCallBack((void*)& callback_data, &moveControl);
	std::string cmd;
	while (cmd != "exit") {
		std::cin >> cmd;
	}
}

