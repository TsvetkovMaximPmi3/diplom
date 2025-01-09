#include "Intersection.h"


std::set<seg>::iterator prev(std::set<seg>::iterator it) {
	return it == s.begin() ? s.end() : --it;
}

std::set<seg>::iterator next(std::set<seg>::iterator it) {
	return it == s.end() ? s.begin() : ++it;
}


bool intersect1d(double l1, double r1, double l2, double r2) {
	if (l1 > r1)  std::swap(l1, r1);
	if (l2 > r2)  std::swap(l2, r2);
	return std::max(l1, l2) <= std::min(r1, r2) + EPS;
}
int vec(const pt & a, const pt & b, const pt & c) {
	double s = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
	return abs(s) < EPS ? 0 : s > 0 ? +1 : -1;
}
bool intersect(const seg & a, const seg & b) {
	return intersect1d(a.p.x, a.q.x, b.p.x, b.q.x)
		&& intersect1d(a.p.y, a.q.y, b.p.y, b.q.y)
		&& vec(a.p, a.q, b.p) * vec(a.p, a.q, b.q) <= 0
		&& vec(b.p, b.q, a.p) * vec(b.p, b.q, a.q) <= 0;
}


bool operator< (const seg & a, const seg & b) {
	double x = std::max(std::min(a.p.x, a.q.x), std::min(b.p.x, b.q.x));
	return a.get_y(x) < b.get_y(x) - EPS;
}
bool operator== (const pt & a, const pt & b) {
	return a.x == b.x && a.y == b.y;
}

std::vector<std::pair<int, int>> solve(const std::vector<seg> & a) {
	std::vector<std::pair<int, int>> res;
	for (auto tmp : a) {
		int i = 0;
		for (auto tmp1 : a) {
			if (tmp.id == tmp1.id) {
				i++;
			}
		}
	}
	int n = (int)a.size();
	std::vector<event> e;
	for (int i = 0; i < n; ++i) {
		e.push_back(event(std::min(a[i].p.x, a[i].q.x), +1, i));
		e.push_back(event(std::max(a[i].p.x, a[i].q.x), -1, i));
	}
	sort(e.begin(), e.end());

	s.clear();
	where.resize(a.size());
	for (size_t i = 0; i < e.size(); ++i) {
		int id = e[i].id;
		if (e[i].tp == +1) {
			std::set<seg>::iterator
				nxt = s.lower_bound(a[id]),
				prv = prev(nxt);
			/*if (nxt->p == prv->p || nxt->p == prv->q || nxt->q == prv->p || nxt->q == prv->q)
			continue;*/
			if (nxt != s.end() && intersect(*nxt, a[id]))
				res.push_back(std::make_pair(nxt->id, id));
			if (prv != s.end() && intersect(*prv, a[id]))
				res.push_back(std::make_pair(prv->id, id));
			where[id] = s.insert(nxt, a[id]);
		} else {
			std::set<seg>::iterator
				tmp = where[id],
				nxt = next(where[id]),
				prv = prev(where[id]);
			if (nxt != s.end() && prv != s.end() && intersect(*nxt, *prv)) {

				res.push_back(std::make_pair(prv->id, nxt->id));
			}
			s.erase(where[id]);
		}
	}

	return res;
}

void SweepingLine(Grid* grid) {
	std::set<seg> edges;
	std::vector<seg> res;
	for (int i = 0; i < grid->edges.size(); i++) {
		for (int j = 0; j < grid->edges[i].idPointsIn.size() - 1; j++) {
			pt p, q;
			p.x = grid->aPoints2D[grid->edges[i].idPointsIn[j]].pt.x;
			p.y = grid->aPoints2D[grid->edges[i].idPointsIn[j]].pt.y;
			q.x = grid->aPoints2D[grid->edges[i].idPointsIn[j + 1]].pt.x;
			q.y = grid->aPoints2D[grid->edges[i].idPointsIn[j + 1]].pt.y;
			int id = i * 100 + j;
			seg segment;
			segment.p = p;
			segment.q = q;
			segment.id = id;
			edges.insert(segment);
		}
	}
	for (auto edge : edges) {
		res.push_back(edge);
	}
	std::vector<std::pair<int, int>> per = solve(res);
}


void edgeIntersection2(Grid* grid, Cell& cell, int indEdge) {
	Edge edge = grid->edges[indEdge];
	for (int i = 0; i < edge.idPointsIn.size(); i++) {
		for (auto ind2Edge : cell.edges) {
			//int ind2Edge = Inc(indEdge, 2, 4);

			if (indEdge == ind2Edge || grid->edges[indEdge].pt1() == grid->edges[indEdge].pt2() || grid->edges[ind2Edge].pt1() == grid->edges[ind2Edge].pt2())
				continue;
			auto find = std::find(grid->edges[ind2Edge].idPointsIn.begin(), grid->edges[ind2Edge].idPointsIn.end(), edge.idPointsIn[i]);
			if (find != grid->edges[ind2Edge].idPointsIn.end())
				continue;

			Edge edge1 = grid->edges[ind2Edge];
			for (int j = 0; j < edge1.idPointsIn.size() - 1; j++) {
				if (edge.idPointsIn[i] == edge.idPointsIn[j]) continue;
				MbCartPoint3D p1 = grid->aPoints[edge.idPointsIn[i]].pt,
					p2 = MbCartPoint3D(grid->aPoints[edge.idPointsIn[i]].pt.x + grid->aPoints[edge.idPointsIn[i]].moveVect.x * grid->speed*0.5,
									   grid->aPoints[edge.idPointsIn[i]].pt.y + grid->aPoints[edge.idPointsIn[i]].moveVect.y * grid->speed*0.5,
									   grid->aPoints[edge.idPointsIn[i]].pt.z + grid->aPoints[edge.idPointsIn[i]].moveVect.z * grid->speed*0.5),
					p3 = grid->aPoints[edge1.idPointsIn[j]].pt,
					p4 = grid->aPoints[edge1.idPointsIn[j + 1]].pt;
				MbCartPoint3D p13(p1.x - p3.x, p1.y - p3.y, p1.z - p3.z),
					p43(p4.x - p3.x, p4.y - p3.y, p4.z - p3.z), p21;

				double d1343, d4321, d1321, d4343, d2121;
				double denom;

				double eps = Math::lengthEpsilon;

				if (std::abs(p43.x) < eps && std::abs(p43.y) < eps && std::abs(p43.z) < eps) {
					grid->aPoints[edge1.idPointsIn[i]].peresech = false;
					continue;
				}

				p21 = MbCartPoint3D(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
				if (std::abs(p21.x) < eps && std::abs(p21.y) < eps && std::abs(p21.z) < eps) {
					grid->aPoints[edge1.idPointsIn[i]].peresech = false;
					continue;
				}
				d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
				d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
				d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
				d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
				d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

				denom = d2121 * d4343 - d4321*d4321;
				if (std::abs(denom) < eps) {
					grid->aPoints[edge1.idPointsIn[i]].peresech = false;
					continue;
				}

				double numer = d1343 * d4321 - d1321 * d4343;
				double mua = numer / denom;
				double mub = (d1343 + d4321 * mua) / d4343;

				if (mua - eps <= 0 || mub - eps < 0 || mua + eps >= 1 || mub + eps >= 1) {
					grid->aPoints[edge1.idPointsIn[i]].peresech = false;
					continue;
				}
				grid->aPoints[edge1.idPointsIn[i]].peresech = true;

			}
		}
	}

}

