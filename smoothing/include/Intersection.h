#pragma once
#include "Knot.h"
#include "Grid.h"
#include "Edge.h"
#include "Cell.h"
struct pt {
	double x, y;
};
struct seg {
	pt p, q;
	int id;

	double get_y(double x) const {
		if (abs(p.x - q.x) < EPS)  return p.y;
		return p.y + (q.y - p.y) * (x - p.x) / (q.x - p.x);
	}
};
struct event {
	double x;
	int tp, id;

	event() {}
	event(double x, int tp, int id)
		: x(x), tp(tp), id(id) {}

	bool operator< (const event & e) const {
		if (abs(x - e.x) > EPS)  return x < e.x;
		return tp > e.tp;
	}
};

template<typename T>
struct myVector {
	myVector(size_t sz) : size(sz) {
		chunk = new T[size];
	}
	T* chunk{ nullptr };
	size_t size{ 0 };
};

std::set<seg> s;

std::vector <std::set<seg>::iterator > where;
std::set<seg>::iterator prev(std::set<seg>::iterator it);
std::set<seg>::iterator next(std::set<seg>::iterator it);
bool intersect1d(double l1, double r1, double l2, double r2);
int vec(const pt & a, const pt & b, const pt & c);
bool intersect(const seg & a, const seg & b);
bool operator< (const seg & a, const seg & b);
bool operator== (const pt & a, const pt & b);
std::vector<std::pair<int, int>> solve(const std::vector<seg> & a);
void SweepingLine(Grid* grid);
void edgeIntersection2(Grid* grid, Cell& cell, int indEdge);