#include "Knot.h"

Knot::Knot()
{
	pt = MbCartPoint3D(0, 0, 0);
	flagFix = false;
	Init();
};

Knot::Knot(double _x, double _y)
{
	pt = MbCartPoint3D(_x, _y, 0);
	flagFix = false;
	Init();
}

Knot::Knot(double _x, double _y, double _z)
{
	pt = MbCartPoint3D(_x, _y, _z);
	flagFix = false;
	Init();
}


Knot::Knot(MbCartPoint3D point)
{
	pt = point;
	flagFix = false;
	Init();
}
