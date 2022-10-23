#include <iostream>
#include "Graph.h"

struct HighArcPolygonTest {
	GeomGraph gg;
	GeomNode& n1{ gg.addGeomNode(Point(-38.2, 0)).second };
	GeomNode& n2{ gg.addGeomNode(Point(29.87, 45.12)).second };
	GeomNode& n3{ gg.addGeomNode(Point(60, 15.8)).second };
	GeomNode& n4{ gg.addGeomNode(Point(11.3, -23.09)).second };
	GeomNode& n5{ gg.addGeomNode(Point(0, 5)).second };
	GeomNode& n6{ gg.addGeomNode(Point(-12, -49.784)).second };
	GeomNode& n7{ gg.addGeomNode(Point(27, -35.55)).second };
};

PolyGraphExport kite(HighArcPolygonTest& hapt) {
	hapt.gg.addGeomEdge(hapt.n1, hapt.n2);
	hapt.gg.addGeomEdge(hapt.n2, hapt.n3);
	hapt.gg.addGeomEdge(hapt.n3, hapt.n4);
	hapt.gg.addGeomEdge(hapt.n4, hapt.n1);
	hapt.gg.addGeomEdge(hapt.n5, hapt.n1);
	hapt.gg.addGeomEdge(hapt.n5, hapt.n2);
	hapt.gg.addGeomEdge(hapt.n5, hapt.n3);
	hapt.gg.addGeomEdge(hapt.n5, hapt.n4);
	PolygonBuilder pb(hapt.gg);
	PolyGraph pg{ std::move(pb.buildPolygons()) };
	return exportPolyGraph(pg);
}


int main()
{
	HighArcPolygonTest hapt;
	auto kite_export = kite(hapt);
	std::cout << kite_export;
}
