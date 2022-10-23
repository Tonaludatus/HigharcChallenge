#include "Graph.h"

Vector GeomGraph::outwardDirFrom(const GeomEdge& e, const GeomNode& n) {
    if (n.node == *e.edge.start) return e.dir;
    return e.dir.flipped();
}

GeomEdge::GeomEdge(GeomGraph& g, Edge& e, GeomNode& start, GeomNode& end)
    : edge(e), dir(Vector(start.pos, end.pos).normalized()),
    start_edges_item(g.insertEdge(start, this)),
    end_edges_item(g.insertEdge(end, this)) {}


PolyEdge::PolyEdge(Edge& e, Polygon& l, Polygon& r, const GeomEdge* ge) :
    edge(e), left(l), right(r), geom_edge(ge) {}

bool GeomGraph::compare(const GeomNode& n, const GeomEdge& one, const GeomEdge& other) const {
    return ccwLessNormalized(outwardDirFrom(one, n), outwardDirFrom(other, n));
}

Node::EdgesListType::iterator GeomGraph::insertEdge(GeomNode& n, GeomEdge* e) {
    auto& edges_list = n.node.edges;
    auto it = edges_list.begin();
    while (it != edges_list.end() && compare(n, getGeomEdge((*it)->key), *e)) {
        ++it;
    }
    return edges_list.emplace(it, &(e->edge));
}

