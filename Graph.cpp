#include "Graph.h"

Vector GeomGraph::outwardDirFrom(const GeomEdge& e, const GeomNode& n) {
    if (n.node == *e.edge.start) return e.dir;
    return e.dir.flipped();
}

GeomEdge::GeomEdge(GeomGraph& g, Edge& e, GeomNode& start, GeomNode& end)
    : edge(e), dir(Vector(start.pos, end.pos).normalized()),
    start_edges_item(g.insertEdge(start, this)),
    end_edges_item(g.insertEdge(end, this)) {}


PolyEdge::PolyEdge(Edge& e, const GeomEdge& ge) :
    edge(e), geom_edge(ge) {}

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

DualGraph importDualGraph(const PolyGraphExport& pgx) {
    DualGraph dg;
    GeomGraph& gg = dg.geom_graph;
    PolyGraph& pg = dg.poly_graph;
    for (auto& p : pgx.geom_nodes) {
        gg.addGeomNode(Point(p.first, p.second));
    }
    for (auto& e : pgx.geom_edges) {
        auto ge = gg.addGeomEdge(gg.getGeomNode(e.first),
            gg.getGeomNode(e.second));
        auto ue = pg.g.addUnboundEdge();
        pg.poly_edges.emplace_back(
            std::make_unique<PolyEdge>(ue.second, ge.second));
    }
    for (auto& p : pgx.polygons) {
        auto n = pg.g.addNode();
        auto& node = n.second;
        std::string name = p.name;
        auto& poly = pg.polygons.emplace_back(
            std::make_unique<Polygon>(node, std::move(name)));
        poly->revolution = p.is_interior_poly 
            ? innerRevolutionOfNGon(p.edges.size())
            : outerRevolutionOfNGon(p.edges.size());
        for (auto signed_edge_idx : p.edges) {
            size_t edge_idx = abs(signed_edge_idx) - 1;
            Edge& e = pg.g.getEdge(edge_idx);
            node.edges.emplace_back(&e);
            if (signed_edge_idx > 0) {
                e.start = &node;
            }
            else {
                e.end = &node;
            }
        }
    }
    return dg;
}

PolyGraphExport exportPolyGraph(const PolyGraph& poly_graph) {
    PolyGraphExport pgx;
    for (size_t i = 0; i < poly_graph.geom_graph.numNodes(); ++i) {
        auto& geom_node = poly_graph.geom_graph.getGeomNode(i);
        pgx.geom_nodes.emplace_back(
            geom_node.pos.x, geom_node.pos.y);
    }
    for (size_t i = 0; i < poly_graph.g.edges.size(); ++i) {
        auto& geom_edge = poly_graph.geom_graph.getGeomEdge(i);
        pgx.geom_edges.emplace_back(
            geom_edge.edge.start->key, geom_edge.edge.end->key);
    }
    for (auto& p : poly_graph.polygons) {
        std::vector<int> edges;
        for (auto* e : p->node.edges) {
            int idx = e->key + 1;
            if (e->start == &p->node) {
                edges.push_back(idx);
            }
            else {
                edges.push_back(-idx);
            }
        }
        pgx.polygons.emplace_back(
            p->name, 
            p->revolution < innerRevolutionOfNGon(p->node.edges.size())
                + Rotation{0.0, -1.0, 0} /* accounting for numeric errors */,
            std::move(edges));
    }
    return pgx;
}

std::ostream& operator<<(std::ostream& os, const PolyGraphExport& pge) {
    os << "{" << std::endl;
    os << "  \"vertices\": [" << std::endl << "    ";
    size_t i = 0;
    for (; i < pge.geom_nodes.size(); ++i) {
        auto& gn = pge.geom_nodes[i];
        if (i != 0) os << ", ";
        os << "[" << gn.first << ", " << gn.second << "]";
        if (i % 4 == 3) os << std::endl << "    ";
    }
    if (i % 4 != 3) os << std::endl;
    os << "  ]," << std::endl;

    os << "  \"edges\": [" << std::endl << "    ";
    i = 0;
    for (; i < pge.geom_edges.size(); ++i) {
        auto& ge = pge.geom_edges[i];
        if (i != 0) os << ", ";
        os << "[" << ge.first << ", " << ge.second << "]";
        if (i % 4 == 3) os << std::endl << "    ";
    }
    if (i % 4 != 3) os << std::endl;
    os << "  ]," << std::endl;

    os << "  \"faces\": [";
    for (size_t i = 0; i < pge.polygons.size(); ++i) {
        if (i != 0) os << ",";
        os << std::endl << "    ";
        auto& p = pge.polygons[i];
        os << "{\"name\"=\"" << p.name << "\", " << std::endl;
        os << "     \"interior\"=" << p.is_interior_poly << ", " <<std::endl;
        os << "     \"edges\"=[";
        size_t j = 0;
        for (; j < p.edges.size(); ++j) {
            int signed_edge_idx = p.edges[j];
            if (j != 0) os << ", ";
            os << signed_edge_idx;
            if (j % 4 == 3) os << std::endl << "      ";
        }
        os << "]}";
    }
    os << "  ]," << std::endl;

    os << "}" << std::endl;
    return os;
}

