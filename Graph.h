#pragma once
#include <list>
#include <memory>
#include <queue>
#include <string>

#include "CircularList.h"
#include "Constants.h"
#include "Point.h"
#include "Vector.h"

struct GeomNode;
struct Polygon;
struct PolyEdge;

template <typename Data, typename EdgeT>
struct Node {
    Data data;
    CList<EdgeT*> edges; // ordered
    Node(const Data& d) : data(d) {};
    Node(const Node&) = delete;
    Node(Node&&) = default;
    bool operator==(const Node& other) const {
        return &other == this;
    }
};


template <typename NodeT, typename Assoc>
struct Edge {
    NodeT& start;
    NodeT& end;
    Assoc assoc;

    Edge(NodeT& s, NodeT& e, Assoc&& a) : start(s), end(e), assoc(std::move(a)) {}
    Edge(const Edge&) = delete;
    Edge(Edge&&) = default;
};

struct GeomEdge : public Edge<GeomNode, Vector> {
    using NodeT = GeomNode;
    Vector& dir; // normalized
    Vector outwardDirFrom(const GeomNode& n) const;

    GeomEdge(GeomNode& s, GeomNode& e);
};

using PolyEdgeAdder = void(*)(Polygon&, PolyEdge*);

struct PolyEdge : public Edge<Polygon, GeomEdge*> {
    using NodeT = Polygon;

    PolyEdge(Polygon& s, Polygon& e, PolyEdgeAdder sadder, PolyEdgeAdder eadder, GeomEdge* ge);
};

// Edges ordered by rotation from (1, 0)
struct GeomNode : public Node<Point, GeomEdge> {
    Point& pos;
    explicit GeomNode(const Point& p) : Node(p), pos(data) {}
    void insertEdge(GeomEdge*);
    bool compare(const GeomEdge* one, const GeomEdge* other);
};

struct Polygon : public Node<std::string, PolyEdge> {
    std::string& name;
    explicit Polygon(const std::string& n) : Node(n), name(data) {}
};

struct PolygonBuildStep {
    const GeomNode& next;
    const GeomEdge* from;
    Polygon* outgoing_left_poly;
    PolyEdgeAdder outgoing_left_edge_adder;
    Polygon* outgoing_rigth_poly;
    PolyEdgeAdder outgoing_right_edge_adder;
};

// ****************DEFINITIONS******************

void addPolyEdgeToBack(Polygon& poly, PolyEdge* e) {
    poly.edges.emplace_back(e);
}

void addPolyEdgeToFront(Polygon& poly, PolyEdge* e) {
    poly.edges.emplace_front(e);
}

void buildPolygonEdges(std::list<Polygon> polys, std::queue<PolygonBuildStep> steps) {

}

Vector GeomEdge::outwardDirFrom(const GeomNode& n) const {
    if (n == start) return dir;
    return dir.flipped();
}

GeomEdge::GeomEdge(GeomNode& s, GeomNode& e) : Edge(s, e, Vector(s.pos, e.pos).normalized()), dir(assoc) {
    start.insertEdge(this);
    end.insertEdge(this);
}


PolyEdge::PolyEdge(Polygon& s, Polygon& e, PolyEdgeAdder sadder, PolyEdgeAdder eadder, GeomEdge* ge) :
    Edge(s, e, std::move(ge)) {
    sadder(s, this);
    eadder(e, this);
}

bool GeomNode::compare(const GeomEdge* one, const GeomEdge* other) {
    return ccwLessNormalized(one->outwardDirFrom(*this), other->outwardDirFrom(*this));
}

void GeomNode::insertEdge(GeomEdge* e) {
    auto it = edges.begin();
    while (it != edges.end() && compare(*it, e)) {
        ++it;
    }
    edges.emplace(it, e);
}

